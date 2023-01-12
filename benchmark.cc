#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>


#include "print.h"
#include "channel.h"
#include "benchmark.h"
#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"

/*
    table of conversion from SINR to spectral efficiency

    bit/s/Hz = 1 Mbit/s/MHz
*/
static const std::map<double, double, std::greater<double>> SINR_to_spectral_efficiency = { {0.0, 0}, {2.0, 0.5}, {4.0, 0.75}, {5.0, 1.0},
                                                                                             {9.0, 1.5}, {11.0, 2.0}, {15.0, 3.0}, {18.0, 4.0},
                                                                                             {20.0,5.0}};

/*std::vector<int> initializedStep(std::vector<double> &RF_data_rate_vector,
                                std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                std::vector<std::vector<int>> &local_AP_association_matrix,
                                std::vector<std::vector<double>> &throughtput_per_iteration){
    // step1: construct strategy sets of the form {RF_AP, best VLC_AP} for all UEs
    std::vector<std::vector<int>> strategy_set (UE_num, std::vector<int> (2, -1)); // {{RF_AP_idx, best VLC_AP_idx}, {RF_AP_idx, best VLC_AP_idx}, ...}

    for (int i = 0; i < UE_num; i++) {
        strategy_set[i][0] = 0; // strategy set always contains the RF AP

        int best_VLC_AP = 0;
        double max_data_rate = -1;

        for (int j = 0; j < VLC_AP_num; j++) {
            double total_link_data_rate = 0.0;

            for (int k = 1; k < effective_subcarrier_num + 1; k++)
                total_link_data_rate += VLC_data_rate_matrix[j][i][k];

            total_link_data_rate *= time_slot_num;

            if (total_link_data_rate > max_data_rate) {
                max_data_rate = total_link_data_rate;
                best_VLC_AP = j;
            }
        }
        strategy_set[i][1] = best_VLC_AP + RF_AP_num; // VLC APs are indexed starting from 1
    }

    // step2: UE are randomly allocated to an AP from their strategy set (APA)
    std::vector<int> connected_AP (UE_num, -1); // for output
    std::vector<std::vector<int>> serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ()); // record the UEs served by a AP

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0, 1);

    // clear the local_AP_association_matrix
    for (int i = 0; i < local_AP_association_matrix.size(); i++)
        for (int j = 0; j < local_AP_association_matrix[0].size(); j++)
            local_AP_association_matrix[i][j] = 0;

    for (int i = 0; i < UE_num; i++) {
        int chosen_AP = strategy_set[i][distribution(generator)];

        local_AP_association_matrix[chosen_AP][i] = 1;
        connected_AP[i] = chosen_AP;
        serving_UE[chosen_AP].push_back(i);
    }


    //step3: resource allocation
    // if connected to RF AP -> equally divide RF AP resource
    for (int i = 0; i < RF_AP_num; i++) {
        for (int j = 0; j < serving_UE[i].size(); j++) {
            throughtput_per_iteration[serving_UE[i][j]].push_back(RF_data_rate_vector[serving_UE[i].size()]);
        }
    }

    // if connected to VLC AP -> OFDMA
    for (int i = RF_AP_num; i < VLC_AP_num + RF_AP_num; i++) {
        std::vector<double> data_rate = OFDMA(i-RF_AP_num, serving_UE[i], VLC_data_rate_matrix);

        for (int j = 0; j < serving_UE[i].size(); j++)
            throughtput_per_iteration[serving_UE[i][j]].push_back(data_rate[j]);
    }

    return connected_AP;
}
*/

void benchmarkMethod(int &state,
                       NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<std::vector<double>> &VLC_LOS_matrix,
                       std::vector<std::vector<double>> &VLC_SINR_matrix,
                       std::vector<std::vector<double>> &VLC_data_rate_matrix,
                       std::vector<double> &RF_channel_gain_vector,
                       std::vector<double> &RF_SINR_vector,
                       std::vector<double> &RF_data_rate_vector,
                       std::vector<std::vector<int>> &AP_association_matrix,
                       std::vector<MyUeNode> &my_UE_list,
                       std::vector<double> &UE_final_data_rate_vector)
{
    /*
        calculate VLC LOS and VLC SINR / RF LOS and RF SINR
    */
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix, RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector, my_UE_list);
    LA_SINR(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector);

}

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector)
{

    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;

    for(int i = 0;i < UE_num;i++){
        local_AP_association_matrix[0][i] = 1 ;
    }

    for(int i=0;i<UE_num;i++){
        int max_AP_index = 0;
        int max_AP_SINR_value = INT_MIN;
        for(int j=1;j<VLC_AP_num;j++){
            if(VLC_SINR_matrix[j][i] > max_AP_SINR_value){
                max_AP_SINR_value = VLC_SINR_matrix[j][i];
                max_AP_index = j;
            }
        }
        local_AP_association_matrix[max_AP_index][i] = 1;
    }

/*#if DEBUG_MODE

    std::cout<<"\n after LA-SINR : \n"<<std::endl;
    printApAssociationMatrix(local_AP_association_matrix);

#endif // DEBUG_MODE*/

    std::vector<int> AP_serve_UE_numbers(RF_AP_num+VLC_AP_num,0); // AP_serve_UE_numbers[0] is RF AP

    for(int i = 0;i<RF_AP_num+VLC_AP_num;i++){
        int served_UE_number = 0;
        for(int j=0;j<UE_num;j++){
            if(local_AP_association_matrix[i][j] == 1){
                served_UE_number += 1;
            }
        }
        AP_serve_UE_numbers[i] = served_UE_number;
    }

/*#if DEBUG_MODE

    printAPServeUENum(AP_serve_UE_numbers);

#endif // DEBUG_MODE*/

    for(int UE_index = 0;UE_index < UE_num;UE_index++){
        //data rate : g * ρ * η * bandwidth
        double vlc_data_rate = 0;
        double rf_data_rate = (1.0/UE_num) * getSpectralEfficiency(RF_SINR_vector[UE_index])* RF_AP_bandwidth; // RF
        for(int AP_index = 1 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
            if(local_AP_association_matrix[AP_index][UE_index] == 1){
                int nums = AP_serve_UE_numbers[AP_index];
                vlc_data_rate = (1/nums) * getSpectralEfficiency(VLC_SINR_matrix[AP_index][UE_index]) * VLC_AP_bandwidth; // VLC
            }
        }
        double final_data_rate = (rf_data_rate + vlc_data_rate)* la_overhead;
        UE_final_data_rate_vector[UE_index] = final_data_rate;
    }

#if DEBUG_MODE

    printUEFinalDataRate(UE_final_data_rate_vector);

#endif // DEBUG_MODE

}

double getSpectralEfficiency(double SINR){
    auto it = SINR_to_spectral_efficiency.lower_bound(SINR);
    return it->second;
}

/*std::vector<int> initializedStep(std::vector<std::vector<double>> VLC_SINR_matrix,
                                 std::vector<std::vector<double>> RF_SINR_vector){

}*/


void updateApAssociationResult(std::vector<std::vector<int>> &local_AP_sssociation_matrix,
                               std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list)
{
    AP_sssociation_matrix = local_AP_sssociation_matrix;

    // update every myUeNode
    for (int i = 0; i < local_AP_sssociation_matrix.size(); i++) {
        for (int j = 0; j < local_AP_sssociation_matrix[0].size(); j++) {
            if (local_AP_sssociation_matrix[i][j] == 1)
                my_UE_list[j].setCurrAssociatedAP(i);
        }
    }
}


void updateResourceAllocationResult(std::vector<std::vector<double>> &throughtput_per_iteration, std::vector<MyUeNode> &my_UE_list)
{
    for (int i = 0; i < my_UE_list.size(); i++) {
        double curr_throughput = throughtput_per_iteration[i].back();
        double curr_satisfaction = std::min(curr_throughput / my_UE_list[i].getRequiredDataRate(), 1.0);

        my_UE_list[i].addThroughput(curr_throughput);
        my_UE_list[i].addSatisfaction(curr_satisfaction);
    }
}
