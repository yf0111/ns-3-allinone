#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>

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
        calculate VLC LOS and VLC SINR and RF LOS and RF SINR
    */
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix, RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector, my_UE_list);

    /*
        algorithm 1 , LA-SINR
    */
    //LA_SINR(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list);

    /*
        algorithm 2 , LA-EQOS
    */
    LA_EQOS(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list);

}

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list)
{

    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;

    // step 1 : AP association using stand alone WiFi formula (9) and stand LiFi formula (11)
    for(int i = 0 ; i < UE_num ; i++){
        local_AP_association_matrix[0][i] = 1 ;
    }

    for(int i = 0 ; i < UE_num ; i++){
        int max_AP_index = 0;
        int max_AP_SINR_value = INT_MIN;
        for(int j = 0 ; j < VLC_AP_num ; j++){
            if(VLC_SINR_matrix[j][i] > max_AP_SINR_value){
                max_AP_SINR_value = VLC_SINR_matrix[j][i];
                max_AP_index = j+1;
            }
        }
        local_AP_association_matrix[max_AP_index][i] = 1;
    }

/*#if DEBUG_MODE

    std::cout<<"\n after LA-SINR : \n"<<std::endl;
    printApAssociationMatrix(local_AP_association_matrix);

#endif // DEBUG_MODE*/

    // step 1.5 : calculate the numbers of users served by the AP
    std::vector<int> AP_serve_UE_numbers(RF_AP_num+VLC_AP_num,0); // AP_serve_UE_numbers[0] is RF AP

    for(int i = 0 ; i < RF_AP_num+VLC_AP_num ; i++){
        int served_UE_number = 0;
        for(int j = 0 ; j < UE_num ; j++){
            if(local_AP_association_matrix[i][j] == 1){
                served_UE_number += 1;
            }
        }
        AP_serve_UE_numbers[i] = served_UE_number;
    }

#if DEBUG_MODE

    printAPServeUENum(AP_serve_UE_numbers);

#endif // DEBUG_MODE

    // step 2 : calculate UE can get data rate using (13)
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        double vlc_data_rate = 0;
        int nums = AP_serve_UE_numbers[0];
        double rf_data_rate = (1.0/nums) * getSpectralEfficiency(RF_SINR_vector[UE_index])* RF_AP_bandwidth; // RF
        for(int VLC_AP_index = 1 ; VLC_AP_index < RF_AP_num + VLC_AP_num ; VLC_AP_index++){
            if(local_AP_association_matrix[VLC_AP_index][UE_index] == 1){
                int nums = AP_serve_UE_numbers[VLC_AP_index];
                vlc_data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index-1][UE_index]) * VLC_AP_bandwidth; // VLC
            }
        }
        double final_data_rate = (rf_data_rate + vlc_data_rate)* la_overhead;
        UE_final_data_rate_vector[UE_index] = final_data_rate;
    }

#if DEBUG_MODE

    printUEFinalDataRate(UE_final_data_rate_vector);

#endif // DEBUG_MODE

    updateApAssociationResult(local_AP_association_matrix,AP_association_matrix,my_UE_list);

}


void LA_EQOS(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list)
{
    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;

    // step 1 : AP association using stand LiFi formula (11)
    for(int i = 0 ; i < UE_num ; i++){
        int max_AP_index = 0;
        int max_AP_SINR_value = INT_MIN;
        for(int j = 0 ; j < VLC_AP_num ; j++){
            if(VLC_SINR_matrix[j][i] > max_AP_SINR_value){
                max_AP_SINR_value = VLC_SINR_matrix[j][i];
                max_AP_index = j+1;
            }
        }
        local_AP_association_matrix[max_AP_index][i] = 1;
    }

    // step 1.5 : calculate the numbers of users served by the AP
    std::vector<int> AP_serve_UE_numbers(RF_AP_num+VLC_AP_num,0); // AP_serve_UE_numbers[0] is RF AP

    for(int i = 0 ; i < RF_AP_num+VLC_AP_num ; i++){
        int served_UE_number = 0;
        for(int j = 0 ; j < UE_num ; j++){
            if(local_AP_association_matrix[i][j] == 1){
                served_UE_number += 1;
            }
        }
        AP_serve_UE_numbers[i] = served_UE_number;
    }


    // step 2 : calculate data rate using stand LiFi formula (12)
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        double vlc_data_rate = 0;
        for(int VLC_AP_index = 1 ; VLC_AP_index < RF_AP_num + VLC_AP_num ; VLC_AP_index++){
            if(local_AP_association_matrix[VLC_AP_index][UE_index] == 1){
                int nums = AP_serve_UE_numbers[VLC_AP_index];
                vlc_data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index-1][UE_index]) * VLC_AP_bandwidth; // VLC
            }
        }
        UE_final_data_rate_vector[UE_index] = vlc_data_rate;
    }
    AP_serve_UE_numbers.clear();

    // step 3 : decide LA users that having lowest QoS
    // QoS : QoS in ref2. is defined as the minimum data rate a particular user can achieve.
    std::vector<double> UE_final_data_rate_vector_sorted = UE_final_data_rate_vector;
    sort(UE_final_data_rate_vector_sorted.begin(),UE_final_data_rate_vector_sorted.end());
    std::vector<int> indicate_seleced(UE_num,0);
    for(int i = 0 ; i < LA_UE_num ; i++){
        for(int j = 0 ; j < UE_num ; j++){
            if(UE_final_data_rate_vector[j] == UE_final_data_rate_vector_sorted[i] && indicate_seleced[j] == 0){
                local_AP_association_matrix[0][j] = 1;
                indicate_seleced[j] = 1;
                break;
            }
        }
    }

    // step 3.5 : calculate the numbers of users served by the AP
    for(int i = 0 ; i < RF_AP_num+VLC_AP_num ; i++){
        int served_UE_number = 0;
        for(int j = 0 ; j < UE_num ; j++){
            if(local_AP_association_matrix[i][j] == 1){
                served_UE_number += 1;
            }
        }
        AP_serve_UE_numbers[i] = served_UE_number;
    }

    // step 4 : calculate LA user's data rate using (14)
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        double vlc_data_rate = 0;
        double rf_data_rate = 0;
        for(int AP_index = 0 ; AP_index < RF_AP_num+VLC_AP_num ; AP_index++){
            if(local_AP_association_matrix[AP_index][UE_index] == 1){
                if(AP_index == 0 ){
                    int nums = AP_serve_UE_numbers[AP_index];
                    rf_data_rate = (1.0/nums) * getSpectralEfficiency(RF_SINR_vector[UE_index]) * RF_AP_bandwidth; //RF
                }
                if(AP_index > 0 ){
                    int nums = AP_serve_UE_numbers[AP_index];
                    vlc_data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[AP_index-1][UE_index]) * VLC_AP_bandwidth; //VLC
                }
            }
        }
        double final_data_rate = (rf_data_rate + vlc_data_rate)* la_overhead;
        UE_final_data_rate_vector[UE_index] = final_data_rate;
    }
#if DEBUG_MODE

    printUEFinalDataRate(UE_final_data_rate_vector);

#endif // DEBUG_MODE

    updateApAssociationResult(local_AP_association_matrix,AP_association_matrix,my_UE_list);
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
