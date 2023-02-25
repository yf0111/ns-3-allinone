#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <float.h>
#include <map>

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
    precalculation(RF_AP_node,VLC_AP_nodes, UE_nodes,
                   VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                   RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                   my_UE_list);
#if LASINR
    /*
        ref'2 , LA-SINR
    */
    LA_SINR(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list);
#endif // LASINR

#if LAEQOS
    /*
        ref'2 , LA-EQOS
    */
    LA_EQOS(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list);
#endif // LAEQOS

#if RLLB
    /*
        ref'1 , RLLB
    */
    RL_LB(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix);
#endif // RLLB

}

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list)
{

    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;


/*  Standalone LiFi */
/*
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        local_AP_association_matrix[0][UE_index] = 0;
        int max_AP_index = 0;
        double max_AP_value = -DBL_MAX;
        for(int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            if(VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_value){
                max_AP_index = VLC_AP_index + 1;
                max_AP_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
            }
        }
        if(max_AP_index != 0){
            local_AP_association_matrix[max_AP_index][UE_index] = 1;
        }
    }
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
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        double data_rate = 0.0;
        for(int AP_index = 0 ;AP_index < RF_AP_num+VLC_AP_num ;AP_index ++){
            if(local_AP_association_matrix[AP_index][UE_index] == 1){ // UE is RF
                if(AP_index != 0){
                    int nums = AP_serve_UE_numbers[AP_index];
                    data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[AP_index - 1][UE_index]) * VLC_AP_bandwidth;
                }
            }
        }
        UE_final_data_rate_vector[UE_index] = data_rate;
    }
*/

/*  Standalone WiFi */
/*
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        local_AP_association_matrix[0][UE_index] = 1;
        double data_rate = (1.0 / UE_num ) * getSpectralEfficiency(RF_SINR_vector[UE_index]) * RF_AP_bandwidth;
        UE_final_data_rate_vector[UE_index] = data_rate;
    }
*/


/*   Hybrid   */
/*
    for(int UE_index = 0; UE_index < UE_num ; UE_index ++){
        int max_AP_index = 0;
        double max_AP_value = -DBL_MAX;
        for(int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            if(VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_value){
                max_AP_index = VLC_AP_index + 1;
                max_AP_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
            }
        }
        if(RF_SINR_vector[UE_index] > max_AP_value){
            max_AP_index = 0;
            max_AP_value = RF_SINR_vector[UE_index];
        }
        local_AP_association_matrix[max_AP_index][UE_index] = 1;
    }
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

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //data rate : g * ρ * η * bandwidth
        double data_rate = 0.0;
        for(int AP_index = 0 ;AP_index < RF_AP_num+VLC_AP_num ;AP_index ++){
            if(local_AP_association_matrix[AP_index][UE_index] == 1){ // UE is RF
                if(AP_index == 0){
                    int nums = AP_serve_UE_numbers[AP_index];
                    data_rate = (1.0/nums) * getSpectralEfficiency(RF_SINR_vector[UE_index]) * RF_AP_bandwidth;
                }
                else{ // UE is VLC
                    int nums = AP_serve_UE_numbers[AP_index];
                    data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[AP_index - 1 ][UE_index]) * VLC_AP_bandwidth;
                }
            }
        }
        UE_final_data_rate_vector[UE_index] = data_rate;
    }
*/


/*  LASINR  */

    // step 1 : AP association using stand alone WiFi formula (9) and stand alone LiFi formula (11)
    for(int i = 0 ; i < UE_num ; i++){
        local_AP_association_matrix[0][i] = 1 ;
    }

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        int max_AP_index = 0;
        double max_AP_SINR_value = -DBL_MAX;
        for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            if(VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_SINR_value){
                max_AP_SINR_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
                max_AP_index = VLC_AP_index+1;
            }
        }
        local_AP_association_matrix[max_AP_index][UE_index] = 1;
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

    // step 2 : calculate UE can get data rate using (13)
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        double RF_data_rate = 0.0;
        double VLC_data_rate = 0.0;
        for(int AP_index = 0; AP_index < RF_AP_num+VLC_AP_num; AP_index++){
            //data rate : g * ρ * η * bandwidth
            if(local_AP_association_matrix[AP_index][UE_index] == 1){
                if(AP_index == 0){ // RF
                    double nums = 1.0 / UE_num ;
                    RF_data_rate = nums * getSpectralEfficiency(RF_SINR_vector[UE_index]) * RF_AP_bandwidth;
                }
                else{ // VLC
                    double nums = 1.0 / AP_serve_UE_numbers[AP_index];
                    VLC_data_rate = nums * getSpectralEfficiency(VLC_SINR_matrix[AP_index-1][UE_index]) * VLC_AP_bandwidth;
                }
            }
        }
        double total = (RF_data_rate + VLC_data_rate) * la_overhead;
        UE_final_data_rate_vector[UE_index] = (RF_data_rate + VLC_data_rate) * la_overhead;
    }


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

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        int max_AP_index = 0;
        double max_AP_SINR_value = -DBL_MAX;
        for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            if(VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_SINR_value){
                max_AP_SINR_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
                max_AP_index = VLC_AP_index+1;
            }
        }
        local_AP_association_matrix[max_AP_index][UE_index] = 1;
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

    // step 3 : decide LA users that having lowest QoS
    // QoS : QoS in ref2. is defined as the minimum data rate a particular user can achieve.
    std::vector<double> UE_final_data_rate_vector_use = UE_final_data_rate_vector;
    for(int i = 0 ; i < LA_UE_num ; i++){
        int minPosition = min_element(UE_final_data_rate_vector_use.begin(),UE_final_data_rate_vector_use.end()) - UE_final_data_rate_vector_use.begin();
        local_AP_association_matrix[0][minPosition] = 1;
        UE_final_data_rate_vector_use[minPosition] = DBL_MAX;
    }

    // step 3.5 : calculate the numbers of users served by the AP
    AP_serve_UE_numbers.clear();
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
                else{
                    int nums = AP_serve_UE_numbers[AP_index];
                    vlc_data_rate = (1.0/nums) * getSpectralEfficiency(VLC_SINR_matrix[AP_index-1][UE_index]) * VLC_AP_bandwidth; //VLC
                }
            }
        }
        double final_data_rate = (rf_data_rate + vlc_data_rate)* la_overhead;
        UE_final_data_rate_vector[UE_index] = final_data_rate;
    }



    updateApAssociationResult(local_AP_association_matrix,AP_association_matrix,my_UE_list);

}

double getSpectralEfficiency(double SINR){
    auto it = SINR_to_spectral_efficiency.lower_bound(SINR);
    return it->second;
}

void RL_LB(std::vector<std::vector<int>> &AP_association_matrix,
           std::vector<double> &RF_SINR_vector,
           std::vector<std::vector<double>> &VLC_SINR_matrix)
{
    /* state
        1. SNR between the user and various APs (UE_num x 3 , entry is Wifi SNR + highest VLC SINR + second VLC SINR)
        2. Current load on each AP (AP_num , entry is number of users connected to AP <int>)
    */

    /* action
        0. WiFi AP
        1. LiFi AP 0 (highest SINR)
        2. LiFi AP 1 (second SINR)
        3. WiFi + LiFi AP (highest SINR)
        4. WiFi + LiFi AP (second SINR)
    */

    /*  state  */
    std::vector<std::vector<double>> SINR_matrix = combineSINRmatrix(VLC_SINR_matrix,RF_SINR_vector); //  (RF + VLC SINR) x UE
    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;

    std::vector<std::vector<double>> State_SINR = SINR_matrix_to_AP_index(SINR_matrix); // UE x 3
    std::vector<int> State_AP_load = AP_association_matrix_to_UE_numbers(local_AP_association_matrix);

    /*  set UE type  */
    std::vector<int> UE_type = std::vector<int>(UE_num,0); // 1 for SAP , 2 for LA
    for(int i = 0 ; i < UE_num ; i++){
        if( i < LA_UE_num)
            UE_type[i] = 2;
        else
            UE_type[i] = 1;
    }

    /* std::vector<int> AP_association (ue)
         0 : means that this UE connected to WiFi
         1 : means that this UE connected to LiFi1
         2 : means that this UE connected to LiFi2
         3 : means that this UE connected to LiFi3
         4 : means that this UE connected to LiFi4
         5 : means that this UE connected to WiFi and LiFi1
         6 : means that this UE connected to WiFi and LiFi2
         7 : means that this UE connected to WiFi and LiFi3
         8 : means that this UE connected to WiFi and LiFi4
         -1 (default) : means that no AP association for this UE
    */

}

std::vector<int> AP_association_matrix_to_UE_numbers(std::vector<std::vector<int>> &AP_association_matrix){
    std::vector<int> AP_serve_UE_numbers = std::vector<int>(RF_AP_num + VLC_AP_num , 0);
    for(int i = 0 ; i < RF_AP_num + VLC_AP_num ; i++){
        int served_UE_number = 0;
        for(int j = 0 ; j < UE_num ; j++){
            if(AP_association_matrix[i][j] == 1){
                served_UE_number += 1;
            }
        }
        AP_serve_UE_numbers[i] = served_UE_number;
    }
    return AP_serve_UE_numbers;
}

std::vector<std::vector<double>> combineSINRmatrix(std::vector<std::vector<double>> &VLC_SINR_matrix,std::vector<double> &RF_SINR_vector){
    std::vector<std::vector<double>> combined_SINR_matrix = std::vector<std::vector<double>> (RF_AP_num+VLC_AP_num,std::vector<double>(UE_num,0.0));
    for(int i = 0; i < RF_AP_num + VLC_AP_num ; i++){
        if( i < RF_AP_num)
            combined_SINR_matrix[i] = RF_SINR_vector;
        else
            combined_SINR_matrix[i] = VLC_SINR_matrix[i-1];
    }
    return combined_SINR_matrix;
}

std::vector<std::vector<double>> SINR_matrix_to_AP_index(std::vector<std::vector<double>> &SINR_matrix){
    std::vector<std::vector<double>> State_SINR = std::vector<std::vector<double>> (UE_num,std::vector<double>(3,0.0));
    for(int i = 0 ; i < UE_num ; i++){
        double highest_AP_value = -DBL_MAX ;
        double second_AP_value = -DBL_MAX ;
        for(int j = 1 ; j < VLC_AP_num + 1 ; j++){
            if(SINR_matrix[j][i] > highest_AP_value){
                second_AP_value = highest_AP_value;
                highest_AP_value = SINR_matrix[j][i];
            }
        }
        State_SINR[i][0] = SINR_matrix[0][i];
        State_SINR[i][1] = (highest_AP_value < 0 )? 0.0 : highest_AP_value;
        State_SINR[i][2] = (second_AP_value < 0 )? 0.0 : second_AP_value;
    }
    return State_SINR;
}

double calculatedR1(std::vector<int> &UE_type,
                 std::vector<int> &pre_AP_association,
                 std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,
                 std::vector<double> &RF_data_rate_vector,
                 std::vector<int> State_AP_load){
    double total_throughput = 0.0;
    for(int i = 0 ;i < UE_num ; i++){
        double throughput = 0.0;
        if(UE_type[i] == 1){ //SAP
            if(AP_association[i] != -1){
                double eta = 0.0;
                if(pre_AP_association[i] == AP_association[i]){ // no change AP
                    eta = 1;
                }
                else if(pre_AP_association[i] != 0 && AP_association[i] != 0){ // LiFi to LiFi
                    eta = eta_hho;
                }
                else if(pre_AP_association[i] == 0 && AP_association[i] != 0){ // WiFi to LiFi
                    eta = eta_vho;
                }
                double data_rate = 0.0;
                if(AP_association[i] < 5)
                    data_rate = (AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[AP_association[i]][i];
                else
                    std::cout << "SAP UE AP association is wrong!\n";
                throughput = eta * data_rate * (1.0 / State_AP_load[AP_association[i]]);
            }
        }
        else if(UE_type[i] == 2){ //LA
            if(AP_association[i] != -1){
                double eta = 0.0;
                if(pre_AP_association[i] == AP_association[i]){ // no change AP
                    eta = 1;
                }
                else if(pre_AP_association[i] > 4 && AP_association[i] > 4){ // WiFi same , LiFi change
                    eta = eta_hho;
                }
                else{
                    eta = eta_vho;
                }
                if(AP_association[i] > 4){
                    throughput = eta * (RF_data_rate_vector[i] * (1.0 / State_AP_load[0])) + (VLC_data_rate_matrix[AP_association[i]-4][i] * (1.0 / State_AP_load[AP_association[i]-4]));
                }
                else{
                    throughput = eta * (AP_association[i] == 0)? RF_data_rate_vector[i] * (1.0 / State_AP_load[0]) : VLC_data_rate_matrix[AP_association[i]][i] * (1.0 / State_AP_load[AP_association[i]]);
                }
            }
        }
        total_throughput += throughput;
    }
    return total_throughput / UE_num;
}

double calculatedR2(std::vector<int> &UE_type,
                 std::vector<int> &pre_AP_association,
                 std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,
                 std::vector<double> &RF_data_rate_vector,
                 std::vector<int> State_AP_load){
    double total_throughput = 0.0;
    for(int i = 0 ;i < UE_num ; i++){
        double throughput = 0.0;
        if(UE_type[i] == 1){ //SAP
            if(AP_association[i] != -1){
                double eta = 0.0;
                if(pre_AP_association[i] == AP_association[i]){ // no change AP
                    eta = 1;
                }
                else if(pre_AP_association[i] != 0 && AP_association[i] != 0){ // LiFi to LiFi
                    eta = eta_hho;
                }
                else if(pre_AP_association[i] == 0 && AP_association[i] != 0){ // WiFi to LiFi
                    eta = eta_vho;
                }
                double data_rate = 0.0;
                if(AP_association[i] < 5)
                    data_rate = (AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[AP_association[i]][i];
                else
                    std::cout << "SAP UE AP association is wrong!\n";
                throughput = eta * data_rate * (1.0 / State_AP_load[AP_association[i]]);
            }
        }
        else if(UE_type[i] == 2){ //LA
            if(AP_association[i] != -1){
                double eta = 0.0;
                if(pre_AP_association[i] == AP_association[i]){ // no change AP
                    eta = 1;
                }
                else if(pre_AP_association[i] > 4 && AP_association[i] > 4){ // WiFi same , LiFi change
                    eta = eta_hho;
                }
                else{
                    eta = eta_vho;
                }
                if(AP_association[i] > 4){
                    throughput = eta * (RF_data_rate_vector[i] * (1.0 / State_AP_load[0])) + (VLC_data_rate_matrix[AP_association[i]-4][i] * (1.0 / State_AP_load[AP_association[i]-4]));
                }
                else{
                    throughput = eta * (AP_association[i] == 0)? RF_data_rate_vector[i] * (1.0 / State_AP_load[0]) : VLC_data_rate_matrix[AP_association[i]][i] * (1.0 / State_AP_load[AP_association[i]]);
                }
            }
        }
        total_throughput += throughput;
    }
    return total_throughput / UE_num;
}

void updateApAssociationResult(std::vector<std::vector<int>> &local_AP_sssociation_matrix,
                               std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list){
    AP_sssociation_matrix = local_AP_sssociation_matrix;

    // update every myUeNode
    for (int i = 0; i < local_AP_sssociation_matrix.size(); i++) {
        for (int j = 0; j < local_AP_sssociation_matrix[0].size(); j++) {
            if (local_AP_sssociation_matrix[i][j] == 1)
                my_UE_list[j].setCurrAssociatedAP(i);
        }
    }
}

void updateResourceAllocationResult(std::vector<std::vector<double>> &throughtput_per_iteration, std::vector<MyUeNode> &my_UE_list){
    for (int i = 0; i < my_UE_list.size(); i++) {
        double curr_throughput = throughtput_per_iteration[i].back();
        double curr_satisfaction = std::min(curr_throughput / my_UE_list[i].getRequiredDataRate(), 1.0);

        my_UE_list[i].addThroughput(curr_throughput);
        my_UE_list[i].addSatisfaction(curr_satisfaction);
    }
}
