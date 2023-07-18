#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>
#include <float.h>
#include <map>
#include <set>
#include <cstdlib>
#include <ctime>


#include "print.h"
#include "channel.h"
#include "benchmark.h"
#include "my_UE_node.h"
#include "proposed_method.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"

/*
    table of conversion from SINR to spectral efficiency

    bit/s/Hz = 1 Mbit/s/MHz
*/

int flag = 0;
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
                     std::vector<double> &UE_final_data_rate_vector,
                     std::vector<double> &UE_final_satisfaction_vector,
                     std::vector<double> &UE_require_data_rate)
{
    precalculation(RF_AP_node,VLC_AP_nodes, UE_nodes,
                   VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                   RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                   my_UE_list);
#if LASINR
    /*
        ref'2 , LA-SINR
    */
    LA_SINR(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list,UE_final_satisfaction_vector,UE_require_data_rate);
#endif // LASINR

#if LAEQOS
    /*
        ref'2 , LA-EQOS
    */
    LA_EQOS(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list,UE_final_satisfaction_vector,UE_require_data_rate);

#endif // LAEQOS

}

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<double> &UE_final_satisfaction_vector,
             std::vector<double> &UE_require_data_rate)
{

    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;
    UE_require_data_rate = createUEDemandVector(my_UE_list);

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

    AP_association_matrix = local_AP_association_matrix;
    updateApAssociationResult(local_AP_association_matrix,my_UE_list);

    // calculate user satisfaction
    std::vector<double> US_reliability = std::vector<double>(UE_num,0.0);
    std::vector<double> US_latency = std::vector<double> (UE_num,0.0);
    std::vector<double> US_datarate = std::vector<double> (UE_num,0.0);
    cal_US_Reliability(RF_SINR_vector,VLC_SINR_matrix,US_reliability,AP_association_matrix);
    cal_US_Latency(US_latency,UE_final_data_rate_vector);
    cal_US_DataRate(UE_final_data_rate_vector,UE_require_data_rate,US_datarate,US_latency,US_reliability,my_UE_list);

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(UE_final_data_rate_vector[UE_index] < UE_require_data_rate[UE_index]){
            UE_final_satisfaction_vector[UE_index] = 0;
        }
        else{
            UE_final_satisfaction_vector[UE_index] = US_datarate[UE_index];
        }
    }
}


void LA_EQOS(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<double> &UE_final_satisfaction_vector,
             std::vector<double> &UE_require_data_rate)
{
    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;
    UE_require_data_rate = createUEDemandVector(my_UE_list);

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
    updateApAssociationResult(local_AP_association_matrix,my_UE_list);

}

double getSpectralEfficiency(double SINR){
    auto it = SINR_to_spectral_efficiency.lower_bound(SINR);
    return it->second;
}

void RL_LB(std::vector<std::vector<int>> &AP_association_matrix,
           std::vector<double> &RF_SINR_vector,
           std::vector<std::vector<double>> &VLC_SINR_matrix,
           std::vector<MyUeNode> &my_UE_list,
           std::vector<std::vector<double>> &VLC_data_rate_matrix,
           std::vector<double> &RF_data_rate_vector,
           std::multimap<std::vector<double>,int> &policy_map,
           std::vector<double> &UE_final_data_rate_vector,
           std::vector<double> &UE_final_satisfaction_vector)
{
    srand(time(NULL));
    /* state
        1. SNR between the user and various APs (UE_num x 3 , entry is Wifi SNR + highest VLC SINR + second VLC SINR)
        2. Current load on each AP (AP_num , entry is number of users connected to AP <int>)

       action
        0. WiFi AP
        1. LiFi AP 0 (highest SINR)
        2. LiFi AP 1 (second SINR)
        3. WiFi + LiFi AP (highest SINR)
        4. WiFi + LiFi AP (second SINR)
    */

    /*  init state  */
    std::vector<std::vector<double>> SINR_matrix = combineSINRmatrix(VLC_SINR_matrix,RF_SINR_vector); //  (RF + VLC SINR) x UE
    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;
    std::vector<std::vector<double>> State_SINR = SINR_matrix_to_two_high_SINR(SINR_matrix); // UE x 3
    std::vector<std::vector<int>> State_SINR_VLC_index = SINR_matrix_to_two_high_AP_index(SINR_matrix); // UE x 2
    /* State_SINR
        UE[0] WiFi SNR, LiFi SINR, LiFi SINR
        UE[1] WiFi SNR, LiFi SINR, LiFi SINR
        UE[2] WiFi SNR, LiFi SINR, LiFi SINR
        ...
        UE[UE_num - 1] WiFi SNR, LiFi SINR, LiFi SINR
    */

    /*  init UE type  */
    std::vector<int> UE_type = std::vector<int>(UE_num,0); // 1 for SAP , 2 for LA
    for(int i = 0 ; i < UE_num ; i++){
        if( i < LA_UE_num)
            UE_type[i] = 2;
        else
            UE_type[i] = 1;
    }

    /* init UE demand */
    std::vector<double> UE_demand = createUEDemandVector(my_UE_list);

    /*  init AP assignment
            std::vector<int> AP_association (ue)
             0 : means that this UE connected to WiFi
             1 : means that this UE connected to LiFi0
             2 : means that this UE connected to LiFi1
             3 : means that this UE connected to LiFi2
             4 : means that this UE connected to LiFi3
             5 : means that this UE connected to WiFi and LiFi0
             6 : means that this UE connected to WiFi and LiFi1
             7 : means that this UE connected to WiFi and LiFi2
             8 : means that this UE connected to WiFi and LiFi3
             -1 (default) : means that no AP association for this UE
    */
    std::vector<int> init_AP_association = std::vector<int>(UE_num,-1);
    local_AP_association_matrix = AP_association_vector_to_matrix(init_AP_association);

    std::vector<std::vector<int>> all_possible_AP_association = generate_next_possible_AP_association(SINR_matrix,UE_type,State_SINR_VLC_index); // should do only once(?) , represent all possible action
    std::vector<double> each_APS_R1 = std::vector<double>(all_possible_AP_association.size(),0.0);
    std::vector<double> each_APS_R2 = std::vector<double>(all_possible_AP_association.size(),0.0);
    std::vector<double> each_APS_R3 = std::vector<double>(all_possible_AP_association.size(),0.0);

    for( int i = 0 ; i < all_possible_AP_association.size(); i++){
        std::vector<std::vector<int>> now_APS_matrix = AP_association_vector_to_matrix(all_possible_AP_association[i]);
        std::vector<int> State_AP_load = AP_association_matrix_to_UE_numbers (now_APS_matrix);
        each_APS_R1[i] = calculatedR1(UE_type,init_AP_association,all_possible_AP_association[i],VLC_data_rate_matrix,RF_data_rate_vector,State_AP_load);
        each_APS_R2[i] = calculatedR2(UE_type,init_AP_association,all_possible_AP_association[i],VLC_data_rate_matrix,RF_data_rate_vector,State_AP_load,UE_demand);
        each_APS_R3[i] = calculatedR3(UE_type,init_AP_association,all_possible_AP_association[i],VLC_data_rate_matrix,RF_data_rate_vector,State_AP_load,UE_demand);
    }

    std::vector<int> R1_index_sorted_index = std::vector<int>(each_APS_R1.size(),-1);
    for(int i = 0 ; i < R1_index_sorted_index.size() ; i++){
        R1_index_sorted_index[i] = i;
    }
    std::sort(R1_index_sorted_index.begin(),R1_index_sorted_index.end(),[&](const double &a , const double &b){
            return each_APS_R1[a] > each_APS_R1[b];
    });

    if(flag < 80.0 / 100.0 * state_num){
        int n = 4; // each UE save top n results
        for(int i = 0 ; i < n ; i++){
            for(int k = 0 ; k < UE_num ; k++){
                Vector UE_pos = my_UE_list[k].getPosition();
                std::vector<double> key;
                key.push_back(UE_type[k]);
                key.push_back(UE_pos.x);
                key.push_back(UE_pos.y);
                int value = all_possible_AP_association[R1_index_sorted_index[i]][k];
                policy_map.insert(std::pair<std::vector<double>,int>(key,value));
            }
        }
    }
    else{
        std::vector<int> assign_AP_association = std::vector<int> (UE_num,-5);
        for( int i = 0 ; i < UE_num ; i++){
            double distance = DBL_MAX;
            int AP_association = -5;
            if(UE_type[i] == 1){ //SAP
                for(const auto& m : policy_map){
                    if( m.first[0] == 1){
                        Vector UE_pos = my_UE_list[i].getPosition();
                        double dx = UE_pos.x - m.first[1];
                        double dy = UE_pos.y - m.first[2];
                        if(sqrt(dx*dx + dy*dy) < distance){
                            distance = sqrt(dx*dx + dy*dy);
                            AP_association = m.second;
                        }
                    }
                }
            }
            if(UE_type[i] == 2){ //LA
                for(const auto& m : policy_map){
                    if( m.first[0] == 2){
                        Vector UE_pos = my_UE_list[i].getPosition();
                        double dx = UE_pos.x - m.first[1];
                        double dy = UE_pos.y - m.first[2];
                        if(sqrt(dx*dx + dy*dy) < distance){
                            distance = sqrt(dx*dx + dy*dy);
                            AP_association = m.second;
                        }
                    }
                }
            }
            assign_AP_association[i] = AP_association;
        }

        local_AP_association_matrix = AP_association_vector_to_matrix(assign_AP_association);
        printApAssociationMatrix(local_AP_association_matrix);
        std::vector<int> State_AP_load_after_assign = AP_association_matrix_to_UE_numbers (local_AP_association_matrix);

        for(int i = 0 ; i < UE_num ; i++){
            double throughput = 0.0 , us = 0.0;
            if(UE_type[i] == 1){ //SAP
                if(assign_AP_association[i] != -1){
                    double data_rate = 0.0;
                    if(assign_AP_association[i] < 5)
                        data_rate = (assign_AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[assign_AP_association[i]-1][i];
                    else
                        std::cout<<"**(benchmark.cc) SAP UE AP association is wrong**\n";

                    double eta = 0.0;
                    if(init_AP_association[i] == assign_AP_association[i]){ // no change AP
                        eta = 1;
                    }
                    else if(init_AP_association[i] != 0 && assign_AP_association[i] != 0){ // LiFi to LiFi
                        eta = eta_hho;
                    }
                    else if(init_AP_association[i] == 0 && assign_AP_association[i] != 0){ // WiFi to LiFi
                        eta = eta_vho;
                    }
                    throughput = eta * data_rate * (1.0 / State_AP_load_after_assign[assign_AP_association[i]]);
                    us = throughput / UE_demand[i];
                }
            }
            else if(UE_type[i] == 2){ //LA
                if(assign_AP_association[i] != -1){
                    double eta = 0.0;
                    if(init_AP_association[i] == assign_AP_association[i]){ // no change AP
                        eta = 1;
                    }
                    else if(init_AP_association[i] > 4 && assign_AP_association[i] > 4){ // WiFi same , LiFi change
                        eta = eta_hho;
                    }
                    else{
                        eta = eta_vho;
                    }
                    if(assign_AP_association[i] > 4){
                        throughput = eta * ((RF_data_rate_vector[i] * (1.0 / State_AP_load_after_assign[0])) + (VLC_data_rate_matrix[assign_AP_association[i]-5][i] * (1.0 / State_AP_load_after_assign[assign_AP_association[i]-4])));
                    }
                    else{
                        throughput = (assign_AP_association[i] == 0)? eta * RF_data_rate_vector[i] * (1.0 / State_AP_load_after_assign[0]) : eta * VLC_data_rate_matrix[assign_AP_association[i]-1][i] * (1.0 / State_AP_load_after_assign[assign_AP_association[i]]);
                    }
                    us = throughput / UE_demand[i];
                }
            }
            UE_final_data_rate_vector[i] = throughput;
            UE_final_satisfaction_vector[i] = us;
        }
    }
    std::cout<< flag << "\n";
    flag++;
}

std::vector<std::vector<int>> generate_next_possible_AP_association(std::vector<std::vector<double>> &SINR_matrix,std::vector<int> &UE_type,std::vector<std::vector<int>> &State_SINR_VLC_index){ // should do only once(?)
    std::vector<std::vector<int>> all_possible_next_AP_association;
    std::vector<std::vector<int>> store_each_UE_possible_AP;
    for(int i = 0 ; i < UE_num ; i++){
        if(UE_type[i] == 1){ // SAP
            std::vector<int> v;
            v.push_back(-1); // no connected
            v.push_back(0); // wifi
            if(State_SINR_VLC_index[i][0] != -5)
                v.push_back(State_SINR_VLC_index[i][0]); // highest SINR VLC
            if(State_SINR_VLC_index[i][1] != -5)
                v.push_back(State_SINR_VLC_index[i][1]); // second SINR VLC
            store_each_UE_possible_AP.push_back(v);
        }
        if(UE_type[i] == 2){ // LA
            std::vector<int> v;
            v.push_back(-1); // no connected
            v.push_back(0); // only wifi
            if(State_SINR_VLC_index[i][0] != -5){
                v.push_back(State_SINR_VLC_index[i][0]); // highest SINR VLC
                v.push_back(State_SINR_VLC_index[i][0]+4); // wifi + highest SINR VLC
            }
            if(State_SINR_VLC_index[i][1] != -5){
                v.push_back(State_SINR_VLC_index[i][1]); // second SINR VLC
                v.push_back(State_SINR_VLC_index[i][1]+4); // wifi + highest SINR VLC
            }
            store_each_UE_possible_AP.push_back(v);
        }
    }

    /*for (int i = 0; i < store_each_UE_possible_AP.size(); i++) {

        for (auto x : store_each_UE_possible_AP[i])
            std::cout << x << " ";
        std::cout << std::endl;
    }*/

    std::vector<int> possible_AP_association = std::vector<int>(UE_num,-1);
    for(int i = 0 ;i < store_each_UE_possible_AP[0].size() ; i++){
        possible_AP_association[0] = store_each_UE_possible_AP[0][i];
        all_possible_next_AP_association.push_back(possible_AP_association);
        for(int j = 0 ; j < store_each_UE_possible_AP[1].size() ; j++){
            possible_AP_association[1] = store_each_UE_possible_AP[1][j];
            all_possible_next_AP_association.push_back(possible_AP_association);
            for(int o = 0 ; o < store_each_UE_possible_AP[2].size() ; o++){
                possible_AP_association[2] = store_each_UE_possible_AP[2][o];
                all_possible_next_AP_association.push_back(possible_AP_association);
                for(int l = 0 ; l < store_each_UE_possible_AP[3].size() ; l++){
                    possible_AP_association[3] = store_each_UE_possible_AP[3][l];
                    all_possible_next_AP_association.push_back(possible_AP_association);
                    for(int k = 0 ; k < store_each_UE_possible_AP[4].size() ; k++){
                        possible_AP_association[4] = store_each_UE_possible_AP[4][k];
                        all_possible_next_AP_association.push_back(possible_AP_association);
                        for(int m = 0 ; m < store_each_UE_possible_AP[5].size() ; m++){
                            possible_AP_association[5] = store_each_UE_possible_AP[5][m];
                            all_possible_next_AP_association.push_back(possible_AP_association);
                            for(int n = 0 ; n < store_each_UE_possible_AP[6].size() ; n++){
                                possible_AP_association[6] = store_each_UE_possible_AP[6][n];
                                all_possible_next_AP_association.push_back(possible_AP_association);
                                for(int h = 0 ; h < store_each_UE_possible_AP[7].size() ; h++){
                                    possible_AP_association[7] = store_each_UE_possible_AP[7][h];
                                    all_possible_next_AP_association.push_back(possible_AP_association);
                                    for(int g = 0 ; g < store_each_UE_possible_AP[8].size() ; g++){
                                        possible_AP_association[8] = store_each_UE_possible_AP[8][g];
                                        all_possible_next_AP_association.push_back(possible_AP_association);
                                        for(int f = 0 ; f < store_each_UE_possible_AP[9].size() ; f++){
                                            possible_AP_association[9] = store_each_UE_possible_AP[9][f];
                                            all_possible_next_AP_association.push_back(possible_AP_association);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    std::sort(all_possible_next_AP_association.begin(), all_possible_next_AP_association.end());
    auto last = std::unique(all_possible_next_AP_association.begin(), all_possible_next_AP_association.end());
    all_possible_next_AP_association.erase(last, all_possible_next_AP_association.end());

    //std::cout << "all_possible_next_AP_association size : "<< all_possible_next_AP_association.size() << "\n";
    return all_possible_next_AP_association;
}

std::vector<double> createUEDemandVector(std::vector<MyUeNode> &my_UE_list){
    std::vector<double> demands;
    for(int i = 0 ; i < my_UE_list.size() ; i++){
        demands.push_back(my_UE_list[i].getRequiredDataRate());
    }
    return demands;
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

std::vector<std::vector<int>> AP_association_vector_to_matrix(std::vector<int> &AP_association_vector){
    std::vector<std::vector<int>> AP_association_matrix = std::vector<std::vector<int>>(RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0));
    for(int i = 0 ; i < UE_num ; i++){
        switch(AP_association_vector[i]){
            case 0:
                AP_association_matrix[0][i] += 1;
                break;
            case 1:
                AP_association_matrix[1][i] += 1;
                break;
            case 2:
                AP_association_matrix[2][i] += 1;
                break;
            case 3:
                AP_association_matrix[3][i] += 1;
                break;
            case 4:
                AP_association_matrix[4][i] += 1;
                break;
            case 5:
                AP_association_matrix[0][i] += 1;
                AP_association_matrix[1][i] += 1;
                break;
            case 6:
                AP_association_matrix[0][i] += 1;
                AP_association_matrix[2][i] += 1;
                break;
            case 7:
                AP_association_matrix[0][i] += 1;
                AP_association_matrix[3][i] += 1;
                break;
            case 8:
                AP_association_matrix[0][i] += 1;
                AP_association_matrix[4][i] += 1;
                break;
            default:
                break;
        }
    }
    return AP_association_matrix;
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

std::vector<std::vector<double>> SINR_matrix_to_two_high_SINR(std::vector<std::vector<double>> &SINR_matrix){
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
    /*  new
    std::fstream output;
    output.open("/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/UE_SINR.csv",std::ios::out | std::ios::app );
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        for(int i = 0 ; i<UE_num;i++){
            output << State_SINR[i][0] << "," << State_SINR[i][1] << "," << State_SINR[i][2];
            output << std::endl;
        }
    }
    output.close();*/
    return State_SINR;
}

std::vector<std::vector<int>> SINR_matrix_to_two_high_AP_index(std::vector<std::vector<double>> &SINR_matrix){
    std::vector<std::vector<int>> State_SINR_VLC_index = std::vector<std::vector<int>> (UE_num,std::vector<int>(2,0));
    for(int i = 0 ; i < UE_num ; i++){
        double highest_AP_value = -DBL_MAX ;
        double second_AP_value = -DBL_MAX ;
        int highest_AP_index = -INT_MAX ;
        int second_AP_index = -INT_MAX;
        for(int j = 1 ; j < VLC_AP_num + 1 ; j++){
            if(SINR_matrix[j][i] > highest_AP_value){
                second_AP_value = highest_AP_value;
                second_AP_index = highest_AP_index;
                highest_AP_value = SINR_matrix[j][i];
                highest_AP_index = j;
            }
        }
        //std::cout << "high :" << highest_AP_value << "\t second : " << second_AP_value <<"\n";
        State_SINR_VLC_index[i][0] = (highest_AP_value == -DBL_MAX)? -5 : highest_AP_index; // if State_SINR_VLC_index[i][0] = -5 means that there is no highest SINR VLC AP
        State_SINR_VLC_index[i][1] = (second_AP_value == 0 || second_AP_value == -DBL_MAX)? -5 : second_AP_index; // if State_SINR_VLC_index[i][1] = -5 means that there is no second SINR VLC AP
    }
    return State_SINR_VLC_index;
}

double calculatedR1(std::vector<int> &UE_type,
                 std::vector<int> &pre_AP_association,
                 std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,
                 std::vector<double> &RF_data_rate_vector,
                 std::vector<int> &State_AP_load){
    double total_throughput = 0.0;
    for(int i = 0 ;i < UE_num ; i++){

        double throughput = 0.0;
        if(UE_type[i] == 1){ //SAP
            if(AP_association[i] != -1){
                double data_rate = 0.0;
                if(AP_association[i] < 5)
                    data_rate = (AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[AP_association[i]-1][i];
                else
                    std::cout<<"**(benchmark.cc) SAP UE AP association is wrong**\n";

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
                if(AP_association[i] > 4)
                    throughput = eta * ((RF_data_rate_vector[i] * (1.0 / State_AP_load[0])) + (VLC_data_rate_matrix[AP_association[i]-5][i] * (1.0 / State_AP_load[AP_association[i]-4])));
                else
                    throughput = (AP_association[i] == 0)? eta * RF_data_rate_vector[i] * (1.0 / State_AP_load[0]) : eta * VLC_data_rate_matrix[AP_association[i]-1][i] * (1.0 / State_AP_load[AP_association[i]]);
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
                 std::vector<int> &State_AP_load,
                 std::vector<double> &UEdemands){
    double total_us = 0.0;
    for(int i = 0 ;i < UE_num ; i++){
        double us = 0.0;
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
                if(AP_association[i] < 5 )
                    data_rate = (AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[AP_association[i]-1][i];
                else
                    std::cout<<"**(benchmark.cc) SAP UE AP association is wrong**\n";
                double throughput = eta * data_rate * (1.0 / State_AP_load[AP_association[i]]);
                us = throughput / UEdemands[i];
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
                double throughput = 0.0;
                if(AP_association[i] > 4)
                    throughput = eta * ((RF_data_rate_vector[i] * (1.0 / State_AP_load[0])) + (VLC_data_rate_matrix[AP_association[i]-5][i] * (1.0 / State_AP_load[AP_association[i]-4])));
                else
                    throughput = (AP_association[i] == 0)? eta * RF_data_rate_vector[i] * (1.0 / State_AP_load[0]) : eta * VLC_data_rate_matrix[AP_association[i]-1][i] * (1.0 / State_AP_load[AP_association[i]]);
                us = throughput / UEdemands[i];
            }
        }
        total_us += us * C_one;
    }
    return total_us / UE_num;
}

double calculatedR3(std::vector<int> &UE_type,
                 std::vector<int> &pre_AP_association,
                 std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,
                 std::vector<double> &RF_data_rate_vector,
                 std::vector<int> &State_AP_load,
                 std::vector<double> &UEdemands){
    double total_q = 0.0;
    for(int i = 0 ;i < UE_num ; i++){
        double q = 0.0;
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
                    data_rate = (AP_association[i] == 0)? RF_data_rate_vector[i] : VLC_data_rate_matrix[AP_association[i]-1][i];
                else
                    std::cout<<"**(benchmark.cc) SAP UE AP association is wrong**\n";
                double throughput = eta * data_rate * (1.0 / State_AP_load[AP_association[i]]);
                double us = throughput / UEdemands[i];
                q = (us <= 0.5)? (-C_two * (1 - us)) : C_one * us;
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
                double throughput = 0.0;
                if(AP_association[i] > 4)
                    throughput = eta * ((RF_data_rate_vector[i] * (1.0 / State_AP_load[0])) + (VLC_data_rate_matrix[AP_association[i]-5][i] * (1.0 / State_AP_load[AP_association[i]-4])));
                else
                    throughput = eta * (AP_association[i] == 0)? RF_data_rate_vector[i] * (1.0 / State_AP_load[0]) : VLC_data_rate_matrix[AP_association[i]-1][i] * (1.0 / State_AP_load[AP_association[i]]);
                double us = throughput / UEdemands[i];
                q = (us <= 0.5)? (-C_two * (1 - us)) : C_one * us;
            }
        }
        total_q += q;
    }
    return total_q / UE_num;
}

void updateApAssociationResult(std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list){

    // weird, useless?
    for(int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            if(AP_index == 0){
                my_UE_list[UE_index].setCurrRFAssociatedAP(AP_sssociation_matrix[0][UE_index]);
            }
            else{
                if(AP_sssociation_matrix[AP_index][UE_index] == 1){
                    my_UE_list[UE_index].setCurrVlcAssociatedAP(AP_index);
                }
            }
        }
    }
}
