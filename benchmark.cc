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
#include "action_type.h"
#include "env_state_type.h"

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
                     std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
                     std::vector<std::vector<double>> &VLC_data_rate_matrix,
                     std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix_3d,
                     std::vector<double> &RF_channel_gain_vector,
                     std::vector<double> &RF_SINR_vector,
                     std::vector<double> &RF_data_rate_vector,
                     std::vector<std::vector<int>> &AP_association_matrix,
                     std::vector<MyUeNode> &my_UE_list,
                     std::vector<double> &UE_final_data_rate_vector,
                     std::vector<Env_state_type> &env_state_vec,
                     std::vector<Action_type> &action_vec,
                     std::vector<double> &value_func_vec,
                     std::map<Env_state_type,Action_type> &policy_map,
                     std::vector<double> &dqn_vec)
{
    /*
        calculate VLC LOS and VLC SINR and RF LOS and RF SINR
    */
    precalculation(RF_AP_node, VLC_AP_nodes, UE_nodes, VLC_LOS_matrix, VLC_SINR_matrix,VLC_SINR_matrix_3d, VLC_data_rate_matrix,VLC_data_rate_matrix_3d, RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector, my_UE_list);
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

#if PDSERT
    /*
        ref'1 , PDSERT
    */
    PDS_ERT(AP_association_matrix,RF_SINR_vector,VLC_SINR_matrix,UE_final_data_rate_vector,my_UE_list,VLC_SINR_matrix_3d,env_state_vec,action_vec,value_func_vec,policy_map,dqn_vec);
#endif // PDSERT

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

void PDS_ERT(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
             std::vector<Env_state_type> &env_state_vec,
             std::vector<Action_type> &action_vec,
             std::vector<double> &value_func_vec,
             std::map<Env_state_type,Action_type> &policy_map,
             std::vector<double> &dqn_vec)
{

    /*
        URLLC requirement + Minimum data rate Requirements
    */


    /*
        initialize Step
    */
    initializedStep(env_state_vec,value_func_vec,policy_map,dqn_vec,VLC_SINR_matrix_3d,RF_SINR_vector);

}

void initializedStep(std::vector<Env_state_type> &env_state_vec,
                     std::vector<double> &value_func_vec,
                     std::map<Env_state_type,Action_type> &policy_map,
                     std::vector<double> &dqn_vec,
                     std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d, // 36x10x16
                     std::vector<double> &RF_SINR_vector)
{
    /*
       initialize Step
            1. network state s0
            2. value function V(s0)
            3. Policy strategy π(s0)
            4. DQN , parameter θ0
    */
    Env_state_type new_env_state;
    std::vector<std::vector<double>> RF_SINR_vector_2d = extend_vector_1to2d(RF_SINR_vector,RF_AP_subchannel);
    new_env_state.setEnvStateRFSINR(RF_SINR_vector_2d);
    new_env_state.setEnvStateVLCSINR(VLC_SINR_matrix_3d);

    /*   set UE tyepe   */
    for(int i = 0 ; i < UE_num ; i++){
        if(i < UE_num / 2){
            new_env_state.setEnvStateUEtype(i,1);
        }
        else{
            new_env_state.setEnvStateUEtype(i,2);
        }
    }

    /*   set init sub channel association  */
    for(int j = 0 ; j < UE_num ; j++){
        new_env_state.setEnvStateRFSubChannel(0,j,1);
    }
    for(int i = 0 ; i < VLC_AP_num; i++){
        for(int j = 0 ; j < VLC_AP_subchannel ; j++){
            int max_UE_index = -1;
            double max_value = -DBL_MAX;
            for(int k = 0; k < UE_num ; k++){
                if(new_env_state.getEnvStateVLCSINR(i,j,k) > max_value){
                    max_UE_index = k;
                    max_value = new_env_state.getEnvStateVLCSINR(i,j,k);
                }
            }
            if(max_UE_index != -1){
                new_env_state.setEnvStateVLCSubChannel(i,j,max_UE_index,1);
            }
            else{
                std::cout<<"set init sub channel association error!\n";
            }
        }
    }

    /*
        !*-*-TODO*-*-! 20230216 : check data rate !
    */
    std::vector<double> init_data_rate = std::vector<double> (UE_num,0.0);
    calculateDataRate(new_env_state , init_data_rate);
    for(int i = 0 ; i < UE_num ; i++){
        std::cout<<" UE : "<<i<< " data rate : "<<init_data_rate[i]<<"\n";
    }



}

void calculateDataRate(Env_state_type &now_env_state , std::vector<double> &init_data_rate){
    for(int ue_index = 0 ; ue_index < UE_num ; ue_index ++){
        double UE_data_rate = 0.0;
        /*   VLC   */
        for(int i = 0 ; i < VLC_AP_num ; i++){
            for(int j = 0; j < VLC_AP_subchannel ; j++){
                if(now_env_state.getEnvStateVLCSubChannel(i,j,ue_index)){
                    UE_data_rate += (((double)VLC_AP_bandwidth / VLC_AP_subchannel) / 2.0 ) * log(1 + now_env_state.getEnvStateVLCSINR(i,j,ue_index));
                }
            }
        }
        /*   RF   */
        for(int j = 0 ; j < RF_AP_subchannel ; j++){
            if(now_env_state.getEnvStateRFSubChannel(j,ue_index)){
                UE_data_rate += ((double) RF_AP_bandwidth / RF_AP_subchannel) * log(1 + now_env_state.getEnvStateRFSINR(j,ue_index)); // downlink
                UE_data_rate += ((double) RF_AP_bandwidth / RF_AP_subchannel) * log(1 + now_env_state.getEnvStateRFSINR(j,ue_index)); // uplink
            }
        }
        init_data_rate[ue_index] = UE_data_rate;
    }
}


std::vector<std::vector<double>> extend_vector_1to2d(std::vector<double> &extend_vector,int y_size){
    std::vector<std::vector<double>> ans_vector = std::vector<std::vector<double>>(y_size,std::vector<double>(extend_vector.size(),0.0));
    for(int j = 0 ; j < extend_vector.size() ; j++){
        ans_vector[0][j] = extend_vector[j];
    }
    return ans_vector;
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
