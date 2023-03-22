#include <map>
#include <limits>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <string>


#include "print.h"
#include "channel.h"
#include "my_UE_node.h"
#include "proposed_method.h"
#include "benchmark.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"


void proposedStaticLB(int &state,
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

    // trying method 1 : [Wi-Fi : urllc device] [Li-Fi : normal device]
    //      1. Wifi allocate the resources equally
    //      2. Lifi proportional distribution according to require data rate
    //      3. If the normal device cannot achieve the required data rate through LiFi, use WiFi instead and allocate the excess WiFi power intended for URLLC devices to the normal device.
    // channel condition : SINR > SINR_threshold (5dB)

    // get UE require data rate
    std::vector<double> UE_require_data_rate = createUEDemandVector(my_UE_list);

    // step 1 , urllc -> WiFi , normal -> APS based on LiFi SINR
    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(my_UE_list[UE_index].getGroup() == 1){
            local_AP_association_matrix[0][UE_index] = 1;
        }
        if(my_UE_list[UE_index].getGroup() == 2){
            int max_AP_index = 0;
            double max_AP_value = -DBL_MAX;
            for(int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
                if(VLC_SINR_matrix[VLC_AP_index][UE_index]!= 0 && VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_value){
                    max_AP_index = VLC_AP_index + 1;
                    max_AP_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
                }
            }
            local_AP_association_matrix[max_AP_index][UE_index] = 1;
        }
    }

    // step 2 , wifi : devide equally , lifi : proportional distribution according to the required data rate
    std::vector<std::vector<double>> AP_allocate_power = std::vector<std::vector<double>> (RF_AP_num + VLC_AP_num , std::vector<double> (UE_num,0.0)); // save allocate power for each UE (%)
    for(int i = 0 ; i < RF_AP_num + VLC_AP_num ; i++){
        int serve_ue_number = 0;
        double total_data_rate = 0;

        for(int j = 0 ; j < UE_num ; j++){
            if(local_AP_association_matrix[i][j] == 1){
                serve_ue_number += 1;
                total_data_rate += UE_require_data_rate[j];
            }
        }

        for(int j = 0 ; j < UE_num ; j++){
            if(i < RF_AP_num){ // RF , equally
                if(local_AP_association_matrix[i][j] == 1){
                    AP_allocate_power[i][j] = 1.0 / serve_ue_number;
                }
            }
            else{ // VLC , based on require data rate
                if(local_AP_association_matrix[i][j] == 1){
                    AP_allocate_power[i][j] = UE_require_data_rate[j] / total_data_rate;
                }
            }
        }
    }


    updateApAssociationResult(local_AP_association_matrix,AP_association_matrix,my_UE_list);
    updateAllRFSINR(RF_SINR_vector,RF_channel_gain_vector,AP_allocate_power);
    updateAllVlcSINR(VLC_LOS_matrix,VLC_SINR_matrix,local_AP_association_matrix,AP_allocate_power);

    calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);
    calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);

    // step 3 : calculate final UE data rate
    for(int i = 0 ; i < UE_num ; i++){
        UE_final_data_rate_vector[i] += RF_data_rate_vector[i];
        for(int j = 0 ; j < VLC_AP_num ; j++){
            UE_final_data_rate_vector[i] += VLC_data_rate_matrix[j][i];
        }
    }

    printUEFinalDataRate(UE_final_data_rate_vector);

    /*-------0323 : start from step 3---------*/
}


double calDataRate(std::vector<double> &RF_SINR_vector,
                   std::vector<std::vector<double>> &VLC_SINR_matrix,
                   std::vector<std::vector<double>> &AP_allocate_time,
                   int AP_index,
                   int UE_index)
{
    double data_rate = 0.0;
    data_rate = (AP_index < RF_AP_num)? RF_AP_bandwidth / 2.0 * log2(1 + RF_SINR_vector[UE_index])* AP_allocate_time[AP_index][UE_index] : VLC_AP_bandwidth / 2.0 * log2(1 + EE / 2.0 * PI * VLC_SINR_matrix[AP_index-1][UE_index]) * AP_allocate_time[AP_index][UE_index];
    return std::isnan(data_rate)? 0.0 : data_rate;
}

void cal_US_Reliability(std::vector<double> &RF_SINR_vector,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<double> &US_reliability){
    /* check receive SINR > threshold SINR or not , 1/0 */
    for( int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        for(int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
            if( AP_index < RF_AP_num ){ //RF
                if(SINR_to_dB(RF_SINR_vector[UE_index]) > SINR_threshold){
                    US_reliability[UE_index] = 1 ;
                    break;
                }
            }
            else{ //VLC
                if(SINR_to_dB(VLC_SINR_matrix[AP_index-1][UE_index]) < SINR_threshold){
                    US_reliability[UE_index] = 1;
                    break;
                }
            }
        }
    }
}

void cal_US_Latency(std::vector<double> &US_latency){
    /* Is handover required? no handover : 1 ; handover time < threshold handover time 1/0 */
    for(int i = 0 ; i < UE_num ; i++){
        US_latency[i] = 1;
    }
}

void cal_US_DataRate(std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<double> &US_datarate){
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //std::cout << final_data_rate[UE_index] << "\t" << require_data_rate[UE_index] << "\t" <<  std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) << "\n";
        US_datarate[UE_index] = std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) ;
    }
}

double SINR_to_dB(double SINR){
    return (SINR == 0.0)? 0.0 : 10 * log10(SINR);
}

double dB_to_SINR(double dB){
    return std::pow(10.0,dB/10.0);
}
