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

    // trying method 1 : divide equally (under a specific Channel Condition)
    // channel condition : SINR > SINR_threshold
    std::vector<int> eligible_UE_quantity = std::vector<int> (RF_AP_num + VLC_AP_num , 0);
    for( int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
        int number = 0;
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            if( AP_index < RF_AP_num ){ //RF
                if(SINR_to_dB(RF_SINR_vector[UE_index]) > SINR_threshold){
                    number++;
                }
            }
            else{ //VLC
                if(SINR_to_dB(VLC_SINR_matrix[AP_index-1][UE_index]) > SINR_threshold){
                    number++;
                }
            }
        }
        eligible_UE_quantity[AP_index] = number;
    }

    std::vector<std::vector<double>> AP_allocate_time = std::vector<std::vector<double>> (RF_AP_num + VLC_AP_num , std::vector<double>(UE_num , 0)); // save allocate time for each UE (%)
    std::vector<std::vector<double>> reachable_data_rate = std::vector<std::vector<double>>(RF_AP_num + VLC_AP_num , std::vector<double>(UE_num , 0.0));

    for( int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            if( AP_index < RF_AP_num ){ //RF
                if(SINR_to_dB(RF_SINR_vector[UE_index]) > SINR_threshold)
                    AP_allocate_time[AP_index][UE_index] = 1.0 / eligible_UE_quantity[AP_index];
            }
            else{ //VLC
                if(SINR_to_dB(VLC_SINR_matrix[AP_index-1][UE_index]) > SINR_threshold)
                    AP_allocate_time[AP_index][UE_index] = 1.0 / eligible_UE_quantity[AP_index];
            }
        }
    }

    for(int i = 0 ; i < RF_AP_num + VLC_AP_num ; i++){
        for(int j = 0 ; j < UE_num ; j++){
            reachable_data_rate[i][j] = calDataRate(RF_SINR_vector,VLC_SINR_matrix,AP_allocate_time,i,j);
            std::cout << reachable_data_rate[i][j] << "\t";
        }
        std::cout <<"\n";
    }


    /*std::vector<double> US_reliability = std::vector<double>(UE_num,0.0);
    calReliability(RF_SINR_vector,VLC_SINR_matrix,US_reliability);

    for(int i = 0 ; i < UE_num ; i++){
        std::cout << US_reliability[i] << "\n";
    }*/

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

void calReliability(std::vector<double> &RF_SINR_vector,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<double> &US_reliability)
{
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

double SINR_to_dB(double SINR){
    return (SINR == 0.0)? 0.0 : 10 * log10(SINR);
}

double dB_to_SINR(double dB){
    return std::pow(10.0,dB/10.0);
}
