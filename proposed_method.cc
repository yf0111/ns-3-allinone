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

    // get UE require data rate
    std::vector<double> UE_require_data_rate = createUEDemandVector(my_UE_list);



    // set UE final date rate
    std::vector<double> UE_final_data_rate = std::vector<double> (UE_num , 0.0);
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        double data_rate = 0.0;
        for(int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
            reachable_data_rate[AP_index][UE_index] = calDataRate(RF_SINR_vector,VLC_SINR_matrix,AP_allocate_time,AP_index,UE_index);
            data_rate += reachable_data_rate[AP_index][UE_index];
        }
        UE_final_data_rate[UE_index] = data_rate;
    }

    // init AP selection (first state is INT_MAX)
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        my_UE_list[UE_index].changeCurrAssociatedAP(INT_MAX);
    }

    // calculate UE satisfaction (US)
    std::vector<double> US_reliability = std::vector<double> (UE_num , 0.0);
    std::vector<double> US_latency = std::vector<double> (UE_num , 0.0);
    std::vector<double> US_datarate = std::vector<double> (UE_num , 0.0);
    cal_US_Reliability(RF_SINR_vector,VLC_SINR_matrix,US_reliability);
    cal_US_Latency(US_latency);
    cal_US_DataRate(UE_final_data_rate,UE_require_data_rate,US_datarate);

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        std::cout << cal_US_Reliability[UE_index] << "\t" << cal_US_Latency[UE_index] << "\t"<< cal_US_DataRate[UE_index] << "\n";
    }

    /*
    // change AP selection
    for( int UE_index = 0; UE_index < UE_num ; UE_index++){
        for(int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
            int AP_associated_index = INT_MAX;
            double AP_associated_max_value = -DBL_MAX;
            if(my_UE_list[UE_index].getGroup() == 1){ // uRLLC UE

                // (看哪個AP可以給他最高的satisfaction)

            }
            if(my_UE_list[UE_index].getGroup() == 2){ // normal UE
                // (看哪個AP可以給他最高的require rate)
                if(reachable_data_rate[AP_index][UE_index] > AP_associated_max_value){
                    AP_associated_index = AP_index;
                    AP_associated_max_value = reachable_data_rate[AP_index][UE_index];
                }
            }
            if(AP_associated_index != INT_MAX){
                my_UE_list[UE_index].setCurrAssociatedAP(AP_associated_index);
            }
        }
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
        std::cout << final_data_rate[UE_index] << "\t" << require_data_rate[UE_index] << "\t" <<  std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) << "\n";
        US_datarate[UE_index] = std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) ;
    }
}

double SINR_to_dB(double SINR){
    return (SINR == 0.0)? 0.0 : 10 * log10(SINR);
}

double dB_to_SINR(double dB){
    return std::pow(10.0,dB/10.0);
}
