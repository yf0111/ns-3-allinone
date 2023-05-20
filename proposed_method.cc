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


void proposedLB(int &state,
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

    // trying method 1 : [Wi-Fi : urllc device] [Li-Fi : normal device]
    //      1. Wifi proportional distribution according to require data rate
    //      2. Lifi proportional distribution according to require data rate
    //      3. If the normal device cannot achieve the required data rate through LiFi, use WiFi instead and allocate the excess WiFi power intended for URLLC devices to the normal device.
    // channel condition : SINR > SINR_threshold (5dB)

    // get UE require data rate
    UE_require_data_rate = createUEDemandVector(my_UE_list);

    // step 1 (APS), urllc -> WiFi , normal -> based on LiFi SINR (SSS)
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
    std::vector<int> AP_serve_number = AP_association_matrix_to_UE_numbers(local_AP_association_matrix);
    std::string path = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output;
    output.open(path + "AP_load_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        output << AP_serve_number[0] << "," << AP_serve_number[1] << "," << AP_serve_number[2] << "," << AP_serve_number[3] << "," << AP_serve_number[4];
        output << std::endl;
    }
    output.close();


    // step 2 (RA), wifi : proportional distribution according to the required data rate , lifi : proportional distribution according to the required data rate
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
            if(i < RF_AP_num){ // RF , based on require data rate
                /*if(local_AP_association_matrix[i][j] == 1){ // equally
                    AP_allocate_power[i][j] = 1.0 / serve_ue_number;
                }*/
                if(local_AP_association_matrix[i][j] == 1){
                    AP_allocate_power[i][j] = UE_require_data_rate[j] / total_data_rate;
                }
            }
            else{ // VLC , based on require data rate
                if(local_AP_association_matrix[i][j] == 1){
                    AP_allocate_power[i][j] = UE_require_data_rate[j] / total_data_rate;
                }
            }
        }
    }


    // step 3 : calculate final UE data rate
    updateApAssociationResult(local_AP_association_matrix,AP_association_matrix,my_UE_list);
    updateAllRFSINR(RF_SINR_vector,RF_channel_gain_vector,AP_allocate_power);
    updateAllVlcSINR(VLC_LOS_matrix,VLC_SINR_matrix,local_AP_association_matrix,AP_allocate_power);

    calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);
    calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);

    UE_final_data_rate_vector = std::vector<double> (UE_num,0.0) ;
    for(int i = 0 ; i < UE_num ; i++){
        UE_final_data_rate_vector[i] += roundNumber(RF_data_rate_vector[i], 3);
        for(int j = 0 ; j < VLC_AP_num ; j++){
            UE_final_data_rate_vector[i] += roundNumber(VLC_data_rate_matrix[j][i],3);
        }
    }

    // step 3.5 : Store normal device which not getting enough data rate and urllc device witch getting too much data rate
    std::vector<double> minimum_rf_allocate_percentage = std::vector<double> (UE_num,0.0);
    cal_minumum_allocate_power_percentage(minimum_rf_allocate_percentage,UE_require_data_rate,RF_channel_gain_vector);
    std::vector<std::pair<int, double>> insufficient_normal_ue; // < ue index , require data rate - final data rate >
    std::vector<std::pair<int,double>> excess_urllc_ue; // < ue index , final data rate - require data rate >
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //std::cout << "UE : " << UE_index << " ,require : " << UE_require_data_rate[UE_index] << " ,get : " << UE_final_data_rate_vector[UE_index] << "\n";
        if(my_UE_list[UE_index].getGroup() == 2 &&  UE_final_data_rate_vector[UE_index] < UE_require_data_rate[UE_index]){
            insufficient_normal_ue.push_back(std::make_pair(UE_index , UE_require_data_rate[UE_index] - UE_final_data_rate_vector[UE_index]));
        }
        if(my_UE_list[UE_index].getGroup() == 1 && UE_final_data_rate_vector[UE_index] > UE_require_data_rate[UE_index]){
            excess_urllc_ue.push_back(std::make_pair(UE_index , UE_final_data_rate_vector[UE_index] - UE_require_data_rate[UE_index]));
        }
    }
    sort(insufficient_normal_ue.begin(), insufficient_normal_ue.end(), sort_by_sec_descending); // descending order , big -> small
    sort(excess_urllc_ue.begin(), excess_urllc_ue.end() , sort_by_sec_descending); // descending order , big -> small


    // step 4 : Reallocate RF AP power
    while(!insufficient_normal_ue.empty() && !excess_urllc_ue.empty()){ // while(insufficient_normal_ue is not empty and has resource to allocate)
        // start with insufficient_normal_ue[insufficient_normal_ue.size()-1].first , (need minimum index)
        // start with excess_urllc_ue[excess_urllc_ue.size()-1].first , (excess minimum index)
        //std::cout << "insufficient ue : " << insufficient_normal_ue[insufficient_normal_ue.size()-1].first << " , insufficient data rate :" << insufficient_normal_ue[insufficient_normal_ue.size()-1].second << "\n";
        //std::cout << "excess ue : " << excess_urllc_ue[excess_urllc_ue.size()-1].first << " , excess data rate :" << excess_urllc_ue[excess_urllc_ue.size()-1].second << "\n";
        AP_allocate_power[0][insufficient_normal_ue[insufficient_normal_ue.size()-1].first] += AP_allocate_power[0][excess_urllc_ue[excess_urllc_ue.size()-1].first] - minimum_rf_allocate_percentage[excess_urllc_ue[excess_urllc_ue.size()-1].first];
        AP_allocate_power[0][excess_urllc_ue[excess_urllc_ue.size()-1].first] = minimum_rf_allocate_percentage[excess_urllc_ue[excess_urllc_ue.size()-1].first];

        updateAllRFSINR(RF_SINR_vector,RF_channel_gain_vector,AP_allocate_power);
        updateAllVlcSINR(VLC_LOS_matrix,VLC_SINR_matrix,local_AP_association_matrix,AP_allocate_power);
        calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);
        calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);

        UE_final_data_rate_vector = std::vector<double> (UE_num,0.0) ;
        for(int i = 0 ; i < UE_num ; i++){
            UE_final_data_rate_vector[i] += roundNumber(RF_data_rate_vector[i], 3);
            for(int j = 0 ; j < VLC_AP_num ; j++){
                UE_final_data_rate_vector[i] += roundNumber(VLC_data_rate_matrix[j][i],3);
            }
        }

        //std::cout << " -- insufficient ue : " << insufficient_normal_ue[insufficient_normal_ue.size()-1].first << " , now get : " << UE_final_data_rate_vector[insufficient_normal_ue[insufficient_normal_ue.size()-1].first] << "\n";

        if( UE_final_data_rate_vector[insufficient_normal_ue[insufficient_normal_ue.size()-1].first] >= UE_require_data_rate[insufficient_normal_ue[insufficient_normal_ue.size()-1].first]){
            insufficient_normal_ue.pop_back();
        }
        else{
            insufficient_normal_ue[insufficient_normal_ue.size()-1].second = UE_require_data_rate[insufficient_normal_ue[insufficient_normal_ue.size()-1].first] - UE_final_data_rate_vector[insufficient_normal_ue[insufficient_normal_ue.size()-1].first];
        }
        excess_urllc_ue.pop_back();
    }

    /*for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        std::cout<<std::setiosflags(std::ios::fixed)<<std::setprecision(3) << "*UE : " << UE_index << " ,require : " << UE_require_data_rate[UE_index] << " ,get : " << UE_final_data_rate_vector[UE_index] << "\n";
    }*/

    // step 5 : calculate user satisfaction
    std::vector<double> US_reliability = std::vector<double>(UE_num,0.0);
    std::vector<double> US_latency = std::vector<double> (UE_num,0.0);
    std::vector<double> US_datarate = std::vector<double> (UE_num,0.0);
    cal_US_Reliability(RF_SINR_vector,VLC_SINR_matrix,US_reliability);
    cal_US_Latency(US_latency,UE_final_data_rate_vector);
    cal_US_DataRate(UE_final_data_rate_vector,UE_require_data_rate,US_datarate);

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        // satisfaction先用大家都一樣去跑，但是老師覺得不同UE有不同線性組合是合理的，先用1:1:0.2跑跑看，之後再決定normal device要換多少

        if(my_UE_list[UE_index].getGroup() == 1){ // urllc
            UE_final_satisfaction_vector[UE_index] = (0.454545455 * US_reliability[UE_index]) + (0.454545455 * US_latency[UE_index]) + (0.09090909 * US_datarate[UE_index]); // 1:1:0.2
        }
        if(my_UE_list[UE_index].getGroup() == 2){ // normal
            UE_final_satisfaction_vector[UE_index] = (0.1 * US_reliability[UE_index]) + (0.1 * US_latency[UE_index]) + (0.8 * US_datarate[UE_index]); // 1:1:8
        }
        //UE_final_satisfaction_vector[UE_index] = (0.454545455 * US_reliability[UE_index]) + (0.454545455 * US_latency[UE_index]) + (0.09090909 * US_datarate[UE_index]); // 1:1:0.2

    }


}
void reConfigure_APS(std::vector<double> &RF_SINR_vector,
                     std::vector<std::vector<double>> &VLC_SINR_matrix,
                     std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<std::vector<int>> &AP_association_matrix){

    std::vector<std::vector<int>> local_AP_association_matrix = AP_association_matrix;

}

void cal_minumum_allocate_power_percentage(std::vector<double> &minimum_rf_allocate_percentage,
                                           std::vector<double> &UE_require_data_rate,
                                           std::vector<double> &RF_channel_gain_vector){
    for(int i = 0 ; i < UE_num ; i++){
        double allocate = ((RF_noise_power_spectral_density * RF_AP_bandwidth) / (RF_AP_power * RF_channel_gain_vector[i])) * ( pow(2,(UE_require_data_rate[i]) / RF_AP_bandwidth) - 1 );
        minimum_rf_allocate_percentage[i] = allocate;
    }
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

void cal_US_Latency(std::vector<double> &US_latency,
                    std::vector<double> &UE_final_data_rate_vector){
    /* TL = Tt + Ta + Tb + Tr + Tp , (TL < Tmax (1ms))? 1:0 */
    /* Tw = waiting time in queue -> can ignore */
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        double TL = 0.1 + 0.3 ; // Ta + Tb = 0.1ms , Tr + Tp = 0.3ms
        TL += packet_size / (1e6 * UE_final_data_rate_vector[UE_index]) * 1e3;
        if( TL < T_max){
            US_latency[UE_index] = 1;
        }
    }
}

void cal_US_DataRate(std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<double> &US_datarate){
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //std::cout << final_data_rate[UE_index] << "\t" << require_data_rate[UE_index] << "\t" <<  std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) << "\n";
        US_datarate[UE_index] = std::min(final_data_rate[UE_index] / require_data_rate[UE_index],1.0) ;
    }
}

double SINR_to_dB(double SINR){
    return (SINR == 0.0)? 0.0 : 10 * log10(SINR);
}

double dB_to_SINR(double dB){
    return std::pow(10.0,dB/10.0);
}

bool sort_by_sec_descending(const std::pair<int,double> &a, const std::pair<int,double> &b){
    return (a.second > b.second);
}

bool sort_by_sec_ascending(const std::pair<int,double> &a, const std::pair<int,double> &b){
    return (a.second < b.second);
}

double roundNumber(double oriNum, int bits = 1) {

	unsigned long times = 1;
	double baseNum = 0.5;

	for (int i = 1; i <= bits; ++i) {
		times *= 10;
		baseNum /= (times*1.0);
	}

	oriNum += baseNum;
	long zhengPart = (long)(oriNum*times);
	oriNum = zhengPart / (times*1.0);
	return oriNum;

}
