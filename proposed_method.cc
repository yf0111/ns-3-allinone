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
                std::vector<double> &UE_require_data_rate,
                double ue_satisfation,
                std::vector<std::vector<double>> &AP_allocate_power,
                double active_ue_satisfaction)
{
    precalculation(RF_AP_node,VLC_AP_nodes, UE_nodes,
                   VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                   RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                   my_UE_list);

    /* initialize */
    UE_require_data_rate = createUEDemandVector(my_UE_list);

    if(ue_satisfation - active_ue_satisfaction < 0.035){
        APS(VLC_SINR_matrix,AP_association_matrix,my_UE_list,UE_nodes);
        RA(AP_allocate_power,AP_association_matrix,UE_require_data_rate);
        cal_performance(AP_association_matrix,my_UE_list,AP_allocate_power,VLC_LOS_matrix,VLC_SINR_matrix,VLC_data_rate_matrix,RF_channel_gain_vector,RF_SINR_vector,RF_data_rate_vector,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate);
        RAPS(VLC_LOS_matrix,UE_final_data_rate_vector,UE_require_data_rate,AP_association_matrix,AP_allocate_power);
        RA(AP_allocate_power,AP_association_matrix,UE_require_data_rate);
        updateApAssociationResult(AP_association_matrix,my_UE_list);
        cal_performance(AP_association_matrix,my_UE_list,AP_allocate_power,VLC_LOS_matrix,VLC_SINR_matrix,VLC_data_rate_matrix,RF_channel_gain_vector,RF_SINR_vector,RF_data_rate_vector,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate);
    }
    else{
        cal_performance(AP_association_matrix,my_UE_list,AP_allocate_power,VLC_LOS_matrix,VLC_SINR_matrix,VLC_data_rate_matrix,RF_channel_gain_vector,RF_SINR_vector,RF_data_rate_vector,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate);
    }


#if DEBUG_MODE // AP load

    std::vector<int> AP_serve_number = AP_association_matrix_to_UE_numbers(AP_association_matrix);
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

#endif // DEBUG_MODE
}


void RA(std::vector<std::vector<double>> &AP_allocate_power,
        std::vector<std::vector<int>> &local_AP_association_matrix,
        std::vector<double> &UE_require_data_rate){
    /*
    wifi : proportional distribution according to the required data rate
    lifi : proportional distribution according to the required data rate
    */
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
}

void APS(std::vector<std::vector<double>> &VLC_SINR_matrix,
         std::vector<std::vector<int>> &AP_association_matrix,
         std::vector<MyUeNode> &my_UE_list,
         NodeContainer &UE_nodes){

    /* method 1 : [Wi-Fi : urllc device] [Li-Fi : normal device] */
    /*for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(my_UE_list[UE_index].getGroup() == 1){
            AP_association_matrix[0][UE_index] = 1;
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
            AP_association_matrix[max_AP_index][UE_index] = 1;
        }
    }*/

    /* method 2 : [Wi-Fi : high speed normal device] [Li-Fi : other normal device and urllc device] */
    AP_association_matrix = std::vector<std::vector<int>> (RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0)); // TODO : 如果把要 reconfigure 的條件用好，這段要改
    int i = 0;
    for (NodeContainer::Iterator it = UE_nodes.Begin(); it != UE_nodes.End(); ++it) {
        Ptr<MobilityModel> UE_mobility_model = (*it)->GetObject<MobilityModel>();
        Vector speed = UE_mobility_model->GetVelocity();
        double new_velocity = sqrt((speed.x * speed.x) + (speed.y * speed.y));
        my_UE_list[i++].setVelocity(new_velocity);
    }

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(my_UE_list[UE_index].getGroup() == 1){
            int max_AP_index = 0;
            double max_AP_value = -DBL_MAX;
            for(int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
                if(VLC_SINR_matrix[VLC_AP_index][UE_index]!= 0 && VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_value){
                    max_AP_index = VLC_AP_index + 1;
                    max_AP_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
                }
            }
            AP_association_matrix[max_AP_index][UE_index] = 1;
        }
        if(my_UE_list[UE_index].getGroup() == 2){
            double speed = my_UE_list[UE_index].getVelocity();
            if(speed > speed_threshold){
                AP_association_matrix[0][UE_index] = 1;
            }
            else{
                int max_AP_index = 0;
                double max_AP_value = -DBL_MAX;
                for(int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
                    if(VLC_SINR_matrix[VLC_AP_index][UE_index]!= 0 && VLC_SINR_matrix[VLC_AP_index][UE_index] > max_AP_value){
                        max_AP_index = VLC_AP_index + 1;
                        max_AP_value = VLC_SINR_matrix[VLC_AP_index][UE_index];
                    }
                }
                AP_association_matrix[max_AP_index][UE_index] = 1;
            }
        }
    }

#if DEBUG_MODE
    printApAssociationMatrix(AP_association_matrix);
#endif // DEBUG_MODE

}

void RAPS(std::vector<std::vector<double>> &VLC_LOS_matrix,
          std::vector<double> &final_data_rate,
          std::vector<double> &require_data_rate,
          std::vector<std::vector<int>> &AP_association_matrix,
          std::vector<std::vector<double>> AP_allocate_power){

    std::vector<int> AP_serve_UE_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);
    // Link Aggregation
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(final_data_rate[UE_index] < require_data_rate[UE_index]){
            //std::cout << "**(WiFi) UE : " << UE_index << " ,require : " << require_data_rate[UE_index] << " ,get : " << final_data_rate[UE_index] << "\n";
            AP_association_matrix[0][UE_index] = 1;
        }
    }

    // WiFi
    while(AP_serve_UE_num[0] > wifi_threshold){
        std::cout <<" ??? 2ji2323ij4j23i4j2i3423 ???  \n" ;
        double min_value = DBL_MAX;
        int min_index = -1;
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            if(AP_association_matrix[0][UE_index] == 1){
                if(final_data_rate[UE_index] / AP_allocate_power[0][UE_index] < min_value){
                    min_index = UE_index;
                    min_value = final_data_rate[UE_index] / AP_allocate_power[0][UE_index];
                }
            }
        }
        AP_association_matrix[0][min_index] = 0;
        AP_serve_UE_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);
    }

    // LiFi** TODO need change
    std::vector<std::vector<double>> VLC_IINR = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double> (UE_num, 0.0));
    std::vector<std::vector<double>> VLC_SINR = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double> (UE_num, 0.0));
    cal_All_VLC_IINR_SINR(VLC_IINR,VLC_SINR,VLC_LOS_matrix,AP_allocate_power,AP_association_matrix);
    std::vector<int> curr_VLC_AP_index = std::vector<int> (UE_num,-1);
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        for(int AP_index = 1 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index++){
            if(AP_association_matrix[AP_index][UE_index] == 1){
                curr_VLC_AP_index[UE_index] = AP_index - 1;
            }
        }
    }
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            if( VLC_SINR[VLC_AP_index][UE_index] < VLC_IINR[VLC_AP_index][UE_index] ){
                //std::cout << "change! "<< UE_index << " ,curr:" << curr_VLC_AP_index[UE_index] << " ,change to:" << VLC_AP_index <<"\n";
                AP_association_matrix[curr_VLC_AP_index[UE_index]+1][UE_index] = 0;
                AP_association_matrix[VLC_AP_index+1][UE_index] = 1;
            }
        }
    }
}

void RRA(std::vector<double> &minimum_rf_allocate_percentage,
                    std::vector<double> &UE_require_data_rate,
                    std::vector<double> &UE_final_data_rate_vector,
                    std::vector<std::vector<double>> &VLC_LOS_matrix,
                    std::vector<std::vector<double>> &VLC_SINR_matrix,
                    std::vector<std::vector<double>> &VLC_data_rate_matrix,
                    std::vector<double> &RF_channel_gain_vector,
                    std::vector<double> &RF_SINR_vector,
                    std::vector<double> &RF_data_rate_vector,
                    std::vector<MyUeNode> &my_UE_list,
                    std::vector<std::vector<int>> &local_AP_association_matrix,
                    std::vector<std::vector<double>> &AP_allocate_power){

    // adjustment (Store normal device which not getting enough data rate and urllc device witch getting too much data rate)
    cal_minumum_allocate_power_percentage(minimum_rf_allocate_percentage,UE_require_data_rate,RF_channel_gain_vector);
    std::vector<std::pair<int, double>> insufficient_normal_ue; // < ue index , require data rate - final data rate >
    std::vector<std::pair<int,double>> excess_urllc_ue; // < ue index , final data rate - require data rate >
    std::vector<int> AP_serve_num = AP_association_matrix_to_UE_numbers(local_AP_association_matrix);
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

    while(!insufficient_normal_ue.empty() && !excess_urllc_ue.empty()){ // while(insufficient_normal_ue is not empty and has resource to allocate)
        // start with insufficient_normal_ue[insufficient_normal_ue.size()-1].first , (need minimum index)
        // start with excess_urllc_ue[excess_urllc_ue.size()-1].first , (excess minimum index)
        std::cout << "insufficient ue : " << insufficient_normal_ue[insufficient_normal_ue.size()-1].first << " , insufficient data rate :" << insufficient_normal_ue[insufficient_normal_ue.size()-1].second << "\n";
        std::cout << "excess ue : " << excess_urllc_ue[excess_urllc_ue.size()-1].first << " , excess data rate :" << excess_urllc_ue[excess_urllc_ue.size()-1].second << "\n";
        AP_allocate_power[0][insufficient_normal_ue[insufficient_normal_ue.size()-1].first] += AP_allocate_power[0][excess_urllc_ue[excess_urllc_ue.size()-1].first] - minimum_rf_allocate_percentage[excess_urllc_ue[excess_urllc_ue.size()-1].first];
        AP_allocate_power[0][excess_urllc_ue[excess_urllc_ue.size()-1].first] = minimum_rf_allocate_percentage[excess_urllc_ue[excess_urllc_ue.size()-1].first];

        updateAllRFSINR(RF_SINR_vector,RF_channel_gain_vector,AP_allocate_power);
        updateAllVlcSINR(VLC_LOS_matrix,VLC_SINR_matrix,local_AP_association_matrix,AP_allocate_power,AP_serve_num);
        calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);
        calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);

        UE_final_data_rate_vector = std::vector<double> (UE_num,0.0) ;
        for(int i = 0 ; i < UE_num ; i++){
            UE_final_data_rate_vector[i] += roundNumber(RF_data_rate_vector[i], 3);
            for(int j = 0 ; j < VLC_AP_num ; j++){
                UE_final_data_rate_vector[i] += roundNumber(VLC_data_rate_matrix[j][i],3);
            }
        }
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
}

void cal_performance(std::vector<std::vector<int>> &AP_association_matrix,
                     std::vector<MyUeNode> &my_UE_list,
                     std::vector<std::vector<double>> &AP_allocate_power,
                     std::vector<std::vector<double>> &VLC_LOS_matrix,
                     std::vector<std::vector<double>> &VLC_SINR_matrix,
                     std::vector<std::vector<double>> &VLC_data_rate_matrix,
                     std::vector<double> &RF_channel_gain_vector,
                     std::vector<double> &RF_SINR_vector,
                     std::vector<double> &RF_data_rate_vector,
                     std::vector<double> &UE_final_data_rate_vector,
                     std::vector<double> &UE_final_satisfaction_vector,
                     std::vector<double> &UE_require_data_rate){
    // calculate final UE data rate
    std::vector<int> AP_serve_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);
    updateAllRFSINR(RF_SINR_vector,RF_channel_gain_vector,AP_allocate_power);
    updateAllVlcSINR(VLC_LOS_matrix,VLC_SINR_matrix,AP_association_matrix,AP_allocate_power,AP_serve_num);
    calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);
    calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);
    UE_final_data_rate_vector = std::vector<double> (UE_num,0.0) ;

    for(int i = 0 ; i < UE_num ; i++){
        /*std::cout << i << " VLC curr : " << my_UE_list[i].getCurrVlcAssociatedAP() << " ,pre : " << my_UE_list[i].getPrevVlcAssociatedAP() << "\n";
        std::cout << i << " RF curr : " << my_UE_list[i].getCurrRFAssociatedAP() << " ,pre : " << my_UE_list[i].getPrevRFAssociatedAP() << "\n";*/
        double handover = 0;
        if(my_UE_list[i].getCurrVlcAssociatedAP() == my_UE_list[i].getPrevVlcAssociatedAP() && my_UE_list[i].getCurrRFAssociatedAP() == my_UE_list[i].getPrevRFAssociatedAP()){
            handover = 1;
        }
        else if(my_UE_list[i].getCurrVlcAssociatedAP() != my_UE_list[i].getPrevVlcAssociatedAP() && my_UE_list[i].getCurrRFAssociatedAP() == my_UE_list[i].getPrevRFAssociatedAP()){
            handover = eta_hho;
        }
        else{
            handover = eta_vho;
        }
        UE_final_data_rate_vector[i] += roundNumber(RF_data_rate_vector[i], 5) * handover;
        for(int j = 0 ; j < VLC_AP_num ; j++){
            UE_final_data_rate_vector[i] += roundNumber(VLC_data_rate_matrix[j][i],5) * handover;
        }
        if(my_UE_list[i].getCurrVlcAssociatedAP() != 0 && my_UE_list[i].getCurrRFAssociatedAP() != 0 && my_UE_list[i].getCurrVlcAssociatedAP() != -1){
            UE_final_data_rate_vector[i] = UE_final_data_rate_vector[i] * la_overhead;
        }
    }

    // calculate user satisfaction
    std::vector<double> US_reliability = std::vector<double>(UE_num,0.0);
    std::vector<double> US_latency = std::vector<double> (UE_num,0.0);
    std::vector<double> US_datarate = std::vector<double> (UE_num,0.0);
    cal_US_Reliability(RF_SINR_vector,VLC_SINR_matrix,US_reliability);
    cal_US_Latency(US_latency,UE_final_data_rate_vector);
    cal_US_DataRate(UE_final_data_rate_vector,UE_require_data_rate,US_datarate);

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        if(UE_final_data_rate_vector[UE_index] < UE_require_data_rate[UE_index]){
            UE_final_satisfaction_vector[UE_index] = 0;
        }
        else{
            if(my_UE_list[UE_index].getGroup() == 1){ // urllc
                UE_final_satisfaction_vector[UE_index] = (0.4 * US_reliability[UE_index]) + (0.4 * US_latency[UE_index]) + (0.2 * US_datarate[UE_index]);
            }
            if(my_UE_list[UE_index].getGroup() == 2){ // normal
                UE_final_satisfaction_vector[UE_index] = (0.1 * US_reliability[UE_index]) + (0.1 * US_latency[UE_index]) + (0.8 * US_datarate[UE_index]);
            }
        }
    }

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
                if(SINR_to_dB(VLC_SINR_matrix[AP_index-1][UE_index]) > SINR_threshold){
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
