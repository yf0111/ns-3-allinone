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
std::vector<double> user_need_power_vlc = std::vector<double> (UE_num,0.0); // <need power>
std::vector<double> user_need_power_rf = std::vector<double> (UE_num,0.0);

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
    /*std::vector<int> indoor_user_index;
    check_indoor_user(my_UE_list,indoor_user_index);
    std::cout << "indoor number of users:" << indoor_user_index.size() << ", index:";
    for(int i = 0 ; i < indoor_user_index.size(); i++){
        std::cout << indoor_user_index[i] << " ";
    }
    std::cout << "\n";*/



    precalculation(RF_AP_node,VLC_AP_nodes, UE_nodes,
                   VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                   RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                   my_UE_list);

    // initialize
    UE_require_data_rate = createUEDemandVector(my_UE_list);

    if(ue_satisfation - active_ue_satisfaction < 0.035){
        APS(VLC_SINR_matrix,AP_association_matrix,my_UE_list,UE_nodes);
        RA(AP_allocate_power,AP_association_matrix,UE_require_data_rate,RF_channel_gain_vector,VLC_LOS_matrix,my_UE_list);
        cal_performance(AP_association_matrix,my_UE_list,AP_allocate_power,VLC_LOS_matrix,VLC_SINR_matrix,VLC_data_rate_matrix,RF_channel_gain_vector,RF_SINR_vector,RF_data_rate_vector,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate);
        RAPS(VLC_LOS_matrix,UE_final_data_rate_vector,UE_require_data_rate,AP_association_matrix,AP_allocate_power);
        RA(AP_allocate_power,AP_association_matrix,UE_require_data_rate,RF_channel_gain_vector,VLC_LOS_matrix,my_UE_list);
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

void APS(std::vector<std::vector<double>> &VLC_SINR_matrix,
         std::vector<std::vector<int>> &AP_association_matrix,
         std::vector<MyUeNode> &my_UE_list,
         NodeContainer &UE_nodes){

    // [Wi-Fi : high speed normal device] [Li-Fi : other normal device and urllc device]
    AP_association_matrix = std::vector<std::vector<int>> (RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0));
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

}

void RA(std::vector<std::vector<double>> &AP_allocate_power,
        std::vector<std::vector<int>> &local_AP_association_matrix,
        std::vector<double> &UE_require_data_rate,
        std::vector<double> &RF_channel_gain_vector,
        std::vector<std::vector<double>> &VLC_LOS_matrix,
        std::vector<MyUeNode> &my_UE_list){

    AP_allocate_power = std::vector<std::vector<double>> (RF_AP_num + VLC_AP_num , std::vector<double> (UE_num,0.0));

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        double vlc_mini = 0;
        double rf_mini = 0;
        for(int AP_index = 0 ; AP_index < RF_AP_num+VLC_AP_num ; AP_index++){
            if(AP_index == 0 && local_AP_association_matrix[AP_index][UE_index] == 1){
                rf_mini = cal_minumum_allocate_power_percentage(UE_require_data_rate,RF_channel_gain_vector,VLC_LOS_matrix,UE_index,0);
            }
            if(AP_index != 0 && local_AP_association_matrix[AP_index][UE_index] == 1){
                vlc_mini = cal_minumum_allocate_power_percentage(UE_require_data_rate,RF_channel_gain_vector,VLC_LOS_matrix,UE_index,AP_index);
            }
        }
        user_need_power_vlc[UE_index] = vlc_mini;
        user_need_power_rf[UE_index] = rf_mini;
    }

    // calculate the EE (D/P) and sort in descending order
    for(int AP_index = 0 ; AP_index < RF_AP_num+VLC_AP_num ; AP_index++){
        std::vector<std::pair<int,double>> each_AP_allocate_power; // <user index,ee>
        each_AP_allocate_power.clear();
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            if(AP_index == 0 && local_AP_association_matrix[AP_index][UE_index] == 1){ // RF
                double ee = UE_require_data_rate[UE_index] / user_need_power_rf[UE_index];
                each_AP_allocate_power.push_back(std::make_pair(UE_index,ee));
            }
            if(AP_index != 0 && local_AP_association_matrix[AP_index][UE_index] == 1){ // VLC
                double ee = UE_require_data_rate[UE_index] / user_need_power_vlc[UE_index];
                each_AP_allocate_power.push_back(std::make_pair(UE_index,ee));
            }

        }
        sort(each_AP_allocate_power.begin(),each_AP_allocate_power.end(),sort_by_sec_descending);
        double total_allocate = 0;
        double max_allocate = (AP_index == 0)?RF_AP_power:VLC_AP_power;
        int now_index = 0;
        while(now_index < each_AP_allocate_power.size() && total_allocate < max_allocate){
            double power = (AP_index == 0)? user_need_power_rf[each_AP_allocate_power[now_index].first]:user_need_power_vlc[each_AP_allocate_power[now_index].first];
            AP_allocate_power[AP_index][each_AP_allocate_power[now_index].first] = power;
            total_allocate += power;
            now_index++;
        }
        while(now_index < each_AP_allocate_power.size()){
            local_AP_association_matrix[AP_index][each_AP_allocate_power[now_index].first] = 0;
            now_index++;
        }
    }

    // Residual RA
    RRA(local_AP_association_matrix,AP_allocate_power);
}

void RAPS(std::vector<std::vector<double>> &VLC_LOS_matrix,
          std::vector<double> &final_data_rate,
          std::vector<double> &require_data_rate,
          std::vector<std::vector<int>> &AP_association_matrix,
          std::vector<std::vector<double>> AP_allocate_power){

    std::vector<int> AP_serve_UE_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);

    // Link Aggregation
    std::vector<std::pair<int,double>> wifi_ee; // <user index, ee>
    wifi_ee.clear();

    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        wifi_ee.push_back(std::make_pair(UE_index,require_data_rate[UE_index] / user_need_power_rf[UE_index]));
    }
    sort(wifi_ee.begin(),wifi_ee.end(),sort_by_sec_descending);

    double wifi_total_allocate = 0;
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        wifi_total_allocate += AP_allocate_power[0][UE_index];
    }

    int index = 0;
    while(wifi_total_allocate < RF_AP_power && AP_serve_UE_num[0] < wifi_threshold && index < wifi_threshold){
        AP_association_matrix[0][wifi_ee[index].first] = 1;
        index++;
        AP_serve_UE_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);
    }


    // LiFi
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
            if( curr_VLC_AP_index[UE_index] != -1 && VLC_AP_index != curr_VLC_AP_index[UE_index] && VLC_SINR[curr_VLC_AP_index[UE_index]][UE_index] < VLC_IINR[VLC_AP_index][UE_index] ){
                AP_association_matrix[curr_VLC_AP_index[UE_index]+1][UE_index] = 0;
                AP_association_matrix[VLC_AP_index+1][UE_index] = 1;
            }
        }
    }
}

void RRA(std::vector<std::vector<int>> &AP_association_matrix,
         std::vector<std::vector<double>> &AP_allocate_power){

    std::vector<int> AP_serve_UE_num = AP_association_matrix_to_UE_numbers(AP_association_matrix);
    for(int AP_index = 0 ; AP_index < RF_AP_num+VLC_AP_num ; AP_index++){
        double total_allocate = 0;
        double max_allocate = (AP_index == 0)? RF_AP_power:VLC_AP_power;
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            total_allocate += AP_allocate_power[AP_index][UE_index];
        }
        if(total_allocate < max_allocate){
            double remain = max_allocate - total_allocate;
            double each = remain / AP_serve_UE_num[AP_index];
            for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
                if(AP_association_matrix[AP_index][UE_index] == 1){
                    AP_allocate_power[AP_index][UE_index] += each;
                    total_allocate += each;
                }
            }
        }
    }

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
        //std::cout <<  "UE:" << i << "," <<UE_final_data_rate_vector[i] << "," << UE_require_data_rate[i] << "\n";
    }

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
            if(my_UE_list[UE_index].getGroup() == 1){ // urllc 0.4:0.3:0.3
                UE_final_satisfaction_vector[UE_index] = (0.5 * US_reliability[UE_index]) + (0.4 * US_latency[UE_index]) + (0.1 * US_datarate[UE_index]);
            }
            if(my_UE_list[UE_index].getGroup() == 2){ // normal 0.3:0.2:0.5
                UE_final_satisfaction_vector[UE_index] = (0.3 * US_reliability[UE_index]) + (0.2 * US_latency[UE_index]) + (0.5 * US_datarate[UE_index]);
            }
        }
    }

}

double cal_minumum_allocate_power_percentage(std::vector<double> &UE_require_data_rate,
                                           std::vector<double> &RF_channel_gain_vector,
                                           std::vector<std::vector<double>> &VLC_LOS_matrix,
                                           int UE_index,
                                           int AP_index){
    double allocate = 0;
    if(AP_index == 0){ // RF
        allocate = ((RF_noise_power_spectral_density * RF_AP_bandwidth) / RF_channel_gain_vector[UE_index]) * ( pow(2,(UE_require_data_rate[UE_index]) / RF_AP_bandwidth) - 1 );
    }
    else{ //VLC
        double allocate2 = VLC_noise_power_spectral_density * VLC_AP_bandwidth * PI * EE * (std::pow(2,2*UE_require_data_rate[UE_index] / VLC_AP_bandwidth)-1) / std::pow(conversion_efficiency * VLC_LOS_matrix[AP_index-1][UE_index],2);
        allocate = std::sqrt(allocate2);
    }
    return allocate;
}

void cal_US_Reliability(std::vector<double> &RF_SINR_vector,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<double> &US_reliability,
                      std::vector<std::vector<int>> &AP_association_matrix){
    /* check receive SINR > threshold SINR or not , 1/0 */
    for( int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        int connect_AP = -1;
        for( int AP_index = 0 ; AP_index < RF_AP_num + VLC_AP_num ; AP_index ++){
            if(AP_association_matrix[AP_index][UE_index] == 1){
                connect_AP = AP_index;
            }
        }
        if(connect_AP != -1){
            //std::cout << SINR_to_dB(RF_SINR_vector[UE_index]) << "," << VLC_SINR_matrix[connect_AP-1][UE_index] << std::endl;
            if(connect_AP == 0 && (SINR_to_dB(RF_SINR_vector[UE_index]) > SINR_threshold)){
                US_reliability[UE_index] = 1;
            }
            if(connect_AP != 0 && (VLC_SINR_matrix[connect_AP-1][UE_index] > SINR_threshold)){
                US_reliability[UE_index] = 1;
            }
        }
        else{
            US_reliability[UE_index] = 0;
        }
    }
}

void cal_US_Latency(std::vector<double> &US_latency,
                    std::vector<double> &UE_final_data_rate_vector){
    /* TL = Tt + Ta + Tb + Tr + Tp , (TL < Tmax (1ms))? 1:0 */
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        double TL = 0.1 + 0.3 ; // Ta + Tb = 0.1ms , Tr + Tp = 0.3ms
        if(UE_final_data_rate_vector[UE_index] == 0){
            US_latency[UE_index] = 0;
        }
        else{
            TL += packet_size / (1e6 * UE_final_data_rate_vector[UE_index]) * 1e3;
            if( TL < T_max){
                US_latency[UE_index] = 1;
            }
        }
    }
}

void cal_US_DataRate(std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<double> &US_datarate,
                     std::vector<double> &US_latency,
                     std::vector<double> &US_reliability,
                     std::vector<MyUeNode> &my_UE_list){
    for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
        //std::cout << final_data_rate[UE_index] << "\t" << require_data_rate[UE_index] << "\t" <<  std::min((double)final_data_rate[UE_index] / require_data_rate[UE_index],1.0) << "\n";
        if(my_UE_list[UE_index].getGroup() == 1){ //urllc
            if(US_latency[UE_index] && US_reliability[UE_index]){
                US_datarate[UE_index] = std::min(final_data_rate[UE_index] / require_data_rate[UE_index],1.0) ;
            }
            else{
                US_datarate[UE_index] = 0;
            }
        }
        else if(my_UE_list[UE_index].getGroup() == 2){ // normal
            US_datarate[UE_index] = std::min(final_data_rate[UE_index] / require_data_rate[UE_index],1.0) ;
        }
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

void check_indoor_user(std::vector<MyUeNode> &my_UE_list,
                       std::vector<int> &indoor_user_index){
    /*indoor_user_index.clear();
    for(int i = 0 ; i < UE_num_max ; i++){
        Vector pos = my_UE_list[i].getPosition();
        if(pos.x <= room_size / 2 && pos.x >= -room_size / 2 && pos.y <= room_size / 2 && pos.y >= -room_size / 2){
            indoor_user_index.push_back(i);
        }
    }*/
}
