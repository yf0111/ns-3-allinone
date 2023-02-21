#include <iostream>

#include "env_state_type.h"
#include "global_configuration.h"

Env_state_type::Env_state_type(){
    env_state_RF_subchannel = std::vector<std::vector<int>> (RF_AP_subchannel ,std::vector<int> (UE_num ,0));
    env_state_VLC_subchannel = std::vector<std::vector<std::vector<int>>> (VLC_AP_num,std::vector<std::vector<int>>(VLC_AP_subchannel,std::vector<int>(UE_num,0)));

    env_state_RF_SINR = std::vector<std::vector<double>> (RF_AP_subchannel,std::vector<double>(UE_num,0.0));
    env_state_VLC_SINR = std::vector<std::vector<std::vector<double>>>(VLC_AP_num , std::vector<std::vector<double>>(VLC_AP_subchannel,std::vector<double> (UE_num,0.0)));

    env_state_UEtype = std::vector<int> (UE_num,0);
    reliability = 0.0;
    latency = 0.0 ;
    mini_data_rate = 0.0;
}

void Env_state_type::setEnvStateRFSubChannel(int sub_channel_index , int UE_index , int setnum){
    env_state_RF_subchannel[sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateRFSubChannel(std::vector<std::vector<int>> &RF_subchannel_matrix){
    env_state_RF_subchannel = RF_subchannel_matrix;
}

void Env_state_type::setEnvStateVLCSubChannel(int VLC_AP_index , int sub_channel_index , int UE_index , int setnum){
    env_state_VLC_subchannel[VLC_AP_index][sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateVLCSubChannel(std::vector<std::vector<std::vector<int>>> &VLC_subchannel_matrix){
    env_state_VLC_subchannel = VLC_subchannel_matrix;
}

void Env_state_type::setEnvStateVLCSINR(int VLC_AP_index , int sub_channel_index , int UE_index , double setnum){
    env_state_VLC_SINR[VLC_AP_index][sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateVLCSINR(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d){
    env_state_VLC_SINR = VLC_SINR_matrix_3d;
}

void Env_state_type::setEnvStateRFSINR( int sub_channel_index , int UE_index , double setnum){
    env_state_RF_SINR[sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateRFSINR(std::vector<std::vector<double>> &RF_SINR_vector_2d){
    env_state_RF_SINR = RF_SINR_vector_2d;
}

void Env_state_type::setEnvStateUEtype(int UE_index , int setnum){
    env_state_UEtype[UE_index] = setnum;
}

void Env_state_type::setEnvStateSatisfaction_reliability (double setnum){
    reliability = setnum;
}

void Env_state_type::setEnvStateSatisfaction_latency(double setnum){
    latency = setnum;
}

void Env_state_type::setEnvStateSatisfaction_mini_data_rate (double setnum){
    mini_data_rate = setnum;
}

bool Env_state_type::getEnvStateVLCSubChannel (int VLC_AP_index , int sub_channel_index, int UE_index){
    if(env_state_VLC_subchannel[VLC_AP_index][sub_channel_index][UE_index] == 1){
        return true;
    }
    else{
        return false;
    }
}

bool Env_state_type::getEnvStateRFSubChannel ( int sub_channel_index, int UE_index){
    if(env_state_RF_subchannel[sub_channel_index][UE_index] == 1){
        return true;
    }
    else{
        return false;
    }
}

double Env_state_type::getEnvStateVLCSINR(int VLC_AP_index , int sub_channel_index , int UE_index){
    return env_state_VLC_SINR[VLC_AP_index][sub_channel_index][UE_index];
}

double Env_state_type::getEnvStateRFSINR( int sub_channel_index , int UE_index){
    return env_state_RF_SINR[sub_channel_index][UE_index];
}

int Env_state_type::getEnvStateUEtype(int UE_index){
    return env_state_UEtype[UE_index];
}

double Env_state_type::getEnvStateSatisfaction_reliability(void){
    return reliability;
}

double Env_state_type::getEnvStateSatisfaction_latency(void){
    return latency;
}

double Env_state_type::getEnvStateSatisfaction_mini_data_rate(void){
    return mini_data_rate;
}

void Env_state_type::printEnvStateRFSubChannel(void){
    std::cout<<" ***env_state RF sub channel matrix as below*** : \n";
    for (int j = 0; j < RF_AP_subchannel; j++) {
        std::cout << "\tFor sub channel " << j << ": \n";
        std::cout << "\t\t";

        for (int k = 0; k < UE_num; k++) {
            std::cout << env_state_RF_subchannel[j][k] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Env_state_type::printEnvStateVLCSubChannel(void){
    std::cout<<" ***env_state VLC sub channel matrix as below*** : \n";
    for (int i = 0; i < VLC_AP_num; i++) {
        std::cout << "For VLC AP " << i << ": \n";
        for (int j = 0; j < VLC_AP_subchannel; j++) {
            std::cout << "\tFor sub channel " << j << ": \n";
            std::cout << "\t\t";

            for (int k = 0; k < UE_num; k++) {
                std::cout << env_state_VLC_subchannel[i][j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Env_state_type::printEnvStateVLCSINR(void){
    std::cout<<" ***env_state VLC SINR matrix as below*** : \n";
    for (int i = 0; i < VLC_AP_num; i++) {
        std::cout << "For AP " << i << ": \n";
        for (int j = 0; j < VLC_AP_subchannel; j++) {
            std::cout << "\tFor sub channel " << j << ": \n";
            std::cout << "\t\t";

            for (int k = 0; k < UE_num; k++) {
                std::cout << env_state_VLC_SINR[i][j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Env_state_type::printEnvStateRFSINR(void){
    std::cout<<" ***env_state RF SINR matrix as below*** : \n";
    for (int j = 0; j < RF_AP_subchannel; j++) {
        std::cout << "\tFor sub channel " << j << ": \n";
        std::cout << "\t\t";
        for (int k = 0; k < UE_num; k++) {
            std::cout << env_state_RF_SINR[j][k] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

}

void Env_state_type::printEnvStateUEtype(void){
    std::cout<<" ***env_state UE type matrix as below*** : \n";
    for(int k = 0 ; k < UE_num ; k++){
        std::cout<<"\t"<<env_state_UEtype[k];
    }
    std::cout<<std::endl;
}

void Env_state_type::printEnvStateSatisfaction(void){
    std::cout<<" ***env_state Satisfaction turple as below*** : \n";
    std::cout << "\t reliability : " << reliability << "\n";
    std::cout << "\t latency : " << latency << "\n";
    std::cout << "\t mini_data_rate : " << mini_data_rate << "\n";
}
