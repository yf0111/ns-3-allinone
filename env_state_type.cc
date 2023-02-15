#include <iostream>

#include "env_state_type.h"
#include "global_configuration.h"

Env_state_type::Env_state_type(){
    env_state_subchannel = std::vector<std::vector<std::vector<int>>> (RF_AP_num+VLC_AP_num , std::vector<std::vector<int>>(RF_AP_subchannel,std::vector<int> (UE_num,0)));
    env_state_SINR = std::vector<std::vector<std::vector<double>>> (RF_AP_num+VLC_AP_num , std::vector<std::vector<double>>(RF_AP_subchannel,std::vector<double>(UE_num,0.0)));
    env_state_UEtype = std::vector<int> (UE_num,0);
    env_state_satisfaction = std::vector<std::vector<double>>(UE_num , std::vector<double>(3,0.0));
}

void Env_state_type::setEnvStateSubChannel(int AP_index , int sub_channel_index , int UE_index , int setnum){
    env_state_subchannel[AP_index][sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateSINR(int AP_index , int sub_channel_index , int UE_index , double setnum){
    env_state_SINR[AP_index][sub_channel_index][UE_index] = setnum;
}

void Env_state_type::setEnvStateUEtype(int UE_index , int setnum){
    env_state_UEtype[UE_index] = setnum;
}

void Env_state_type::setEnvStateSatisfaction (int UE_index , int whichSatis , double setnum){
    env_state_satisfaction[UE_index][whichSatis] = setnum;
}

bool Env_state_type::getEnvStateSubChannel (int AP_index , int sub_channel_index, int UE_index){
    if(env_state_subchannel[AP_index][sub_channel_index][UE_index] == 1){
        return true;
    }
    else{
        return false;
    }
}

double Env_state_type::getEnvStateSINR(int AP_index , int sub_channel_index , int UE_index){
    return env_state_SINR[AP_index][sub_channel_index][UE_index];
}

int Env_state_type::getEnvStateUEtype(int UE_index){
    return env_state_UEtype[UE_index];
}

double Env_state_type::getEnvStateSatisfaction(int UE_index , int whichSatis){
    return env_state_satisfaction[UE_index][whichSatis];
}

void Env_state_type::printEnvStateSubChannel(void){
    std::cout<<" ***env_state sub channel matrix as below*** : \n";
    for (int i = 0; i < RF_AP_num+VLC_AP_num; i++) {
        std::cout << "For AP " << i << ": \n";
        for (int j = 0; j < RF_AP_subchannel; j++) {
            std::cout << "\tFor sub channel " << j << ": \n";
            std::cout << "\t\t";

            for (int k = 0; k < UE_num; k++) {
                std::cout << env_state_subchannel[i][j][k] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Env_state_type::printEnvStateSINR(void){
    std::cout<<" ***env_state SINR matrix as below*** : \n";
    for (int i = 0; i < RF_AP_num+VLC_AP_num; i++) {
        std::cout << "For AP " << i << ": \n";
        for (int j = 0; j < RF_AP_subchannel; j++) {
            std::cout << "\tFor sub channel " << j << ": \n";
            std::cout << "\t\t";

            for (int k = 0; k < UE_num; k++) {
                std::cout << env_state_SINR[i][j][k] << " ";
            }
            std::cout << std::endl;
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
    std::cout<<" ***env_state Satisfaction matrix as below*** : \n";
    for (int k = 0; k < UE_num; k++) {
        std::cout << "For UE " << k << ": \n";
        for (int p = 0; p < 3; p++) {
            std::cout << "\t "<< env_state_satisfaction[k][p];
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
