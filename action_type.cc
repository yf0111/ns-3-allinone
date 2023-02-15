#include <iostream>

#include "action_type.h"
#include "global_configuration.h"

Action_type::Action_type(){
    action_ap = std::vector<std::vector<int>> (RF_AP_num+VLC_AP_num, std::vector<int> (UE_num,0));
    action_subchannel = std::vector<std::vector<std::vector<int>>> (RF_AP_num+VLC_AP_num , std::vector<std::vector<int>> (RF_AP_subchannel , std::vector<int> (UE_num,0)));
    action_power = std::vector<std::vector<double>> (RF_AP_num+VLC_AP_num , std::vector<double>(UE_num,0.0));
}


void Action_type::setActionAP(int AP_index, int UE_index , int setnum){
    action_ap[AP_index][UE_index] = setnum;
}

void Action_type::setActionSubChannel(int AP_index , int sub_channel_index , int UE_index , int setnum){
    action_subchannel[AP_index][sub_channel_index][UE_index] = setnum;
}

void Action_type::setActionPower(int AP_index , int UE_index , double setnum){
    action_power[AP_index][UE_index] = setnum;
}

bool Action_type::getActionAP(int AP_index,int UE_index){
    if(action_ap[AP_index][UE_index] == 1){
        return true;
    }
    else{
        return false;
    }
}

bool Action_type::getActionSubChannel(int AP_index, int sub_channel_index , int UE_index){
    if(action_subchannel[AP_index][sub_channel_index][UE_index] == 1){
        return true;
    }
    else{
        return false;
    }
}

double Action_type::getActionPower(int AP_index , int UE_index){
    return action_power[AP_index][UE_index];
}
