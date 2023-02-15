#ifndef ACTION_TYPE_H
#define ACTION_TYPE_H

#include "global_configuration.h"

/*
    Action :
        AP selection
        sub channel assignment
        transmit power management
*/

class Action_type{
public:
    Action_type();
    void setActionAP(int AP_index,int UE_index, int setnum);
    void setActionSubChannel(int AP_index,int sub_channel_index ,int UE_index , int setnum);
    void setActionPower(int AP_index , int UE_index , double power);

    bool getActionAP(int AP_index,int UE_index);
    bool getActionSubChannel(int AP_index,int sub_channel_index , int UE_index);
    double getActionPower(int AP_index,int UE_index);

private:
    std::vector<std::vector<int>> action_ap; // (AP index , UE index)
    std::vector<std::vector<std::vector<int>>> action_subchannel; // (AP index , sub channel index , UE index)
    std::vector<std::vector<double>> action_power; // (AP index , UE index)
};


#endif // ACTION_TYPE_H
