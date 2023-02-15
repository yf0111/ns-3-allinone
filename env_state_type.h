#ifndef ENV_STATE_TYPE_H
#define ENV_STATE_TYPE_H


#include "global_configuration.h"

/*
    State
        sub channel occupy status (idle or busy),
        the channel quality (SINR value),
        the service application types (normal services (low priority) and URLLC services (high priority)),
        service satisfaction (reliability, latency, and minimum data rate)
*/



class Env_state_type{
public:
    Env_state_type();

    void setEnvStateSubChannel(int AP_index ,int sub_channel_index , int UE_index ,int setnum );

    void setEnvStateSINR (int AP_index ,int sub_channel_index ,int UE_index ,double setnum);

    void setEnvStateUEtype (int UE_index , int setnum);

    void setEnvStateSatisfaction (int UE_index , int whichSatis , double setnum);

    bool getEnvStateSubChannel (int AP_index ,int sub_channel_index , int UE_index);

    double getEnvStateSINR (int AP_index , int sub_channel_index , int UE_index);

    int getEnvStateUEtype(int UE_index);

    double getEnvStateSatisfaction(int UE_index , int whichSatis);

    void printEnvStateSubChannel(void);

    void printEnvStateSINR(void);

    void printEnvStateUEtype(void);

    void printEnvStateSatisfaction(void);
private:

    std::vector<std::vector<std::vector<int>>> env_state_subchannel; // (AP index , sub channel index , UE index )
    std::vector<std::vector<std::vector<double>>> env_state_SINR; // (AP index, sub channel index , UE index)
    std::vector<int> env_state_UEtype; // (UE index) (1 is for normal service , 2 is for URLLC service)
    std::vector<std::vector<double>> env_state_satisfaction; // (UE index , (reliability , latency , minimum data rate)(0,1,2))
};

#endif // ENV_STATE_TYPE_H
