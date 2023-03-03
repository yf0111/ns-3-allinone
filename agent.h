#ifndef AGENT_H
#define AGENT_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


using namespace ns3;

class Agent
{

public:
    Agent(std::vector<std::vector<double>> &State_SINR, std::vector<int> &State_AP_load,std::vector<int> &Action);

private:
    /* state
        1. SNR between the user and various APs (UE_num x 3 , entry is Wifi SNR + highest VLC SINR + second VLC SINR)
        2. Current load on each AP (AP_num , entry is number of users connected to AP <int>)
    */

    /* action
        0. WiFi AP
        1. LiFi AP 0 (highest SINR)
        2. LiFi AP 1 (second SINR)
        3. WiFi + LiFi AP (highest SINR)
        4. WiFi + LiFi AP (second SINR)
    */
    std::vector<std::vector<double>> State_SINR; // UE x 3
    std::vector<int> State_AP_load; // AP , entry is numbers of UE connected to AP
    std::vector<int> Action; // UE_num , entry is 0~4
};


#endif // AGENT_H

