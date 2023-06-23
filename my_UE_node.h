#ifndef MY_UE_NODE_H
#define MY_UE_NODE_H

#include <random>
#include <chrono>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


using namespace ns3;

class MyUeNode
{

public:
    MyUeNode(int node_ID, Vector pos, double required_data_rate);
    MyUeNode(int node_ID, Vector pos, double required_data_rate, int group);

    int getID(void);

    void setGroup(int UE_group);
    int getGroup(void);

    void setPosition(Vector pos_from_mobility_model);
    Vector getPosition(void);

    void setRequiredDataRate(double data_rate_in_Mbps);
    double getRequiredDataRate(void);

    void setCurrVlcAssociatedAP(int associated_AP_index);
    int getCurrVlcAssociatedAP(void);
    int getPrevVlcAssociatedAP(void);

    void setCurrRFAssociatedAP(int associated_AP_index);
    int getCurrRFAssociatedAP(void);
    int getPrevRFAssociatedAP(void);

    double getPolarAngle(void);
    double getAzimuthAngle(void);
    void randomOrientationAngle(Ptr<Node> UE);

    double getVelocity(void);
    void setVelocity(double new_velocity);

private:
    int node_ID;
    Vector pos;
    int group; // 1 for urllc UE , 2 for normal UE
    double required_data_rate;
    double polar_angle; // θ in rad
    double azimuth_angle; // ω in rad
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    // 0 indicates RF AP, and [1, VLC_AP_num] indicates VLC APs
    int prev_VLC_associated_AP;
    int curr_VLC_associated_AP;
    int prev_RF_associated_AP;
    int curr_RF_associated_AP;
    std::vector<double> throughput_per_state;
    std::vector<double> satisfaction_per_state;

    // a vector of positions that record start and end of used RU by this UE
    // of the form <<subcarrier start, time slot start>, <subcarrier end, time slot end>>
    //* std::vector<RuRangeType> RU_block;

    /*std::vector<RuType> RU_block;*/

    void setPolarAngle(double new_polar_angle);
    void setAzimuthAngle(double new_azimuth_angle);

    double velocity;
};


#endif // MY_UE_NODE_H

