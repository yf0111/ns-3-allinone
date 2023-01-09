#ifndef MY_UE_NODE_H
#define MY_UE_NODE_H

#include <random>
#include <chrono>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


using namespace ns3;

//* typedef std::pair<std::pair<int,int>, std::pair<int,int>> RuRangeType;
typedef std::pair<int,int> RuType; // <sub channel index , time>

enum Order {high_to_low, low_to_high}; //?

class MyUeNode
{

public:
    MyUeNode(int node_ID, Vector pos, double required_data_rate);

    int getID(void);

    void setPosition(Vector pos_from_mobility_model);
    Vector getPosition(void);

    void setRequiredDataRate(double data_rate_in_Mbps);
    double getRequiredDataRate(void);

    void setCurrAssociatedAP(int associated_AP_index); // diff?
    void changeCurrAssociatedAP(int associated_AP_index); // diff?
    int getCurrAssociatedAP(void);
    int getPrevAssociatedAP(void);

    void addThroughput(double new_data_rate);
    double getLastThroughput(void);
    double calculateAvgThroughput(void);
    std::vector<double> getThroughputHistory(void);

    void addSatisfaction(double new_satis);
    double getLastSatisfaction(void);
    double calculateAvgSatisfaction(void);
    std::vector<double> getSatisfactionHistory(void);

    double getPolarAngle(void);
    double getAzimuthAngle(void);
    void randomOrientationAngle(Ptr<Node> UE);

    // 2023/01/09 : NEED CHANGE !
    void recordResourceUnit(std::pair<int,int> newRu);
    //* void updateNthResourceUnitBlock(int n, RuRangeType new_RU);
    void updateNthResourceUnitBlock(int n,RuType new_Ru);
    //* RuRangeType getNthResourceUnitBlock(int n);
    RuType getNthResourceUnitBlock(int n);
    void removeNthResourceUnitBlock(int n);
    void removeLastResourceUnitBlock(void);
    int getRuBlockSize(void);
    void clearRuBlock(void);
    std::vector<RuRangeType> getWholeRuBlock(void);
    //* void arrangeRuBlock(Order order);


private:
    int node_ID;
    Vector pos;
    double required_data_rate;
    double polar_angle; // θ in rad
    double azimuth_angle; // ω in rad
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    // 0 indicates RF AP, and [1, VLC_AP_num] indicates VLC APs
    int prev_associated_AP;
    int curr_associated_AP;
    std::vector<double> throughput_per_state;
    std::vector<double> satisfaction_per_state;

    // a vector of positions that record start and end of used RU by this UE
    // of the form <<subcarrier start, time slot start>, <subcarrier end, time slot end>>
    //* std::vector<RuRangeType> RU_block;

    std::vector<RuType> RU_block;

    void setPolarAngle(double new_polar_angle);
    void setAzimuthAngle(double new_azimuth_angle);
};


#endif // MY_UE_NODE_H

