#ifndef CHANNEL_H
#define CHANNEL_H

#include <map>

#include "global_configuration.h"
#include "my_UE_node.h"

using namespace ns3;


/*
 * a high-level function for calculating all channel-related information for VLC and RF,
 * including channel gain, SINR, achievable data rate
 */
void precalculation(NodeContainer &RF_AP_node ,
                      NodeContainer &VLC_AP_nodes ,
                      NodeContainer &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<std::vector<double>> &VLC_data_rate_matrix,
                      std::vector<double> &RF_channel_gain_vector,
                      std::vector<double> &RF_SINR_vector,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<MyUeNode> &my_UE_list);

/* VLC Line of sight */
double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes, std::vector<MyUeNode> &my_UE_list, std::vector<std::vector<double>> &VLC_LOS_matrix);
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node);
double estimateOneVlcNonLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node);
double getIrradianceAngle(Ptr<Node> AP, MyUeNode & UE_node); // in radians
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node);
double radian2Degree(const double &radian);
double degree2Radian(const double &degree);
double getDistance(Ptr<Node> AP, MyUeNode &UE_node); // in meters


/* VLC SINR */
void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,std::vector<std::vector<double>> &VLC_SINR_matrix);
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,int VLC_AP_index,int UE_index);


/* VLC data rate */
void calculateAllVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix,std::vector<std::vector<double>> &VLC_data_rate_matrix);
double estimateOneVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix,int VLC_AP_index, int UE_index);


/* RF channel gain */
void calculateRFChannelGain(NodeContainer &RF_AP_Node,NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<double> &RF_channel_gain_vector);
double estimateOneRFChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node);
double estimateOneRFICIChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node);


/* RF SINR */
void calculateALLRFSINR(std::vector<double> &RF_SINR_vector,std::vector<double> &RF_channel_gain_vector);
double estimateOneRFSINR(std::vector<double> &RF_channel_gain_vector,int UE_index);


/* RF data rate */
void calculateALLRFDataRate(std::vector<double> &RF_data_rate_vector,std::vector<double> &RF_SINR_vector);
double estimatOneRFDataRate(std::vector<double> &RF_SINR_vector,int UE_index);
#endif // CHANNEL_H
