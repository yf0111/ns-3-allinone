#ifndef PROPOSED_METHOD_H
#define PROPOSED_METHOD_H

#include <vector>

#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void proposedStaticLB(int &state,
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
                     std::vector<double> &UE_final_data_rate_vector);


double calDataRate(std::vector<double> &RF_SINR_vector,
                   std::vector<std::vector<double>> &VLC_SINR_matrix,
                   std::vector<std::vector<double>> &AP_allocate_time,
                   int AP_index,
                   int UE_index);


void calReliability(std::vector<double> &RF_SINR_vector,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<double> &US_reliability);

double SINR_to_dB(double SINR);

double dB_to_SINR(double dB);
#endif // PROPOSED_METHOD_H
