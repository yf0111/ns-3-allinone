#ifndef PRINT_H
#define PRINT_H

#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix);

void printVlcSinrMatrix(std::vector<std::vector<double>> &VLC_SINR_matrix);

void printVlcDataRateMatrix(std::vector<std::vector<double>> &VLC_data_rate_matrix);

void printRFChannelGainVector(std::vector<double> &RF_channel_gain_vector);

void printRFSINRVector(std::vector<double> &RF_SINR_vector);

void printRFDataRateVector(std::vector<double> &RF_data_rate_vector);

void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix);

void printApAllocatePowerMatrix(std::vector<std::vector<double>> &AP_allocate_power);

void printRfApPosition(ns3::NodeContainer &RF_AP_node);

void printVlcApPosition(ns3::NodeContainer &VLC_AP_nodes);

void printUePosition(ns3::NodeContainer &UE_nodes);

void printUeVelocity(ns3::NodeContainer &VLC_AP_nodes);

void printUePosition(std::vector<MyUeNode> &my_UE_list);

void printMyUeList(std::vector<MyUeNode> &my_UE_list);

void printAPServeUENum(std::vector<int> &AP_serve_UE_numbers);

void printUeFinalDataRate(std::vector<double> &UE_final_data_rate_vector);

void printIndoorUeIndex(std::vector<int> &indoor_user_index);
#endif
