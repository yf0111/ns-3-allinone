#ifndef PROPOSED_METHOD_H
#define PROPOSED_METHOD_H

#include <vector>

#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"


void proposedLB(int &state,
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
                     std::vector<double> &UE_final_data_rate_vector,
                     std::vector<double> &UE_final_satisfaction_vector,
                     std::vector<double> &UE_require_data_rate);

void RA(std::vector<std::vector<double>> &AP_allocate_power,
        std::vector<std::vector<int>> &local_AP_association_matrix,
        std::vector<double> &UE_require_data_rate);

void APS(std::vector<std::vector<double>> &VLC_SINR_matrix,
         std::vector<std::vector<int>> &AP_association_matrix,
         std::vector<MyUeNode> &my_UE_list,
         NodeContainer &UE_nodes);

void RRA(std::vector<double> &minimum_rf_allocate_percentage,
                    std::vector<double> &UE_require_data_rate,
                    std::vector<double> &UE_final_data_rate_vector,
                    std::vector<std::vector<double>> &VLC_LOS_matrix,
                    std::vector<std::vector<double>> &VLC_SINR_matrix,
                    std::vector<std::vector<double>> &VLC_data_rate_matrix,
                    std::vector<double> &RF_channel_gain_vector,
                    std::vector<double> &RF_SINR_vector,
                    std::vector<double> &RF_data_rate_vector,
                    std::vector<MyUeNode> &my_UE_list,
                    std::vector<std::vector<int>> &local_AP_association_matrix,
                    std::vector<std::vector<double>> &AP_allocate_power);

void RAPS(std::vector<double> &RF_SINR_vector,
                     std::vector<std::vector<double>> &VLC_SINR_matrix,
                     std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<std::vector<int>> &AP_association_matrix);

void cal_minumum_allocate_power_percentage(std::vector<double> &minimum_rf_allocate_percentage,
                                           std::vector<double> &UE_require_data_rate,
                                           std::vector<double> &RF_channel_gain_vector);

void cal_US_Reliability(std::vector<double> &RF_SINR_vector,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<double> &US_reliability);

void cal_US_Latency(std::vector<double> &US_latency,
                    std::vector<double> &UE_final_data_rate_vector);

void cal_US_DataRate(std::vector<double> &final_data_rate,
                     std::vector<double> &require_data_rate,
                     std::vector<double> &US_datarate);

double SINR_to_dB(double SINR);

double dB_to_SINR(double dB);

bool sort_by_sec_descending(const std::pair<int,double> &a, const std::pair<int,double> &b);

bool sort_by_sec_ascending(const std::pair<int,double> &a, const std::pair<int,double> &b);

double roundNumber(double oriNum, int bits);
#endif // PROPOSED_METHOD_H
