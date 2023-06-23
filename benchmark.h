#ifndef BENCHMARK_H
#define BENCHMARK_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"

void benchmarkMethod(int &state,
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

void RL_LB(std::vector<std::vector<int>> &AP_association_matrix,
           std::vector<double> &RF_SINR_vector,
           std::vector<std::vector<double>> &VLC_SINR_matrix,
           std::vector<MyUeNode> &my_UE_list,
           std::vector<std::vector<double>> &VLC_data_rate_matrix,
           std::vector<double> &RF_data_rate_vector,
           std::multimap<std::vector<double>,int> &policy_map,
           std::vector<double> &UE_final_data_rate_vector,
           std::vector<double> &UE_final_satisfaction_vector);

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<double> &UE_final_satisfaction_vector,
             std::vector<double> &UE_require_data_rate);

void LA_EQOS(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<double> &UE_final_satisfaction_vector,
             std::vector<double> &UE_require_data_rate);

double getSpectralEfficiency(double SINR);

std::vector<std::vector<int>> generate_next_possible_AP_association(std::vector<std::vector<double>> &SINR_matrix,std::vector<int> &UE_type,std::vector<std::vector<int>> &State_SINR_VLC_index);

std::vector<std::vector<double>> combineSINRmatrix(std::vector<std::vector<double>> &VLC_SINR_matrix,std::vector<double> &RF_SINR_vector);

std::vector<std::vector<double>> SINR_matrix_to_two_high_SINR(std::vector<std::vector<double>> &SINR_matrix);

std::vector<std::vector<int>> SINR_matrix_to_two_high_AP_index(std::vector<std::vector<double>> &SINR_matrix);

std::vector<int> AP_association_matrix_to_UE_numbers(std::vector<std::vector<int>> &AP_association_matrix);

std::vector<std::vector<int>> AP_association_vector_to_matrix(std::vector<int> &AP_association_vector);

double calculatedR1(std::vector<int> &UE_type,std::vector<int> &pre_AP_association,std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,std::vector<double> &RF_data_rate_vector,
                 std::vector<int> &State_AP_load);

double calculatedR2(std::vector<int> &UE_type,std::vector<int> &pre_AP_association,std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,std::vector<double> &RF_data_rate_vector,
                 std::vector<int> &State_AP_load,std::vector<double> &UEdemands);

double calculatedR3(std::vector<int> &UE_type,std::vector<int> &pre_AP_association,std::vector<int> &AP_association,
                 std::vector<std::vector<double>> &VLC_data_rate_matrix,std::vector<double> &RF_data_rate_vector,
                 std::vector<int> &State_AP_load,std::vector<double> &UEdemands);

std::vector<double> createUEDemandVector(std::vector<MyUeNode> &my_UE_list);

void updateApAssociationResult(std::vector<std::vector<int>> &AP_sssociation_matrix,std::vector<MyUeNode> &my_UE_list);

#endif // BENCHMARK_H
