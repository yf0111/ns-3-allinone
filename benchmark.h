#ifndef BENCHMARK_H
#define BENCHMARK_H


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "action_type.h"
#include "env_state_type.h"

void benchmarkMethod(int &state,
                     NodeContainer &RF_AP_node,
                     NodeContainer &VLC_AP_nodes,
                     NodeContainer &UE_nodes,
                     std::vector<std::vector<double>> &VLC_LOS_matrix,
                     std::vector<std::vector<double>> &VLC_SINR_matrix,
                     std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
                     std::vector<std::vector<double>> &VLC_data_rate_matrix,
                     std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix_3d,
                     std::vector<std::vector<std::vector<double>>> &VLC_allocated_power_3d,
                     std::vector<double> &RF_channel_gain_vector,
                     std::vector<double> &RF_SINR_vector,
                     std::vector<double> &RF_data_rate_vector,
                     std::vector<std::vector<double>> &RF_SINR_vector_2d,
                     std::vector<std::vector<double>> &RF_data_rate_vector_2d,
                     std::vector<std::vector<double>> &RF_allocated_power_2d,
                     std::vector<double> &RF_ICI_channel_gain_vector,
                     std::vector<std::vector<int>> &AP_association_matrix,
                     std::vector<MyUeNode> &my_UE_list,
                     std::vector<double> &UE_final_data_rate_vector,
                     std::vector<Env_state_type> &env_state_vec,
                     std::vector<Action_type> &action_vec,
                     std::vector<double> &value_func_vec,
                     std::map<Env_state_type,Action_type> &policy_map,
                     std::vector<double> &dqn_vec);

void PDS_ERT(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<std::vector<double>> &RF_SINR_vector_2d,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list,
             std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
             std::vector<Env_state_type> &env_state_vec,
             std::vector<Action_type> &action_vec,
             std::vector<double> &value_func_vec,
             std::map<Env_state_type,Action_type> &policy_map,
             std::vector<double> &dqn_vec);

void LA_SINR(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list);

void LA_EQOS(std::vector<std::vector<int>> &AP_association_matrix,
             std::vector<double> &RF_SINR_vector,
             std::vector<std::vector<double>> &VLC_SINR_matrix,
             std::vector<double> &UE_final_data_rate_vector,
             std::vector<MyUeNode> &my_UE_list);


double getSpectralEfficiency(double SINR);

void initializedStep(std::vector<Env_state_type> &env_state_vec,
                     std::vector<double> &value_func_vec,
                     std::map<Env_state_type,Action_type> &policy_map,
                     std::vector<double> &dqn_vec,
                     std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
                     std::vector<std::vector<double>> &RF_SINR_vector_2d);

double calculateReliability (std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d , std::vector<std::vector<double>> &RF_SINR_vector_2d);

void calculateDataRate(Env_state_type &now_env_state , std::vector<double> &init_data_rate);

std::vector<std::vector<double>> extend_vector_1to2d(std::vector<double> &extend_vector,int y_size);

void updateApAssociationResult(std::vector<std::vector<int>> &local_AP_sssociation_matrix,
                               std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list);

void updateResourceAllocationResult(std::vector<std::vector<double>> &throughtput_per_iteration, std::vector<MyUeNode> &my_UE_list);

#endif // BENCHMARK_H
