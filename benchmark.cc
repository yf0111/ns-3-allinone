#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <random>
#include <chrono>


#include "print.h"
#include "channel.h"
#include "benchmark.h"
#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"



std::vector<int> initializedStep(std::vector<double> &RF_data_rate_vector,
                                std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix,
                                std::vector<std::vector<int>> &local_AP_association_matrix,
                                std::vector<std::vector<double>> &throughtput_per_iteration){
    // step1: construct strategy sets of the form {RF_AP, best VLC_AP} for all UEs
    std::vector<std::vector<int>> strategy_set (UE_num, std::vector<int> (2, -1)); // {{RF_AP_idx, best VLC_AP_idx}, {RF_AP_idx, best VLC_AP_idx}, ...}

    for (int i = 0; i < UE_num; i++) {
        strategy_set[i][0] = 0; // strategy set always contains the RF AP

        int best_VLC_AP = 0;
        double max_data_rate = -1;

        for (int j = 0; j < VLC_AP_num; j++) {
            double total_link_data_rate = 0.0;

            for (int k = 1; k < effective_subcarrier_num + 1; k++)
                total_link_data_rate += VLC_data_rate_matrix[j][i][k];

            total_link_data_rate *= time_slot_num;

            if (total_link_data_rate > max_data_rate) {
                max_data_rate = total_link_data_rate;
                best_VLC_AP = j;
            }
        }
        strategy_set[i][1] = best_VLC_AP + RF_AP_num; // VLC APs are indexed starting from 1
    }

    // step2: UE are randomly allocated to an AP from their strategy set (APA)
    std::vector<int> connected_AP (UE_num, -1); // for output
    std::vector<std::vector<int>> serving_UE (RF_AP_num + VLC_AP_num, std::vector<int> ()); // record the UEs served by a AP

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0, 1);

    // clear the local_AP_association_matrix
    for (int i = 0; i < local_AP_association_matrix.size(); i++)
        for (int j = 0; j < local_AP_association_matrix[0].size(); j++)
            local_AP_association_matrix[i][j] = 0;

    for (int i = 0; i < UE_num; i++) {
        int chosen_AP = strategy_set[i][distribution(generator)];

        local_AP_association_matrix[chosen_AP][i] = 1;
        connected_AP[i] = chosen_AP;
        serving_UE[chosen_AP].push_back(i);
    }


    //step3: resource allocation
    // if connected to RF AP -> equally divide RF AP resource
    for (int i = 0; i < RF_AP_num; i++) {
        for (int j = 0; j < serving_UE[i].size(); j++) {
            throughtput_per_iteration[serving_UE[i][j]].push_back(RF_data_rate_vector[serving_UE[i].size()]);
        }
    }

    // if connected to VLC AP -> OFDMA
    for (int i = RF_AP_num; i < VLC_AP_num + RF_AP_num; i++) {
        std::vector<double> data_rate = OFDMA(i-RF_AP_num, serving_UE[i], VLC_data_rate_matrix);

        for (int j = 0; j < serving_UE[i].size(); j++)
            throughtput_per_iteration[serving_UE[i][j]].push_back(data_rate[j]);
    }

    return connected_AP;
}



void updateApAssociationResult(std::vector<std::vector<int>> &local_AP_sssociation_matrix,
                               std::vector<std::vector<int>> &AP_sssociation_matrix,
                               std::vector<MyUeNode> &my_UE_list)
{
    AP_sssociation_matrix = local_AP_sssociation_matrix;

    // update every myUeNode
    for (int i = 0; i < local_AP_sssociation_matrix.size(); i++) {
        for (int j = 0; j < local_AP_sssociation_matrix[0].size(); j++) {
            if (local_AP_sssociation_matrix[i][j] == 1)
                my_UE_list[j].setCurrAssociatedAP(i);
        }
    }
}


void updateResourceAllocationResult(std::vector<std::vector<double>> &throughtput_per_iteration, std::vector<MyUeNode> &my_UE_list)
{
    for (int i = 0; i < my_UE_list.size(); i++) {
        double curr_throughput = throughtput_per_iteration[i].back();
        double curr_satisfaction = std::min(curr_throughput / my_UE_list[i].getRequiredDataRate(), 1.0);

        my_UE_list[i].addThroughput(curr_throughput);
        my_UE_list[i].addSatisfaction(curr_satisfaction);
    }
}
