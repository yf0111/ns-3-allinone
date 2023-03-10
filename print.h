#ifndef PRINT_H
#define PRINT_H

#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

/*
    !*-*-NEW*-*-! 2023/01/10
    printVlcLosMatrix
    printVlcSinrMatrix
    printVlcDataRateMatrix
    printRFChannelGainVector
    printRFSINRMatrix

    !*-*-NEW*-*-! 2023/02/05
    printVlcSinrMatrix3d
    printVlcDataRateMatrix3d
*/
void printVlcLosMatrix(std::vector<std::vector<double>> &VLC_LOS_matrix);

void printVlcSinrMatrix(std::vector<std::vector<double>> &VLC_SINR_matrix);

//void printVlcSinrMatrix3d(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d);

void printVlcDataRateMatrix(std::vector<std::vector<double>> &VLC_data_rate_matrix);

//void printVlcDataRateMatrix3d(std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix_3d);

void printRFChannelGainVector(std::vector<double> &RF_channel_gain_vector);

void printRFSINRVector(std::vector<double> &RF_SINR_vector);

//void printRFSINRVector2d(std::vector<std::vector<double>> &RF_SINR_vector_2d);

void printRFDataRateVector(std::vector<double> &RF_data_rate_vector);

//void printRFDataRateVector2d(std::vector<std::vector<double>> &RF_data_rate_vector_2d);

void printApAssociationMatrix(std::vector<std::vector<int>> &AP_association_matrix);

void printRfApPosition(ns3::NodeContainer &RF_AP_node);

void printVlcApPosition(ns3::NodeContainer &VLC_AP_nodes);

void printUePosition(ns3::NodeContainer &UE_nodes);

void printUePosition(std::vector<MyUeNode> &my_UE_list);

void printMyUeList(std::vector<MyUeNode> &my_UE_list);

void printAPServeUENum(std::vector<int> &AP_serve_UE_numbers);

void printUEFinalDataRate(std::vector<double> &UE_final_data_rate_vector);

//* void printResourceUnitMatrix(std::vector<std::vector<std::vector<int>>> &resource_unit_matrix, int VLC_AP_index);

//* void printResourceUnitMatrix(std::vector<std::vector<int>> &RU_matrix);

//* void printBlockedUE(std::vector<int> &blocked_UE);

#endif
