/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//
// Network topology
//
//           10Mb/s, 10ms       10Mb/s, 10ms
//       n0-----------------n1-----------------n2
//
//
// - Tracing of queues and packet receptions to file
//   "tcp-large-transfer.tr"
// - pcap traces also generated in the following files
//   "tcp-large-transfer-$n-$i.pcap" where n and i represent node and interface
// numbers respectively
//  Usage (e.g.): ./waf --run tcp-large-transfer

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "global_configuration.h"
#include "install_mobility.h"
#include "my_UE_node.h"
#include "my_UE_node_list.h"
#include "channel.h"
#include "print.h"
#include "benchmark.h"
#include "proposed_method.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TcpLargeTransfer");
std::vector<double> Received(1, 0.0);
std::vector<double> theTime(1, 0.0);


static int state = 0;
static double ue_satisfaction = 0;
static double total_ue_satisfaction = 0;
static double active_ue_satisfaction = 0;
static double total_active_ue_satisfaction = 0;
/*
    AP association matrix : AP_num x UE_num
    if AP i is association to UE j then (i,j) == 1

    AP_association_matrix[0] is RF AP
*/
std::vector<std::vector<int>> AP_association_matrix(RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0));

/*
    VLC
*/
std::vector<std::vector<double>> VLC_LOS_matrix(VLC_AP_num, std::vector<double> (UE_num, 0.0));
std::vector<std::vector<double>> VLC_SINR_matrix(VLC_AP_num, std::vector<double> (UE_num ,0.0)); // in dB
std::vector<std::vector<double>> VLC_data_rate_matrix(VLC_AP_num, std::vector<double> (UE_num, 0.0)); // in Mbps

/*
    RF
*/
std::vector<double> RF_channel_gain_vector(UE_num, 0.0);
std::vector<double> RF_SINR_vector(UE_num, 0.0);
std::vector<double> RF_data_rate_vector(UE_num, 0.0); // in Mbps

/*
    UE
*/
std::vector<double> UE_final_data_rate_vector(UE_num , 0.0);
std::vector<double> UE_require_data_rate(UE_num , 0.0);
std::vector<double> UE_final_satisfaction_vector(UE_num,0.0);
std::vector<std::vector<double>> AP_allocate_power = std::vector<std::vector<double>> (RF_AP_num + VLC_AP_num , std::vector<double> (UE_num,0.0)); // save allocate power for each UE (%)
std::vector<int> indoor_user_index;
/*
    performance evaluation
*/
std::vector<double> recorded_average_outage_probability(state_num,0.0); // UE average outage probability
std::vector<double> recorded_average_data_rate(state_num,0.0); // UE average data rate
std::vector<double> recorded_average_satisfaction(state_num,0.0); // UE average satisfaction (new)
std::vector<double> recorded_average_active_satisfaction(state_num,0.0); // UE average active satisfaction (new)
std::vector<double> recorded_average_old_satisfaction(state_num,0.0); // UE average satisfaction (old)
std::vector<double> recorded_average_urllc_satisfaction(state_num,0.0); // URLLC user average satisfaction
std::vector<double> recorded_average_urllc_active_satisfaction(state_num,0.0); // URLLC active user average satisfaction
std::vector<double> recorded_average_normal_satisfaction(state_num,0.0); //normal user average satisfaction
std::vector<double> recorded_average_normal_active_satisfaction(state_num,0.0); // normal active user average satisfaciton
std::vector<double> recorded_average_jain_fairness_index(state_num,0.0); // jain's fairness index
std::vector<int> recorded_indoor_user_num(state_num,0); // number of indoor user

static const uint32_t totalTxBytes = 10000000;
static uint32_t currentTxBytes = 0;
static const uint32_t writeSize = 1040;
uint8_t data[writeSize];

std::string path = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";

void StartFlow(Ptr<Socket>, Ipv4Address, uint16_t);
void WriteUntilBufferFull(Ptr<Socket>, uint32_t);

/*
convert anything to string function
*/
template <typename T>
std::string to_string(T x){
    std::stringstream ss;
    ss<<x;
    return ss.str();
}


/*
  used for tracing and calculating throughput
 */
static void RxEndAddress(Ptr<const Packet> p, const Address &address) {
    // appends on the received packet to the received data up until that packet
    // and adds that total to the end of the vector
    Received.push_back(Received.back() + p->GetSize());
    // total += p->GetSize();
    theTime.push_back(Simulator::Now().GetSeconds()); // keeps track of time during simulation that a packet is received
    double throughput = ((Received.back() * 8)) / theTime.back(); // goodput calculation
    std::cout << "Received.back() is :" << Received.back() << std::endl;
    std::cout << "Rx throughput value is :" << throughput << std::endl;
    std::cout << "Current time is :" << theTime.back() << std::endl;
    std::cout << "Received size: " << p->GetSize() << " at: " << Simulator::Now().GetSeconds() << "s"
              << "IP: " << InetSocketAddress::ConvertFrom(address).GetIpv4() << std::endl;
}


static void initialize() {
    state = 0;
    ue_satisfaction = 0;
    total_ue_satisfaction = 0;
    active_ue_satisfaction = 0;
    total_active_ue_satisfaction = 0;

    AP_association_matrix = std::vector<std::vector<int>> (RF_AP_num + VLC_AP_num, std::vector<int> (UE_num, 0));

    VLC_LOS_matrix = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double> (UE_num, 0.0));
    VLC_SINR_matrix = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double> (UE_num, 0.0));
    VLC_data_rate_matrix = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double> (UE_num, 0.0)); // in Mbps

    RF_channel_gain_vector = std::vector<double> (UE_num,0.0);
    RF_SINR_vector = std::vector<double> (UE_num,0.0);
    RF_data_rate_vector = std::vector<double> (UE_num, 0.0); // in Mbps

    UE_final_data_rate_vector = std::vector<double>(UE_num,0.0);
    UE_final_satisfaction_vector = std::vector<double>(UE_num,0.0);
    UE_require_data_rate = std::vector<double>(UE_num,0.0);
    AP_allocate_power = std::vector<std::vector<double>> (RF_AP_num + VLC_AP_num , std::vector<double> (UE_num,0.0));
    indoor_user_index.clear();

    recorded_average_outage_probability = std::vector<double>(state_num,0.0);
    recorded_average_data_rate = std::vector<double>(state_num,0.0);
    recorded_average_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_active_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_old_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_urllc_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_urllc_active_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_normal_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_normal_active_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_jain_fairness_index = std::vector<double>(state_num,0.0);
    recorded_indoor_user_num = std::vector<int>(state_num,0);
}

static struct timespec diff(struct timespec start, struct timespec end) {
    struct timespec tmp;
    if ((end.tv_nsec - start.tv_nsec) < 0) {
        tmp.tv_sec = end.tv_sec - start.tv_sec - 1;
        tmp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    }
    else {
        tmp.tv_sec = end.tv_sec - start.tv_sec;
        tmp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }

    return tmp;
}


static void updateToNextState(NodeContainer &RF_AP_node,
                       NodeContainer &VLC_AP_nodes,
                       NodeContainer &UE_nodes,
                       std::vector<MyUeNode> &my_UE_list)
{

#if PROPOSED_METHOD
    proposedLB(state, RF_AP_node, VLC_AP_nodes, UE_nodes,
                       VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                       RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                        AP_association_matrix, my_UE_list,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate,
                        ue_satisfaction,AP_allocate_power,active_ue_satisfaction,indoor_user_index);

#else
    benchmarkMethod(state, RF_AP_node, VLC_AP_nodes, UE_nodes,
                       VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                       RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                        AP_association_matrix, my_UE_list,UE_final_data_rate_vector,
                        UE_final_satisfaction_vector,UE_require_data_rate,indoor_user_index);
#endif


    /* CALCULATION OF PERFORMANCE METRICS */
    double indoor_user = (SUPER_DYNAMIC)? indoor_user_index.size() : UE_num;

    // UE average outage probability
    int outage_UE_number = 0;
    if(SUPER_DYNAMIC){
        for(int i = 0 ; i < indoor_user_index.size() ; i++){
            double thre = UE_require_data_rate[indoor_user_index[i]];
            if(UE_final_data_rate_vector[indoor_user_index[i]] < thre){
                outage_UE_number += 1;
            }
        }
    }
    else{
        for(int i = 0 ; i < UE_num ; i++){
            double thre = UE_require_data_rate[i];
            if(UE_final_data_rate_vector[i] < thre){
                outage_UE_number += 1;
            }
        }
    }
    recorded_average_outage_probability[state] = (double) outage_UE_number / indoor_user;

#if DEBUG_MODE // outage user file output
    double avg_speed = 0.0;
    double total_speed = 0.0;
    double avg_require = 0.0;
    double total_require = 0.0;
    for(int i=0;i<UE_num;i++){
        total_speed += my_UE_list[i].getVelocity();
        total_require += UE_require_data_rate[i];
    }
    avg_speed = total_speed / UE_num;
    avg_require = total_require / UE_num;

    std::string path = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output;
    output.open(path + "outage_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);

    for(int i = 0 ; i < UE_num ; i++){
        double thre = 0;
        if (LASINR || LAEQOS && !PROPOSED_METHOD) thre = require_data_rate_threshold;
        if (PROPOSED_METHOD && !LASINR && !LAEQOS) thre = UE_require_data_rate[i];
        if(UE_final_data_rate_vector[i] < thre){
            std::string group ;
            if(my_UE_list[i].getGroup() == 1) group = "urllc";
            if(my_UE_list[i].getGroup() == 2) group = "normal";
            if (!output.is_open()) {
                std::cout << "Fail to open file\n";
                exit(EXIT_FAILURE);
            }
            else{
                output << i << "," << avg_speed << "," << avg_require << "," << group << "," << my_UE_list[i].getVelocity() << "," << my_UE_list[i].getCurrRFAssociatedAP() << "," << UE_require_data_rate[i] << "," << UE_final_data_rate_vector[i] << std::endl ;
            }
        }
    }
    output.close();
#endif // DEBUG_MODE

    // UE average data rate
    double total_UE_data_rate = 0;
    if(SUPER_DYNAMIC){
        for(int i = 0 ; i < indoor_user_index.size() ; i++){
            total_UE_data_rate += UE_final_data_rate_vector[indoor_user_index[i]];
        }
    }
    else{
        for(int i = 0 ; i < UE_num ; i++){
            total_UE_data_rate += UE_final_data_rate_vector[i];
        }
    }
    recorded_average_data_rate[state] = (double) total_UE_data_rate / indoor_user;

    // UE average satisfaction (old)
    double total_ue_old_satis = 0;
    if(SUPER_DYNAMIC){
        for (int i = 0 ; i < indoor_user_index.size() ; i++){
            double temp = (double) UE_final_data_rate_vector[indoor_user_index[i]] / UE_require_data_rate[indoor_user_index[i]];
            temp = std::min(temp,1.0);
            total_ue_old_satis += temp;
        }
    }
    else{
        for (int i = 0 ; i < UE_num ; i++){
            double temp = (double) UE_final_data_rate_vector[i] / UE_require_data_rate[i];
            temp = std::min(temp,1.0);
            total_ue_old_satis += temp;
        }
    }
    recorded_average_old_satisfaction[state] = (double) total_ue_old_satis / indoor_user;

    // URLLC user avg satisfaction (new)
    double urllc_UE_statis = 0;
    double urllc_active_UE_satis = 0;
    int urllc_cal = 0;
    if(SUPER_DYNAMIC){
        for(int i = 0 ; i < indoor_user_index.size() ; i++){
            if(my_UE_list[indoor_user_index[i]].getGroup() == 1){
                urllc_UE_statis += UE_final_satisfaction_vector[indoor_user_index[i]];
                if(UE_final_satisfaction_vector[indoor_user_index[i]] != 0){
                    urllc_cal += 1;
                    urllc_active_UE_satis += UE_final_satisfaction_vector[indoor_user_index[i]];
                }
            }
        }
    }
    else{
        for(int i = 0 ; i < UE_num ; i++){
            if(my_UE_list[i].getGroup() == 1){
                urllc_UE_statis += UE_final_satisfaction_vector[i];
                if(UE_final_satisfaction_vector[i] != 0){
                    urllc_cal += 1;
                    urllc_active_UE_satis += UE_final_satisfaction_vector[i];
                }
            }
        }
    }


    // normal user avg satisfaction (new)
    double normal_UE_satis = 0;
    double normal_active_UE_satis = 0;
    int normal_cal = 0;
    if(SUPER_DYNAMIC){
        for(int i = 0 ; i < indoor_user_index.size() ; i++){
            if(my_UE_list[indoor_user_index[i]].getGroup() == 2){
                normal_UE_satis += UE_final_satisfaction_vector[indoor_user_index[i]];
                if(UE_final_satisfaction_vector[indoor_user_index[i]] != 0){
                    normal_cal += 1;
                    normal_active_UE_satis += UE_final_satisfaction_vector[indoor_user_index[i]];
                }
            }
        }
    }
    else{
        for(int i = 0 ; i < UE_num ; i++){
            if(my_UE_list[i].getGroup() == 2){
                normal_UE_satis += UE_final_satisfaction_vector[i];
                if(UE_final_satisfaction_vector[i] != 0){
                    normal_cal += 1;
                    normal_active_UE_satis += UE_final_satisfaction_vector[i];
                }
            }
        }
    }


    recorded_average_urllc_satisfaction[state] = (urllc_UE_statis == 0)? 0:urllc_UE_statis / urllc_UE_num;
    recorded_average_urllc_active_satisfaction[state] = (urllc_active_UE_satis == 0)? 0:urllc_active_UE_satis / urllc_cal;

    recorded_average_normal_satisfaction[state] = (normal_UE_satis == 0) ? 0:normal_UE_satis / (indoor_user - urllc_UE_num);
    recorded_average_normal_active_satisfaction[state] = (normal_active_UE_satis == 0)? 0:normal_active_UE_satis / normal_cal;


    double urllc_percentage = urllc_UE_num / indoor_user;
    double normal_percentage = (indoor_user - urllc_UE_num) / indoor_user;

    recorded_average_satisfaction[state] = (urllc_percentage*recorded_average_urllc_satisfaction[state]) + (normal_percentage*recorded_average_normal_satisfaction[state]);
    recorded_average_active_satisfaction[state] = (urllc_percentage*recorded_average_urllc_satisfaction[state]) + (normal_percentage*recorded_average_normal_active_satisfaction[state]);


    total_ue_satisfaction += recorded_average_satisfaction[state];
    ue_satisfaction = total_ue_satisfaction / (state+1);

    total_active_ue_satisfaction += recorded_average_active_satisfaction[state];
    active_ue_satisfaction = total_active_ue_satisfaction / (state+1);

#if DEBUG_MODE // satisfaction & active user satisfaction file output
    // total user
    /*std::string path1 = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output1;

    output1.open(path1 + "satisfaction_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    if (!output1.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        output1 << recorded_average_satisfaction[state] << "," << recorded_average_active_satisfaction[state] << std::endl ;
    }
    output1.close();*/

    // urllc user
    std::string path2 = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output2;

    output2.open(path2 + "urllc_satisfaction_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    if (!output2.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        output2 << recorded_average_urllc_satisfaction[state] << "," << recorded_average_urllc_active_satisfaction[state] << std::endl ;
    }
    output2.close();

    // normal
    std::string path3 = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output3;

    output3.open(path3 + "normal_satisfaction_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    if (!output3.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        output3 << recorded_average_normal_satisfaction[state] << "," << recorded_average_normal_active_satisfaction[state] << std::endl ;
    }
    output3.close();

#endif // DEBUG_MODE

    // jain's fairness index
    double top = 0;
    double bottom = 0;
    if(SUPER_DYNAMIC){
        for(int i = 0 ; i < indoor_user_index.size() ; i++){
            if(LASINR || LAEQOS){
                double temp = (double) UE_final_data_rate_vector[indoor_user_index[i]] / UE_require_data_rate[indoor_user_index[i]];
                temp = std::min(temp,1.0);
                top += temp;
                bottom += std::pow(temp,2);
            }
            if(PROPOSED_METHOD){
                top += UE_final_satisfaction_vector[indoor_user_index[i]];
                bottom += std::pow(UE_final_satisfaction_vector[indoor_user_index[i]],2);
            }
        }
    }
    else{
        for(int i = 0 ; i < UE_num ; i++){
            if(LASINR || LAEQOS){
                double temp = (double) UE_final_data_rate_vector[i] / UE_require_data_rate[i];
                temp = std::min(temp,1.0);
                top += temp;
                bottom += std::pow(temp,2);
            }
            if(PROPOSED_METHOD){
                top += UE_final_satisfaction_vector[i];
                bottom += std::pow(UE_final_satisfaction_vector[i],2);
            }
        }
    }
    top = std::pow(top,2);
    bottom = bottom * indoor_user;
    recorded_average_jain_fairness_index[state] = top / bottom;

    // number of indoor user
    recorded_indoor_user_num[state] = indoor_user;

    std::cout << "state : " << state << "\n";
    std::cout << "one state avg outage: "<<recorded_average_outage_probability[state] << std::endl;
    std::cout << "one state avg satisfaction (old): "<<recorded_average_old_satisfaction[state] << std::endl;
    std::cout << "one state avg satisfaction (new): "<<recorded_average_satisfaction[state] << std::endl;
    std::cout << "one state avg activate satisfaction: " <<recorded_average_active_satisfaction[state] << std::endl;
    std::cout << "one state avg data rate: "<<recorded_average_data_rate[state] << std::endl;
    std::cout << "one state jain's fairness index:" << recorded_average_jain_fairness_index[state] << std::endl;
    std::cout << "one state number of indoor user:" << recorded_indoor_user_num[state] << std::endl<<std::endl;

    if (!Simulator::IsFinished())
        Simulator::Schedule(Seconds(time_period), &updateToNextState, RF_AP_node, VLC_AP_nodes, UE_nodes, my_UE_list);
    state++;
}

int main(int argc, char *argv[])
{
    struct timespec start, end;
    initialize();

    // create RF AP node
    NodeContainer RF_AP_node;
    RF_AP_node.Create(RF_AP_num);
    installRfApMobility(RF_AP_node);

#if DEBUG_MODE
    printRfApPosition(RF_AP_node);
#endif

    // create VLC AP nodes
    NodeContainer VLC_AP_nodes;
    VLC_AP_nodes.Create (VLC_AP_num);
    installVlcApMobility(VLC_AP_nodes);

#if DEBUG_MODE
    printVlcApPosition(VLC_AP_nodes);
#endif

    // create UE nodes
    NodeContainer UE_nodes;
    UE_nodes.Create (UE_num);
    installUeMobility(UE_nodes);

#if DEBUG_MODE
    printUePosition(UE_nodes);
#endif

    std::vector<MyUeNode> my_UE_list = initializeMyUeNodeList(UE_nodes);

    // start time
    clock_gettime(CLOCK_MONOTONIC, &start);


    Simulator::Schedule(Seconds(0.0), &updateToNextState, RF_AP_node, VLC_AP_nodes, UE_nodes, my_UE_list);
    Simulator::Stop(Seconds(state_num * time_period));

    Simulator::Run();


    // end time
    clock_gettime(CLOCK_MONOTONIC, &end);

    // overall avg. outage probability
    double avg_outage_probability = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_outage_probability += recorded_average_outage_probability[i];
    }
    avg_outage_probability = avg_outage_probability / state_num;

    // overall avg. satisfaction (old)
    double avg_old_satisfaction = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_old_satisfaction += recorded_average_old_satisfaction[i];
    }
    avg_old_satisfaction = avg_old_satisfaction / state_num;


    // overall avg. satisfaction (new)
    double avg_new_satisfaction = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_new_satisfaction += recorded_average_satisfaction[i];
    }
    avg_new_satisfaction = avg_new_satisfaction / state_num;

    // overall avg. data rate
    double avg_data_rate = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_data_rate += recorded_average_data_rate[i];
    }
    avg_data_rate = avg_data_rate / state_num;

    // calculate the actual executing time
    struct timespec tmp = diff(start, end);
    double exec_time = tmp.tv_sec + (double) tmp.tv_nsec / 1000000000.0;

    // overall avg. jain's fairness index
    double avg_jain_fairness_index = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_jain_fairness_index += recorded_average_jain_fairness_index[i];
    }
    avg_jain_fairness_index = avg_jain_fairness_index / state_num;

    // overall avg number of indoor user
    double avg_indoor_user = 0.0;
    for(int i = 0 ; i < state_num ; i++){
        avg_indoor_user += recorded_indoor_user_num[i];
    }
    avg_indoor_user = avg_indoor_user / state_num;


    std::cout << "overall avg outage: "<< avg_outage_probability << std::endl;
    std::cout << "overall avg satisfaction: "<<avg_new_satisfaction << std::endl;
    std::cout << "overall avg data rate: "<< avg_data_rate << std::endl;
    std::cout << "overall avg jain's fairness index:" << avg_jain_fairness_index << std::endl;
    std::cout << "overall avg number of indoor user:" << avg_indoor_user << std::endl<<std::endl;


    std::string method;
    if(PROPOSED_METHOD){
        method = "proposed";
    }
    else{
        if(LASINR){
            method = "benchmark-LASINR";
        }
        else{
            method = "benchmark-LAEQOS";
        }
    }

    std::cout << "In this simulation :(" << method << "), UE=" << std::to_string(UE_num) << ", execution time: " << exec_time << std::endl << std::endl;

    std::fstream output;
    if(LASINR || LAEQOS){
        output.open(path + method + ",UE=" + std::to_string(UE_num) + ",LA=" + std::to_string(LA_UE_num) + ".csv", std::ios::out | std::ios::app);
    }
    else{
        output.open(path + method + ",UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    }

    if (!output.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        if(LASINR || LAEQOS){
            output << avg_outage_probability << "," << avg_old_satisfaction << "," << avg_data_rate << "," << exec_time << "," << avg_jain_fairness_index;
        }
        else{
            output << avg_outage_probability << "," << avg_new_satisfaction << "," << avg_data_rate << "," << exec_time << "," << avg_jain_fairness_index;
        }
        if(SUPER_DYNAMIC){
            output << "," << avg_indoor_user;
        }
        output << std::endl;
    }
    output.close();
    Simulator::Destroy();
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//begin implementation of sending "Application"

void StartFlow (Ptr<Socket> localSocket,
                Ipv4Address servAddress,
                uint16_t servPort){
    NS_LOG_LOGIC ("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
    localSocket->Connect (InetSocketAddress (servAddress, servPort)); //connect

  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available
  //localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
    WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
    currentTxBytes=0;
}

void WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace){
    while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0){
        uint32_t left = totalTxBytes - currentTxBytes;
        uint32_t dataOffset = currentTxBytes % writeSize;
        uint32_t toWrite = writeSize - dataOffset;
        toWrite = std::min (toWrite, left);
        toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
        int amountSent = localSocket->Send (&data[dataOffset], toWrite, 0);
        if(amountSent < 0){
          // we will be called again when new tx space becomes available.
          return;
        }
        currentTxBytes += amountSent;
    }
    localSocket->Close ();
}
