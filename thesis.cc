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
/*
    performance evaluation
*/
std::vector<double> recorded_average_outage_probability(state_num,0.0); // UE average outage probability
std::vector<double> recorded_average_data_rate(state_num,0.0); // UE average data rate
std::vector<double> recorded_average_satisfaction(state_num,0.0); // UE average satisfaction (new)
std::vector<double> recorded_average_active_satisfaction(state_num,0.0); // UE average active satisfaction (new)
std::vector<double> recorded_average_old_satisfaction(state_num,0.0); // UE average satisfaction (old)

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

    recorded_average_outage_probability = std::vector<double>(state_num,0.0);
    recorded_average_data_rate = std::vector<double>(state_num,0.0);
    recorded_average_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_active_satisfaction = std::vector<double>(state_num,0.0);
    recorded_average_old_satisfaction = std::vector<double>(state_num,0.0);
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
#if DEBUG_MODE
    std::cout << "state : " << state << "\n";
    printUePosition(UE_nodes);
    printUEVelocity(UE_nodes);
#endif


#if PROPOSED_METHOD

    proposedLB(state, RF_AP_node, VLC_AP_nodes, UE_nodes,
                       VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                       RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                        AP_association_matrix, my_UE_list,UE_final_data_rate_vector,UE_final_satisfaction_vector,UE_require_data_rate,ue_satisfaction,AP_allocate_power,active_ue_satisfaction);

#else
    benchmarkMethod(state, RF_AP_node, VLC_AP_nodes, UE_nodes,
                       VLC_LOS_matrix, VLC_SINR_matrix, VLC_data_rate_matrix,
                       RF_channel_gain_vector, RF_SINR_vector, RF_data_rate_vector,
                        AP_association_matrix, my_UE_list,UE_final_data_rate_vector,
                        UE_final_satisfaction_vector,UE_require_data_rate);
#endif


    /* CALCULATION OF PERFORMANCE METRICS */

    // UE average outage probability
    int outage_UE_number = 0;
    for(int i = 0 ; i < UE_num ; i++){
        double thre = UE_require_data_rate[i];
        /*if (LASINR || LAEQOS && !PROPOSED_METHOD) thre = require_data_rate_threshold;
        if (PROPOSED_METHOD && !LASINR && !LAEQOS) thre = UE_require_data_rate[i];*/
        if(UE_final_data_rate_vector[i] < thre){
            outage_UE_number += 1;
        }
    }
    recorded_average_outage_probability[state] = (double) outage_UE_number / UE_num;

#if DEBUG_MODE
    /* outage user file output*/
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
    for(int i = 0 ; i < UE_num ; i++){
        total_UE_data_rate += UE_final_data_rate_vector[i];
    }
    recorded_average_data_rate[state] = (double) total_UE_data_rate / UE_num;

    // UE average satisfaction (old)
    double total_ue_old_satis = 0;
    for (int i = 0 ; i < UE_num ; i++){
        double temp = (double) UE_final_data_rate_vector[i] / UE_require_data_rate[i];
        temp = std::min(temp,1.0);
        total_ue_old_satis += temp;
    }
    recorded_average_old_satisfaction[state] = (double) total_ue_old_satis / UE_num;

    // UE average satisfaction (new)
    double total_UE_satis = 0;
    double total_active_UE_satis = 0;
    int cal = 0;
    for(int i = 0 ; i < UE_num ; i++){
        total_UE_satis += UE_final_satisfaction_vector[i];
        if(UE_final_satisfaction_vector[i] != 0 ){
            cal += 1;
            total_active_UE_satis += UE_final_satisfaction_vector[i];
        }
    }

    recorded_average_satisfaction[state] = (double) total_UE_satis / UE_num;
    recorded_average_active_satisfaction[state] = (double) total_active_UE_satis / cal;

    total_ue_satisfaction += recorded_average_satisfaction[state];
    ue_satisfaction = total_ue_satisfaction / (state+1);

    total_active_ue_satisfaction += recorded_average_active_satisfaction[state];
    active_ue_satisfaction = total_active_ue_satisfaction / (state+1);

#if DEBUG_MODE
    /* user satisfaction & active user satisfaction file output */
    std::string path1 = (PROPOSED_METHOD) ? "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/" : "/home/yu/repos/ns-3-allinone/ns-3.25/scratch/thesis/benchmark/";
    std::fstream output1;

    output1.open(path1 + "satisfaction_UE=" + std::to_string(UE_num) + ".csv", std::ios::out | std::ios::app);
    if (!output1.is_open()) {
        std::cout << "Fail to open file\n";
        exit(EXIT_FAILURE);
    }
    else{
        output1 << recorded_average_satisfaction[state] << "," << recorded_average_active_satisfaction[state] << std::endl ;
    }
    output1.close();

#endif // DEBUG_MODE

    std::cout << "one state avg outage: "<<recorded_average_outage_probability[state] << std::endl;
    std::cout << "one state avg satisfaction (old): "<<recorded_average_old_satisfaction[state] << std::endl;
    std::cout << "one state avg satisfaction (new): "<<recorded_average_satisfaction[state] << std::endl;
    std::cout << "one state avg activate satisfaction: " <<recorded_average_active_satisfaction[state] << std::endl;
    std::cout << "one state avg data rate: "<<recorded_average_data_rate[state] << std::endl<<std::endl;

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



    /*
     * AFTER SIMULATION, CALCULATE OVERALL AVERAGE OUTAGE PROBABILITY, AVERAGE DATA RATE //2//
     */

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


    std::cout << "overall avg outage: "<< avg_outage_probability << std::endl;
    std::cout << "overall avg satisfaction: "<<avg_new_satisfaction << std::endl;
    std::cout << "overall avg data rate: "<< avg_data_rate << std::endl<<std::endl;

    // calculate the actual executing time
    struct timespec tmp = diff(start, end);
    double exec_time = tmp.tv_sec + (double) tmp.tv_nsec / 1000000000.0;


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

    /*
     * OUTPUT THE RESULTS TO .CSV FILES
     */

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
        output << avg_outage_probability << "," << avg_old_satisfaction << "," << avg_new_satisfaction << "," << avg_data_rate;
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
