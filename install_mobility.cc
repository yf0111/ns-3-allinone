#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <time.h>
#include <random>
#include <float.h>
#include <stdio.h>
#include <iomanip>
#include <math.h>
#include <chrono>
#include <complex>
#include <boost/math/distributions/rayleigh.hpp>


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "install_mobility.h"
#include "global_configuration.h"



using namespace ns3;

std::random_device rd;  // Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
std::uniform_int_distribution<> distrib(0, 3);

/*
    install the mobility of RF AP

    Since there is only one RF AP, it is placed at the center of the room
*/
void installRfApMobility(NodeContainer &RF_AP_node) {
    MobilityHelper RF_AP_mobility;
    Ptr<ListPositionAllocator> RF_AP_pos_list = CreateObject<ListPositionAllocator>();

    // assume that the center of the room is at origin
    RF_AP_pos_list->Add(Vector(0, 0, RF_AP_height));

    RF_AP_mobility.SetPositionAllocator(RF_AP_pos_list);

    RF_AP_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    RF_AP_mobility.Install(RF_AP_node);
}


/*
    install the mobility of VLC AP  : 2x2

                ^
         *(2)   |   *(3)
     ----------------------->
         *(0)   |   *(1)

*/

void installVlcApMobility(NodeContainer &VLC_AP_nodes) {
    MobilityHelper VLC_AP_mobility;
    Ptr<ListPositionAllocator> VLC_AP_pos_list = CreateObject<ListPositionAllocator>();

    double set_room_size = (SUPER_DYNAMIC)? 5.0:room_size;
    double delta = set_room_size / VLC_AP_per_row; // 5/2
    for (int i = 0; i < VLC_AP_num; i++) {
        double x = (i%VLC_AP_per_row + 1) * delta;
        double y = (i/VLC_AP_per_row + 1) * delta;
        // change origin from left-down to the center
        x -= set_room_size / 2 + (set_room_size/VLC_AP_per_row)/2;
        y -= set_room_size / 2 + (set_room_size/VLC_AP_per_row)/2;
        VLC_AP_pos_list->Add(Vector(x, y, VLC_AP_height));
    }

    VLC_AP_mobility.SetPositionAllocator(VLC_AP_pos_list);

    VLC_AP_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    VLC_AP_mobility.Install(VLC_AP_nodes);
}


/*
    install the mobility of UE - orientation-based RWP

*/
void installUeMobility(NodeContainer &UE_nodes) {
    SeedManager::SetSeed(time(NULL));

    MobilityHelper UE_mobility_static;
    MobilityHelper UE_mobility_dynamic;

    // set 3D position allocator for random waypoints
    ObjectFactory pos_static;
    ObjectFactory pos_dynamic;
    pos_static.SetTypeId("ns3::RandomBoxPositionAllocator");
    pos_dynamic.SetTypeId("ns3::RandomBoxPositionAllocator");

    std::stringstream ssPos_static;
    std::stringstream ssPos_dynamic;
    ssPos_static << "ns3::UniformRandomVariable[Min=" << -5.0 / 2 << "|Max=" << 5.0 / 2 << "]";
    ssPos_dynamic << "ns3::UniformRandomVariable[Min=" << -room_size / 2 << "|Max=" << room_size / 2 << "]";

    // set an attribute to be set during construction - static
    pos_static.Set("X",StringValue(ssPos_static.str()));
    pos_static.Set("Y",StringValue(ssPos_static.str()));
    pos_static.Set("Z",StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    Ptr<PositionAllocator> position_allocator_static = (pos_static.Create())->GetObject<PositionAllocator>();
    UE_mobility_static.SetPositionAllocator(position_allocator_static);

    // set an attribute to be set during construction - dynamic
    pos_dynamic.Set("X", StringValue(ssPos_dynamic.str()));
    pos_dynamic.Set("Y", StringValue(ssPos_dynamic.str()));
    pos_dynamic.Set("Z", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    Ptr<PositionAllocator> position_allocator_dynamic = (pos_dynamic.Create())->GetObject<PositionAllocator>();
    UE_mobility_dynamic.SetPositionAllocator(position_allocator_dynamic);


    /* static environment */
    UE_mobility_static.SetMobilityModel("ns3::ConstantPositionMobilityModel");


    /* dynamic environment */
    // - the random variable for user speed
    std::stringstream ss_speed;
    ss_speed << "ns3::UniformRandomVariable[Min=" << 0 << "|Max=" << avg_speed * 2 << "]";

    // - the random variable for pause time
    std::stringstream ss_pause;
    ss_pause << "ns3::UniformRandomVariable[Min=0.0|Max=" << pause_time << "]";

    UE_mobility_dynamic.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                              "Speed", StringValue(ss_speed.str()),
                              "Pause", StringValue(ss_pause.str()),
                              "PositionAllocator", PointerValue(position_allocator_dynamic));


    if(PROPOSED_METHOD){
        int i = 0;
        for (NodeContainer::Iterator it = UE_nodes.Begin(); it != UE_nodes.End(); ++it) {
            if(i < urllc_UE_num){
                UE_mobility_static.Install((*it));
            }
            else{
                UE_mobility_dynamic.Install((*it));
            }
            i++;
        }
    }
    else{
        UE_mobility_static.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        UE_mobility_static.Install(UE_nodes);
    }
}



