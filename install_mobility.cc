#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <time.h>
#include <random>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "install_mobility.h"
#include "global_configuration.h"



using namespace ns3;

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
    //1//
    install the mobility of VLC AP  : 6x6

                         ^
        *(30) *(31) *(32)| *(33) *(34) *(35)
        *(24) *(25) *(26)| *(27) *(28) *(29)
        *(18) *(19) *(20)| *(21) *(22) *(23)
     ------------------------------------>
        *(12) *(13) *(14)| *(15) *(16) *(17)
        *(6)  *(7)  *(8) | *(9)  *(10) *(11)
        *(0)  *(1)  *(2) | *(3)  *(4)  *(5)

*/
/*
    //2//
    install the mobility of VLC AP  : 2x2

                ^
         *(2)   |   *(3)
     ----------------------->
         *(0)   |   *(1)

*/

void installVlcApMobility(NodeContainer &VLC_AP_nodes) {
    MobilityHelper VLC_AP_mobility;
    Ptr<ListPositionAllocator> VLC_AP_pos_list = CreateObject<ListPositionAllocator>();

    double delta = room_size / VLC_AP_per_row; // 24/6 , 5/2
    for (int i = 0; i < VLC_AP_num; i++) {
        double x = (i%VLC_AP_per_row + 1) * delta;
        double y = (i/VLC_AP_per_row + 1) * delta;

        // change origin from left-down to the center
        x -= room_size / 2 + (room_size/VLC_AP_per_row)/2;
        y -= room_size / 2 + (room_size/VLC_AP_per_row)/2;

        VLC_AP_pos_list->Add(Vector(x, y, VLC_AP_height));
    }


    VLC_AP_mobility.SetPositionAllocator(VLC_AP_pos_list);

    VLC_AP_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    VLC_AP_mobility.Install(VLC_AP_nodes);
}


/*
    install the mobility of UE - orientation-based RWP
    2023/01/09 : benchmark does not mention about device orientation , and UE no need to move

*/
void installUeMobility(NodeContainer &UE_nodes) {
    SeedManager::SetSeed(time(NULL));

    MobilityHelper UE_mobility;

    // set 3D position allocator for random waypoints
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomBoxPositionAllocator");

    std::stringstream ssPos;
    ssPos << "ns3::UniformRandomVariable[Min=" << -room_size / 2 << "|Max=" << room_size / 2 << "]";

    /* ref'1 :
        choose different heights for UE (0.5,1,1.5,2); A number of devices are randomly distributed at four different heights (0.5, 1, 1.5,and 2 m).
    */

    std::stringstream ssPosZ;
    ssPosZ << "ns3::UniformRandomVariable[Min=0.5|Max=2.0]";


    /*double min_ = 0.5;
    double max_ = 2.0;
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
    x->SetAttribute ("Min", DoubleValue (min_));
    x->SetAttribute ("Max", DoubleValue (max_));
    // The values returned by a uniformly distributed random
    // variable should always be within the range
    //
    //     [min, max)  .
    //
    double value = x->GetValue ();
    std::cout<<"value:\t"<<value<<"\n";*/



    // set an attribute to be set during construction
    pos.Set("X", StringValue(ssPos.str()));
    pos.Set("Y", StringValue(ssPos.str()));
    pos.Set("Z", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    /*if(PDSERT){
        pos.Set("Z", StringValue(ssPosZ.str()));
    }else{
        pos.Set("Z", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    }*/


    Ptr<PositionAllocator> position_allocator = (pos.Create())->GetObject<PositionAllocator>();
    UE_mobility.SetPositionAllocator(position_allocator);

/* //*
    2023/01/09 : RWP

    // set mobility model
    // - the random variable for user speed
    std::stringstream ss_speed;
    ss_speed << "ns3::UniformRandomVariable[Min=" << 0 << "|Max=" << avg_speed * 2 << "]";

    // - the random variable for pause time
    std::stringstream ss_pause;
    ss_pause << "ns3::UniformRandomVariable[Min=0.0|Max=" << pause_time << "]";

    UE_mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue(ss_speed.str()),
                                  "Pause", StringValue(ss_pause.str()),
                                  "PositionAllocator", PointerValue(position_allocator));
*/
    UE_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    UE_mobility.Install(UE_nodes);
}





