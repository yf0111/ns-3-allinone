#ifndef INSTALL_MOBILITY_H
#define INSTALL_MOBILITY_H

using namespace ns3;

/*

2023/01/05 : current no need mobility , but need to check why should install vlc/rf AP mobility
2023/01/09 : because mobility is just like you need to set the location (position) of vlc/rf AP

*/


void installRfApMobility(NodeContainer &RF_AP_nodes);
void installVlcApMobility(NodeContainer &VLC_AP_nodes);
void installUeMobility(NodeContainer &UE_nodes);



#endif // INSTALL_MOBILITY_H
