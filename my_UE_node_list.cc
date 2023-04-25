#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "my_UE_node.h"
#include "my_UE_node_list.h"


/*
 * create an myUeNode object for each UE
 */
std::vector<MyUeNode> initializeMyUeNodeList(NodeContainer &UE_nodes)
{
    std::vector<MyUeNode> my_UE_list;
    std::default_random_engine generator (std::chrono::system_clock::now().time_since_epoch().count());
    std::poisson_distribution<int> distribution_benchmark(70);
    std::uniform_int_distribution<int> uniform_urllc(urllc_dataratea_lower_bound, urllc_dataratea_upper_bound);
    std::uniform_int_distribution<int> uniform_normal(normal_data_rate_lower_bound, normal_data_rate_upper_bound);

    for (int i = 0; i < UE_num; i++) {

        double required_data_rate = 0.0;
        int group = 0;

        if ((LASINR || LAEQOS) && !PROPOSED_METHOD){
            required_data_rate = require_data_rate_threshold;
        }
        else if( PROPOSED_METHOD ){
            if(i < urllc_UE_num){
                required_data_rate = uniform_urllc(generator);
                group = 1; // urllc device
            }
            else{
                required_data_rate = uniform_normal(generator);
                group = 2; // normal device
            }
        }
        else{
            std::cout<<"** (my_UE_node_list.cc) global configuration about method is WRONG!**\n";
        }

        Ptr<MobilityModel> UE_mobility_model = (UE_nodes.Get(i))->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        if(PROPOSED_METHOD){
            my_UE_list.push_back(MyUeNode(i, pos, required_data_rate, group));
        }
        else{
            my_UE_list.push_back(MyUeNode(i, pos, required_data_rate));
        }
    }

    return my_UE_list;
}
