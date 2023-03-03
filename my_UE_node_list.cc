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
    std::poisson_distribution<int> distribution(70);
    //* std::uniform_int_distribution<int> uniform(1, demand_upper_bound);

    for (int i = 0; i < UE_num; i++) {
        double required_data_rate = 0.0;
        if(RLLB && !LASINR && !LAEQOS){
            required_data_rate = distribution(generator);
        }
        else if ((LASINR || LAEQOS) && !RLLB){
            required_data_rate = require_data_rate_threshold;
        }
        else{
            std::cout<<"**global configuration about method is WRONG!**\n";
        }
        //* double required_data_rate = uniform(generator);

        Ptr<MobilityModel> UE_mobility_model = (UE_nodes.Get(i))->GetObject<MobilityModel>();
        Vector pos = UE_mobility_model->GetPosition();

        my_UE_list.push_back(MyUeNode(i, pos, required_data_rate));
    }

    return my_UE_list;
}
