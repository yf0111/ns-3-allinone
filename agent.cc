#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <cmath>

#include "agent.h"

Agent::Agent(std::vector<std::vector<double>> &State_SINR, std::vector<int> &State_AP_load,std::vector<int> &Action)
{
    State_SINR = State_SINR;
    State_AP_load = State_AP_load;
    Action = Action;
}
