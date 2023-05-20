#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <cmath>


#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_configuration.h"


using namespace ns3;

MyUeNode::MyUeNode(int node_ID, Vector pos, double required_data_rate)
{
    this->node_ID = node_ID;
    this->pos = pos;
    this->required_data_rate = required_data_rate;

    polar_angle = 0.0;
    azimuth_angle = 0.0;
    prev_associated_AP = -1;
    curr_associated_AP = -1;
}

MyUeNode::MyUeNode(int node_ID, Vector pos, double required_data_rate, int group) //* : generator(std::chrono::system_clock::now().time_since_epoch().count()), distribution(0.0, sqrt(noise_variance))
{
    this->node_ID = node_ID;
    this->pos = pos;
    this->required_data_rate = required_data_rate;
    this->group = group;

    polar_angle = 0.0;
    azimuth_angle = 0.0;
    prev_associated_AP = -1;
    curr_associated_AP = -1;
}



int MyUeNode::getID(void) {
    return node_ID;
}

void MyUeNode::setGroup(int UE_group){
    group = UE_group;
}

int MyUeNode::getGroup(void){
    return group;
}

void MyUeNode::setPosition(Vector pos_from_mobility_model) {
    pos = pos_from_mobility_model;
}

Vector MyUeNode::getPosition(void) {
    return pos;
}

void MyUeNode::setRequiredDataRate(double data_rate_in_Mbps) {
    required_data_rate = data_rate_in_Mbps;
}

double MyUeNode::getRequiredDataRate(void) {
    return required_data_rate;
}

void MyUeNode::setCurrAssociatedAP(int associated_AP_index) {
    prev_associated_AP = curr_associated_AP;
    curr_associated_AP = associated_AP_index;
}

void MyUeNode::changeCurrAssociatedAP(int associated_AP_index) {
    curr_associated_AP = associated_AP_index;
}

int MyUeNode::getCurrAssociatedAP(void) {
    return curr_associated_AP;
}

int MyUeNode::getPrevAssociatedAP(void) {
    return prev_associated_AP;
}

void MyUeNode::addThroughput(double new_data_rate) {
    throughput_per_state.push_back(new_data_rate);
}

double MyUeNode::getLastThroughput(void) {
    if (throughput_per_state.empty()) {
        std::cout << "user " << getID() << "'s throughput vector is empty\n";
        return 0.0;
    }

    return throughput_per_state.back();
}

double MyUeNode::calculateAvgThroughput(void) {
    double throughput_sum = 0.0;

    for (int i = 0; i < throughput_per_state.size(); i++)
        throughput_sum += throughput_per_state[i];

    return throughput_sum / throughput_per_state.size();
}

std::vector<double> MyUeNode::getThroughputHistory(void) {
    return throughput_per_state;
}

void MyUeNode::addSatisfaction(double new_satis) {
    satisfaction_per_state.push_back(new_satis);
}

double MyUeNode::getLastSatisfaction(void) {
    if (satisfaction_per_state.empty()) {
        std::cout << "user " << getID() << "'s satisfaction vector is empty\n";
        return 0.0;
    }

    return satisfaction_per_state.back();
}

double MyUeNode::calculateAvgSatisfaction(void) {
    double satis_sum = 0.0;

    for (int i = 0; i < satisfaction_per_state.size(); i++) {
        satis_sum += satisfaction_per_state[i];
    }

    return satis_sum / satisfaction_per_state.size();
}

std::vector<double> MyUeNode::getSatisfactionHistory(void) {
    return satisfaction_per_state;
}

void MyUeNode::setPolarAngle(double new_polar_angle) {
    polar_angle = new_polar_angle;
}


double MyUeNode::getPolarAngle(void) {
    return polar_angle;
}

void MyUeNode::setAzimuthAngle(double new_azimuth_angle) {
    azimuth_angle = new_azimuth_angle;
}

double MyUeNode::getAzimuthAngle(void) {
    return azimuth_angle;
}

// θ[k] = c_0 + c_1*θ[k-1] + w[k] based on (22)
// ω = Ω - π, where Ω is the angle between movement direction of a user and x-axis (East)
void MyUeNode::randomOrientationAngle(Ptr<Node> UE) {
    double new_polar_angle = c_0 + c_1 * polar_angle + distribution(generator); // in degree
    setPolarAngle(new_polar_angle / 180 * PI);

    Ptr<MobilityModel> UE_mobility_model = UE->GetObject<MobilityModel>();
    Vector UE_velocity = UE_mobility_model->GetVelocity();


    if (UE_velocity.x == 0 && UE_velocity.y == 0) // UE is not moving
        setAzimuthAngle(-1*PI);
    else {
        double big_omega = acos(UE_velocity.x / sqrt(UE_velocity.x * UE_velocity.x + UE_velocity.y * UE_velocity.y));

        // since 0 <= Ω < 2π, however the codomain of acos is only [0,π]
        // so, for those angles originally in 3rd or 4th quadrant we have to recover them
        if (UE_velocity.y < 0.0)
            big_omega = 2 * PI - big_omega;

        setAzimuthAngle(big_omega-PI);
    }
}






