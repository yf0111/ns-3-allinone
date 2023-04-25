#include <cmath>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <chrono>
#include <fstream>
#include <complex>
#include <boost/math/distributions/rayleigh.hpp>

#include "channel.h"
#include "print.h"
#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"



static const double lambertian_coefficient = (-1) / (log2(cos(degree2Radian(PHI_half)))); // m
static const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view / 2)), 2);
double X = 0.0 ,H = 0.0,p , X1 = 0.0; // ref'1 , ref'2  about RF channel

/*  VLC LOS  */
double calculateAllVlcLightOfSight(NodeContainer &VLC_AP_nodes, NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<std::vector<double>> &VLC_LOS_matrix) {

    for (int i = 0; i < UE_num; i++)
        my_UE_list[i].randomOrientationAngle(UE_nodes.Get(i));

    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), UE_nodes.Get(j), my_UE_list[j]);
		}
	}
}
double estimateOneVlcLightOfSight(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node) {
    const double cosine_incidence_angle = getCosineOfIncidenceAngle(VLC_AP, UE, UE_node); // cos(ψ)

    if (radian2Degree(acos(cosine_incidence_angle)) > field_of_view / 2){ // incidence angle exceeds half of FoV
        return 0.0;
    }
    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE_node); // the irradiance angle(Φ) of the Tx (in rad)
    const double distance = getDistance(VLC_AP, UE_node);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient); // cos^θ(φ)
    line_of_sight = line_of_sight * cosine_incidence_angle; // cos(ψ)
    line_of_sight = line_of_sight * concentrator_gain; // gcon(ψ)
    line_of_sight = line_of_sight * filter_gain; // gf
    return line_of_sight;
}

// cosψ = 1/d((x_a-x_u)sinθcosω+(y_a-y_u)sinθsinω+(z_a-z_u)cosθ) based on (3)
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node) {
    double polar_angle = UE_node.getPolarAngle();
    double azimuth_angle = UE_node.getAzimuthAngle();

    Ptr<MobilityModel> VLC_AP_mobility = VLC_AP->GetObject<MobilityModel>();
    Vector AP_pos = VLC_AP_mobility->GetPosition();

    Ptr<MobilityModel> UE_mobility = UE->GetObject<MobilityModel>();
    Vector UE_curr_pos = UE_mobility->GetPosition();
    UE_node.setPosition(UE_curr_pos); // here is the first time in this state to access the position of the user, so have to update for usage of other functions

    double dx = AP_pos.x - UE_curr_pos.x;
    double dy = AP_pos.y - UE_curr_pos.y;
    double dz = AP_pos.z - UE_curr_pos.z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);

    double first_term = dx * sin(polar_angle) * cos(azimuth_angle);
    double second_term = dy * sin(polar_angle) * sin(azimuth_angle);
    double last_term = dz * cos(polar_angle);
    return (first_term + second_term + last_term) / dist;
}
/*
    distance and angle calculation

       plane_dist
    AP----------
    |Φ\        |
    |  \       |
    |   \      |
    |    \     | height_diff
    |     \    |
    |      \   |
    |       \  |
    |        \ |
              UE (PD)

    arctan(plane_dist / height_diff) = Φ (rad)
*/
double getIrradianceAngle(Ptr<Node> AP, MyUeNode &UE_node) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();

    Vector UE_pos = UE_node.getPosition();

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;

    double plane_dist = sqrt(dx*dx + dy*dy);
    double height_diff = AP_pos.z - UE_pos.z;

    return atan(plane_dist / height_diff);
}
double radian2Degree(const double &radian) {
    return radian * 180.0 / PI;
}
double degree2Radian(const double &degree) {
    return degree / 180.0 * PI;
}
double getDistance(Ptr<Node> AP, MyUeNode &UE_node) {
    Ptr<MobilityModel> AP_mobility_model = AP->GetObject<MobilityModel>();
    Vector AP_pos = AP_mobility_model->GetPosition();

    Vector UE_pos = UE_node.getPosition();

    double dx = AP_pos.x - UE_pos.x;
    double dy = AP_pos.y - UE_pos.y;
    double dz = AP_pos.z - UE_pos.z;

    return sqrt(dx*dx + dy*dy + dz*dz);
}


/*  VLC SINR  */
void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,std::vector<std::vector<double>> &VLC_SINR_matrix){
    VLC_SINR_matrix = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double>(UE_num,0.0));

    for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            VLC_SINR_matrix[VLC_AP_index][UE_index] = estimateOneVlcSINR(VLC_LOS_matrix, VLC_AP_index, UE_index);
        }
    }
}
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,int VLC_AP_index,int UE_index) {
    double interference = 0.0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index){
            interference += std::pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index],2);
        }
    }
    double noise = VLC_AP_bandwidth * VLC_noise_power_spectral_density;
    double SINR = std::pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index],2) / (interference + noise);
    return (SINR == 0.0) ? 0.0 : 10*log10(SINR);
    /*
        SINR to dB : (SINR == 0.0) ? 0.0 : 10*log10(SINR);
        dB to SINR : std::pow(10.0,dB/10.0);
    */
}
void updateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<std::vector<int>> &AP_association_matrix,std::vector<std::vector<double>> AP_power_allocation){
    for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            VLC_SINR_matrix[VLC_AP_index][UE_index] = estimateUpdateVlcSINR(VLC_LOS_matrix, VLC_AP_index, UE_index,AP_association_matrix,AP_power_allocation);
        }
    }
}
double estimateUpdateVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,int VLC_AP_index,int UE_index,
                                std::vector<std::vector<int>> &AP_association_matrix,std::vector<std::vector<double>> AP_power_allocation){
    double i = 0;
    for(int check = 0 ; check < UE_num ; check++){
        if(check != UE_index && AP_association_matrix[VLC_AP_index][check] == 1)
            i = 1;
    }

    double interference = 0.0;
    if( i == 1 ){
        for (int AP_index = 0; AP_index < VLC_AP_num; AP_index++) {
            if (AP_index != VLC_AP_index){
                interference += std::pow(conversion_efficiency * (VLC_AP_power * AP_power_allocation[VLC_AP_index+1][UE_index]) * VLC_LOS_matrix[AP_index][UE_index],2);
            }
        }
    }

    double noise = VLC_AP_bandwidth * VLC_noise_power_spectral_density;
    double SINR = std::pow(conversion_efficiency * (VLC_AP_power * AP_power_allocation[VLC_AP_index+1][UE_index]) * VLC_LOS_matrix[VLC_AP_index][UE_index],2) / (interference + noise);
    return SINR;
}


/*  VLC data rate  */
void calculateAllVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<std::vector<double>> &VLC_data_rate_matrix) {
    for(int i = 0 ; i < VLC_AP_num ; i++){
        for(int j = 0 ; j < UE_num ; j++){
            VLC_data_rate_matrix[i][j] = estimateOneVlcDataRate(VLC_SINR_matrix,i,j);
        }
    }
}
double estimateOneVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, int VLC_AP_index , int UE_index){
    if(PROPOSED_METHOD && !LASINR && !LAEQOS ){
        double data_rate = (VLC_AP_bandwidth / 2.0) * log2( 1 + (6.0 / PI * EE) * VLC_SINR_matrix[VLC_AP_index][UE_index]);
        return std::isnan(data_rate)? 0.0 : data_rate;
    }
    else if ((LASINR || LAEQOS) && !PROPOSED_METHOD){
        return 0.0;
    }
    else{
        std::cout<<"**(channel.cc) global configuration about method is WRONG!**\n";
        return 0.0;
    }
}


/*  RF Channel Gain  */
void calculateRFChannelGain(NodeContainer &RF_AP_node,NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<double> &RF_channel_gain_vector){
	for(int i = 0 ; i  < UE_num ; i++)
        RF_channel_gain_vector[i] = estimateOneRFChannelGain(RF_AP_node.Get(0), UE_nodes.Get(i), my_UE_list[i]);
}
double estimateOneRFChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node){
    if(PROPOSED_METHOD && !LASINR && !LAEQOS){
        double distance = getDistance(RF_AP,UE_node);
        std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());
        boost::math::rayleigh_distribution<double> rayleigh(2.46); // !*-*-NOTSURE*-*-!
        std::uniform_real_distribution<double> random_p(0.0, 1.0);
        for(int i = 0; i<100000 ; i++){
                p = random_p(gen);
                H += quantile(rayleigh,p);
        }
        H /= 100000; // hr : the small-scale fading gain, which follows independent identical Rayleigh distribution with 2.46 dB average power

        double L_d = (distance < breakpoint_distance)? 20 * log10(distance * RF_carrier_frequency) - 147.5 + 3 : 20 * log10(distance * RF_carrier_frequency) - 147.5 + 35 * log10(distance / breakpoint_distance) + 3;
        double rf_los_channel_gain = std::pow(10,(-L_d/10.0)) * std::pow(H,2) ;
        return rf_los_channel_gain;
    }
    else if((LASINR || LAEQOS) && !PROPOSED_METHOD){
        double distance = getDistance(RF_AP,UE_node);
        /*
        Gaussian distribution
            Xσ is a Gaussian random variable with zero mean and standard deviation σ
                σ before break_distance : 3 dB
                σ after break_distance : 5 dB
        */
        std::normal_distribution<double> Gaussian_before(0.0,3);  // Xσ (before)
        std::normal_distribution<double> Gaussian_after(0.0,5); // Xσ (after)
        std::normal_distribution<double> Gaussian(0,1); // X1
        std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());

        int i = 0;
        while(i < 100000){
            X1 += Gaussian(gen);
            X += (distance > breakpoint_distance)? Gaussian_after(gen) : Gaussian_before(gen);
            i++ ;
        }
        X /= 100000.0;

        double L_d = (distance > breakpoint_distance)? 20 * log10(distance * RF_carrier_frequency) - 147.5 + 35 * log10(distance / breakpoint_distance) + X : 20 * log10(distance * RF_carrier_frequency) - 147.5 + X;

        double angle_phi_rad = getIrradianceAngle(RF_AP,UE_node);
        double angle_phi_deg = radian2Degree(angle_phi_rad);
        std::complex<double> H_comp;
        if(distance <= breakpoint_distance){
            H_comp.real(sqrt(1.0/2) * (cos(angle_phi_deg) + X1));
            H_comp.imag(sqrt(1.0/2) * sin(angle_phi_deg));
        }

        double rf_los_channel_gain = std::norm(H_comp) * std::pow(10,((-1)*L_d)/10.0);
        return rf_los_channel_gain;
    }
    else{
        std::cout<<"**(channel.cc) global configuration about method is WRONG!**\n";
        return 0.0;
    }
}


/*  RF SINR  */
void calculateAllRFSINR(std::vector<double> &RF_SINR_vector,std::vector<double> &RF_channel_gain_vector){
    for( int j = 0; j < UE_num ; j++){
        RF_SINR_vector[j] = estimateOneRFSINR(RF_channel_gain_vector,j);
    }
}
double estimateOneRFSINR(std::vector<double> &RF_channel_gain_vector,int UE_index) {
    double numerator = RF_AP_power * RF_channel_gain_vector[UE_index];
    double denominator = RF_noise_power_spectral_density * RF_AP_bandwidth;
    double SINR = numerator / denominator;
    return SINR;
}
void updateAllRFSINR(std::vector<double> &RF_SINR_vector,std::vector<double> &RF_channel_gain_vector, std::vector<std::vector<double>> &AP_power_allocation){
    for( int j = 0; j < UE_num ; j++){
        RF_SINR_vector[j] = estimateUpdateRFSINR(RF_channel_gain_vector, j , AP_power_allocation);
    }
}
double estimateUpdateRFSINR( std::vector<double> &RF_channel_gain_vector, int UE_index, std::vector<std::vector<double>> &AP_power_allocation){
    double numerator = (RF_AP_power * AP_power_allocation[0][UE_index]) * RF_channel_gain_vector[UE_index];
    double denominator = RF_noise_power_spectral_density * RF_AP_bandwidth;
    double SINR = numerator / denominator;
    return SINR;
}

/*  RF data rate  */
double estimateOneRFDataRate(std::vector<double> &RF_SINR_vector,int UE_index){
    if(PROPOSED_METHOD && !LASINR && !LAEQOS){
        double data_rate = RF_AP_bandwidth * log2(1 + RF_SINR_vector[UE_index]);
        return std::isnan(data_rate)? 0.0 : data_rate;
    }
    else if ((LASINR || LAEQOS) && !PROPOSED_METHOD){
        return 0.0;
    }
    else{
        std::cout<<"**(channel.cc) global configuration about method is WRONG!**\n";
        return 0.0;
    }
}
void calculateALLRFDataRate(std::vector<double> &RF_data_rate_vector,std::vector<double> &RF_SINR_vector){
    for(int j = 0 ; j < UE_num ; j++){
        RF_data_rate_vector[j] = estimateOneRFDataRate(RF_SINR_vector,j);
    }
}

void precalculation(NodeContainer  &RF_AP_node,
                      NodeContainer  &VLC_AP_nodes,
                      NodeContainer  &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<std::vector<double>> &VLC_data_rate_matrix,
                      std::vector<double> &RF_channel_gain_vector,
                      std::vector<double> &RF_SINR_vector,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<MyUeNode> &my_UE_list)
{
    calculateAllVlcLightOfSight(VLC_AP_nodes, UE_nodes, my_UE_list, VLC_LOS_matrix);
    calculateAllVlcSINR(VLC_LOS_matrix, VLC_SINR_matrix);
    calculateAllVlcDataRate(VLC_SINR_matrix,VLC_data_rate_matrix);

    //printVlcLosMatrix(VLC_LOS_matrix);
    //printVlcSinrMatrix(VLC_SINR_matrix);
    //printVlcDataRateMatrix(VLC_data_rate_matrix);

    calculateRFChannelGain(RF_AP_node, UE_nodes, my_UE_list, RF_channel_gain_vector);
    calculateAllRFSINR(RF_SINR_vector, RF_channel_gain_vector);
    calculateALLRFDataRate(RF_data_rate_vector,RF_SINR_vector);

    //printRFChannelGainVector(RF_channel_gain_vector);
    //printRFSINRVector(RF_SINR_vector);
    //printRFDataRateVector(RF_data_rate_vector);
}
