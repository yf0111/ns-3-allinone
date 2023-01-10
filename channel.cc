#include <cmath>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <chrono>
#include <cmath>
#include <fstream>

#include "channel.h"
#include "print.h"
#include "my_UE_node.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"



static const double lambertian_coefficient = (-1) / (log2(cos(degree2Radian(PHI_half)))); // m
static const double concentrator_gain = pow(refractive_index, 2) / pow(sin(degree2Radian(field_of_view / 2)), 2);


void precalculation(NodeContainer  &RF_AP_node, // !*-*-TODO*-*-! change!

                      NodeContainer  &VLC_AP_nodes,
                      NodeContainer  &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      //* std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, //!*-*-NOTICE*-*-! 2023/01/10 : turn VLC_SINR_matrix 3d to 2d
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<std::vector<double>> &VLC_data_rate_matrix,
                      std::vector<double> &RF_channel_gain_vector,
                      std::vector<double> &RF_SINR_vector,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<MyUeNode> &my_UE_list){
    calculateAllVlcLightOfSight(VLC_AP_nodes, UE_nodes, my_UE_list, VLC_LOS_matrix);
    calculateAllVlcSINR(VLC_LOS_matrix, VLC_SINR_matrix);
    calculateAllVlcDataRate(VLC_SINR_matrix, VLC_data_rate_matrix);
#if DEBUG_MODE
    printVlcLosMatrix(VLC_LOS_matrix);
    printVlcSinrMatrix(VLC_SINR_matrix);
    printVlcDataRateMatrix(VLC_data_rate_matrix);
#endif

    calculateRFChannelGain(RF_AP_node, UE_nodes, my_UE_list, RF_channel_gain_vector);
    calculateAllRFSINR(RF_SINR_vector, RF_channel_gain_vector);
    calculateALLRFDataRate(RF_SINR_vector,RF_data_rate_vector);

#if DEBUG_MODE
    printRFChannelGainVector(RF_channel_gain_vector);
    printRFSINRVector(RF_SINR_vector);
    printRFDataRateVector(RF_data_rate_vector);
#endif
}

    for (int i = 0; i < UE_num; i++)
        my_UE_list[i].randomOrientationAngle(UE_nodes.Get(i));

    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			VLC_LOS_matrix[i][j] = estimateOneVlcLightOfSight(VLC_AP_nodes.Get(i), UE_nodes.Get(j), my_UE_list[j]);
		}
	}
}
/*
    VLC LOS !*-*-NOTICE*-*-! 2023/01/10 : benchmark only LOS
*/
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

    if (radian2Degree(acos(cosine_incidence_angle)) > field_of_view / 2) // incidence angle exceeds half of FoV
        return 0.0;

    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE_node); // the irradiance angle(Φ) of the Tx (in rad)
    const double distance = getDistance(VLC_AP, UE_node);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * concentrator_gain; // g(ψ)
    line_of_sight = line_of_sight * filter_gain; // T_s(ψ)
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient); // cos^θ(φ)
    line_of_sight = line_of_sight * cosine_incidence_angle; // cos(ψ)

    return line_of_sight;
}
// cosψ = 1/d((x_a-x_u)sinθcosω+(y_a-y_u)sinθsinω+(z_a-z_u)cosθ) based on (3)
double getCosineOfIncidenceAngle(Ptr<Node> VLC_AP, Ptr<Node> UE, MyUeNode &UE_node) { // 2023/01/10 : parameter Ptr<Node> UE is necessary?
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

/*
    VLC SINR
*/

void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<std::vector<double>> &VLC_SINR_matrix) { //* , std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix
    // initialize VLC_SINR_matrix
    //* VLC_SINR_matrix = std::vector<std::vector<std::vector<double>>> (VLC_AP_num, std::vector<std::vector<double>> (UE_num, std::vector<double> (subcarrier_num, 0.0)));
    VLC_SINR_matrix = std::vector<std::vector<double>>> (VLC_AP_num, std::vector<std::vector<double>>(UE_num,0.0));

    /*
        2023/01/10 : NO NEED front-end response?
    */
    // pre-calculate front-end of all effective subcarriers

    /*std::vector<double> front_end_vector(effective_subcarrier_num + 1, 0.0);
    for (int i = 1; i < effective_subcarrier_num + 1; i++)
        front_end_vector[i] = estimateOneVlcFrontEnd(i);

    for (int VLC_AP_idx = 0; VLC_AP_idx < VLC_AP_num; VLC_AP_idx++) {
		for (int UE_idx = 0; UE_idx < UE_num; UE_idx++) {
			double interference = 0.0;
            for (int i = 0; i < VLC_AP_num; i++) {
                if (i != VLC_AP_idx)
                    interference += pow(VLC_LOS_matrix[i][UE_idx], 2);
            }

            double lower_than_one_sinr_start_idx = 0.0;
            if (std::pow(VLC_LOS_matrix[VLC_AP_idx][UE_idx], 2) - interference <= 0) {
                lower_than_one_sinr_start_idx = 1;
            }
            else {
                double first_term = subcarrier_num * fitting_coefficient * three_dB_cutoff / (VLC_AP_bandwidth * 2);
                double inner_numerator = std::pow(optical_to_electric_power_ratio, 2) * VLC_noise_power_spectral_density * VLC_AP_bandwidth;
                double inner_denominator = std::pow(conversion_efficiency * VLC_AP_power, 2) * (std::pow(VLC_LOS_matrix[VLC_AP_idx][UE_idx], 2) - interference);
                double second_term = log(inner_numerator / inner_denominator);

                lower_than_one_sinr_start_idx = ceil(-1 * first_term * second_term);
            }

			for (int k = 1; k < (int)lower_than_one_sinr_start_idx; k++) {
                VLC_SINR_matrix[VLC_AP_idx][UE_idx][k] = estimateOneVlcSINR(VLC_LOS_matrix, front_end_vector, VLC_AP_idx, UE_idx, k);
			}
		}
	}*/

	for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
        for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
            VLC_SINR_matrix[VLC_AP_index][UE_index] = estimateOneVlcSINR(VLC_LOS_matrix,VLC_AP_index,UE_index);
        }
	}
}
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix, std::vector<double> &front_end_vector, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double interference = 0;
    for (int i = 0; i < VLC_AP_num; i++) {
        if (i != VLC_AP_index)
            //* interference += pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index] * front_end_vector[subcarrier_index], 2);
            interference += pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index],2);
    }
    //1//double VLC_AP_sub_bandwidth = VLC_AP_bandwidth / VLC_sub_channel // B^VLC_sub = B^VLC / N^VLC = VLC_AP_bandwidth / VLC_sub_channel
    double noise = VLC_AP_bandwidth * VLC_noise_power_spectral_density;
    //* double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index] * front_end_vector[subcarrier_index], 2) / (interference + noise);
    double SINR = pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index],2) / (interference + noise);
    return SINR;
}

/*
    2023/01/09 : My benchmark don't have this , !*-*-NOTICE*-*- : if sub channel == sub carrier then can turn sub carrier into sun channel
*/
// front-end
// H_F(k) = exp( -(k * modulation_bandwidth) / (subcarrier_num * fitting_coefficient * 3dB_cutoff)) based on revised (4)
/*double estimateOneVlcFrontEnd(int subcarrier_index) {
    //* return exp((-1) * subcarrier_index * VLC_AP_bandwidth / (subcarrier_num * fitting_coefficient * three_dB_cutoff));
    return exp((-1) * subcarrier_index * VLC_AP_bandwidth / (VLC_AP_subchannel * fitting_coefficient * three_dB_cutoff)); // *-*-QUESTION*-*-! sub channel = sub carrier?
}*/

/*
    VLC data rate
*/
void calculateAllVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix, std::vector<std::vector<double>> &VLC_data_rate_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
		    /*
                !*-*-NOTICE*-*-! : ρ not take into account yet ! (subchannel)
		    */
            VLC_data_rate_matrix[i][j] = estimateOneVlcDataRate(VLC_SINR_matrix,i,j);
		}
	}
}
double estimateOneVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix , int VLC_AP_index , int UE_index){
    double data_rate = (VLC_AP_bandwidth / VLC_sub_channel) * log2(1 + VLC_SINR_matrix[VLC_AP_index][UE_index]) / 2;
    return data_rate;
}
/*void calculateAllVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix) {
    for (int i = 0; i < VLC_AP_num; i++) {
		for (int j = 0; j < UE_num; j++) {
			for (int k = 1; k < effective_subcarrier_num+1; k++) {
                VLC_data_rate_matrix[i][j][k] = estimateOneVlcDataRate(VLC_SINR_matrix, i, j, k);
			}
		}
	}
}
// data rate of the RU on the certain subcarrier based on Eq. (6) in the benchmark

double estimateOneVlcDataRate(std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix, int VLC_AP_index, int UE_index, int subcarrier_index) {
    double numerator = 2 * VLC_AP_bandwidth * getSpectralEfficiency(VLC_SINR_matrix[VLC_AP_index][UE_index][subcarrier_index]);
    double denominator = subcarrier_num * time_slot_num;
    return numerator / denominator;
}*/



/*
    RF Channel Gain
*/
void calculateRFChannelGain(NodeContainer &RF_AP_node,NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<double> &RF_channel_gain_vector){
	for(int i = 0;i<UE_num;i++){
        RF_channel_gain_vector[i] = estimateOneRFChannelGain(RF_AP_node.Get(i), UE_nodes.Get(j), my_UE_list[j]); //!*-*-NOTICE*-*- : NLOS?
	}
}
double estimateOneRFChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node){

    /*
    //1//
    double distance = getDistance(RF_AP, UE_node);
    double path_loss = 18.7*log10(distance) + 46.8 + 20*log10(carrier_frequency/5);
    double path_loss_power = -(path_loss)/10;
    double rf_los_channel_gain = pow(10,path_loss_power);
    return rf_los_channel_gain;
    */

    //2//
    double distance = getDistance(RF_AP,UE_node);
    std::normal_distribution<double> Gaussian (0.0,10);    //normal distribution 即 Gaussian distribution
    boost::math::rayleigh_distribution<double> rayleigh(0.8); // standard rayleigh distribution
    std::uniform_real_distribution<double> random_p(0.0, 1.0);// uniform random variable between 0.0 and 1.0 for inverse transform sampling
    std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());
    double X=0.0,H=0.0,p;
    for(int i=0;i<100000;i++){
        X+= Gaussian(gen);
        p = random_p(gen);
        H+= quantile(rayleigh,p);
    }
    X/=100000;
    H/=100000;

    double L_d;
    if(distance < breakpoint_distance){
        L_d = 20 * log10(RF_carrier_frequency*distance) - 147.5;
    }
    else{
        L_d = 20 * log10(RF_carrier_frequency*pow(distance,2.75)/pow(breakpoint_distance,1.75)) - 147.5;
    }
    double rf_los_channel_gain = pow(H,2) * pow(10,((-1)*L_d+X)/10.0);
    return rf_los_channel_gain;
}

/*
    RF SINR
*/
void calculateAllRFSINR(std::vector<double> &RF_SINR_vector, std::vector<double> &RF_channel_gain_vector) {
    RF_SINR_vector = std::vector<double> (UE_num,0.0);
	for( int i = 0; i < UE_num ; i++){
        RF_SINR_vector[i] = estimateOneRFSINR(RF_channel_gain_vector,i);
	}
}
double estimateOneRFSINR(std::vector<double> &RF_channel_gain_vector, int UE_index) {
    /*
        !*-*-NOTICE*-*-! : this RF SINR not take M (adjacent industrial factory) and I^RF,U_k,n (interference from competing technologies)into account yet !
    */
    /*
    //1//
    double numerator = RF_AP_power * RF_channel_gain_vector[UE_index];
    double denominator = RF_noise_power_spectral_density*(RF_AP_bandwidth / RF_AP_subchannel);
    double SINR = numerator / denominator;
    return SINR;
    */

    double numerator = RF_AP_power * RF_channel_gain_vector[UE_index];
    double denominator = RF_noise_power_spectral_density * RF_AP_bandwidth;
    double SINR = numerator / denominator;
    return SINR;
}

/*
    RF data rate
*/
void calculateALLRFDataRate(std::vector<double> &RF_SINR_vector,std::vector<double> &RF_data_rate_vector){
    for (int i = 0; i < UE_num; i++) {
        /*
            !*-*-NOTICE*-*-! : ρ not take into account yet ! (subchannel)
        */
        RF_data_rate_vector[i] = estimateOneRFDataRate(RF_SINR_vector,i);
    }
}
double estimateOneRFDataRate(std::vector<double> &RF_SINR_vector , int UE_index){
    double data_rate = (RF_AP_bandwidth / RF_AP_subchannel) * log2(1 + RF_SINR_vector[UE_index]);
    return data_rate;
}
