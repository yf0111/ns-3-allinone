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
double X = 0.0 , H = 0.0 ,p; // Xσ in RF channel , ref'2

/*
    VLC LOS
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

    if (radian2Degree(acos(cosine_incidence_angle)) > field_of_view / 2){ // incidence angle exceeds half of FoV
        return 0.0;
    }
    const double irradiance_angle = getIrradianceAngle(VLC_AP, UE_node); // the irradiance angle(Φ) of the Tx (in rad)
    const double distance = getDistance(VLC_AP, UE_node);

    double line_of_sight = (lambertian_coefficient+1) * receiver_area / (2 * PI * pow(distance, 2));
    line_of_sight = line_of_sight * pow(cos(irradiance_angle), lambertian_coefficient); // cos^θ(φ)
    line_of_sight = line_of_sight * concentrator_gain; // g(ψ)
    line_of_sight = line_of_sight * filter_gain; // T_s(ψ)
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

void calculateAllVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,
                         std::vector<std::vector<double>> &VLC_SINR_matrix,
                         std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
                         std::vector<std::vector<std::vector<double>>> &VLC_allocated_power_3d){
    std::vector<double> front_end_vector( effective_VLC_subchannel + 1, 0.0);
    if(PDSERT && !LAEQOS && !LASINR){

        // pre-calculate front-end of all effective subcarriers

        VLC_SINR_matrix_3d = std::vector<std::vector<std::vector<double>>> (VLC_AP_num, std::vector<std::vector<double>> (VLC_AP_subchannel, std::vector<double> (UE_num, 0.0)));
        for (int i = 1; i < effective_VLC_subchannel + 1; i++){
            front_end_vector[i] = estimateOneVlcFrontEnd(i);
        }

        for (int VLC_AP_index = 0; VLC_AP_index < VLC_AP_num; VLC_AP_index++) {
            for (int UE_index = 0; UE_index < UE_num; UE_index++) {
                double interference = 0.0;
                for (int i = 0; i < VLC_AP_num; i++) {
                    if (i != VLC_AP_index)
                        interference += pow(VLC_LOS_matrix[i][UE_index], 2);
                }
                double lower_than_one_sinr_start_idx = 0.0;
                if (std::pow(VLC_LOS_matrix[VLC_AP_index][UE_index], 2) - interference <= 0) {
                    lower_than_one_sinr_start_idx = 1;
                }
                else {
                    double first_term = VLC_AP_subchannel * fitting_coefficient * VLC_three_dB_cutoff / (VLC_AP_bandwidth * 2);
                    double inner_numerator = VLC_noise_power_spectral_density * VLC_AP_bandwidth;
                    double inner_denominator = std::pow(conversion_efficiency * 2, 2) * (std::pow(VLC_LOS_matrix[VLC_AP_index][UE_index], 2) - interference);
                    double second_term = log(inner_numerator / inner_denominator);
                    lower_than_one_sinr_start_idx = ceil(-1 * first_term * second_term);
                }
                /*
                    !*-*-NOTICE*-*-! 2023/02/06 : now lower_than_one_sinr_start_idx value is about 10~15
                */
                for (int k = 1; k < (int)lower_than_one_sinr_start_idx; k++) {
                    VLC_SINR_matrix_3d[VLC_AP_index][k][UE_index] = estimateOneVlcSINR(VLC_LOS_matrix, VLC_AP_index, UE_index, front_end_vector, k , VLC_allocated_power_3d);
                }
            }
        }
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        // initialize VLC_SINR_matrix
        VLC_SINR_matrix = std::vector<std::vector<double>> (VLC_AP_num, std::vector<double>(UE_num,0.0));
        for(int VLC_AP_index = 0 ; VLC_AP_index < VLC_AP_num ; VLC_AP_index++){
            for(int UE_index = 0 ; UE_index < UE_num ; UE_index++){
                VLC_SINR_matrix[VLC_AP_index][UE_index] = estimateOneVlcSINR(VLC_LOS_matrix, VLC_AP_index, UE_index, front_end_vector, 1,VLC_allocated_power_3d); // front_end_vector and k is useless in this scpoe !
            }
        }
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
    }
}
double estimateOneVlcSINR(std::vector<std::vector<double>> &VLC_LOS_matrix,
                          int VLC_AP_index,
                          int UE_index,
                          std::vector<double> &front_end_vector,
                          int subchannel_index,
                          std::vector<std::vector<std::vector<double>>> &VLC_allocated_power_3d) {
    if(PDSERT && !LAEQOS && !LASINR){
        double interference = 0.0;
        for(int i = 0 ; i < VLC_AP_num ; i++){
            if( i != VLC_AP_index){
                interference += std::pow(conversion_efficiency * (VLC_LOS_matrix[VLC_AP_index][UE_index]* 250.0 / 36.0) * VLC_LOS_matrix[i][UE_index] * front_end_vector[subchannel_index], 2);
            }
        }
        double VLC_AP_sub_bandwidth = (double)VLC_AP_bandwidth / VLC_AP_subchannel; // B^VLC_sub = B^VLC / N^VLC = VLC_AP_bandwidth / VLC_sub_channel
        double noise = VLC_AP_sub_bandwidth * VLC_noise_power_spectral_density;

        // the power needs to be allocated by algorithm : total : 250 w
        double SINR = std::pow(conversion_efficiency,2) * VLC_allocated_power_3d[VLC_AP_index][subchannel_index][UE_index] * std::pow(VLC_LOS_matrix[VLC_AP_index][UE_index] * front_end_vector[subchannel_index],2) / (double)(interference + noise);
        return SINR; // A^2
    }
    else if ((LAEQOS || LASINR) && !PDSERT){
        double interference = 0.0;
        for (int i = 0; i < VLC_AP_num; i++) {
            if (i != VLC_AP_index){
                interference += std::pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[i][UE_index],2);
            }
        }
        double noise = VLC_AP_bandwidth * VLC_noise_power_spectral_density;
        double SINR = std::pow(conversion_efficiency * VLC_AP_power * VLC_LOS_matrix[VLC_AP_index][UE_index],2) / (interference + noise);
        /*  change to logarithmic SINR
            !*-*-QUESTION*-*-! 2023/02/07 : is because A^2 need to convert to dBm ?
        */
        return (SINR == 0.0) ? 0.0 : 10*log10(SINR);
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }
}
double estimateOneVlcFrontEnd(int subchannel_index) {
    // H_F(k) = exp( -(k * modulation_bandwidth) / (subcarrier_num * fitting_coefficient * 3dB_cutoff)) based on liu's ref's ref (4)
    return exp((-1) * subchannel_index * (double)VLC_AP_bandwidth / (VLC_AP_subchannel * fitting_coefficient * VLC_three_dB_cutoff));
}

/*
    VLC data rate
    !*-*-NOTICE*-*-! Data rate is calculated assuming that APS is done.
*/
void calculateAllVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix,std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d, std::vector<std::vector<double>> &VLC_data_rate_matrix,std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix_3d) {
    if(PDSERT && !LASINR && !LAEQOS){
        for (int i = 0; i < VLC_AP_num; i++) {
            for(int k = 1 ; k < effective_VLC_subchannel +1  ; k ++){
                for(int j = 0 ; j< UE_num ; j++){
                    VLC_data_rate_matrix_3d[i][k][j] = estimateOneVlcDataRate(VLC_SINR_matrix, VLC_SINR_matrix_3d, i, j, k);
                }
            }
        }
    }
    else if ((LASINR || LAEQOS) && !PDSERT){
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
    }
}
double estimateOneVlcDataRate(std::vector<std::vector<double>> &VLC_SINR_matrix,std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d , int VLC_AP_index , int UE_index,int subchannel_index){
    if(PDSERT && !LASINR && !LAEQOS){
        double first = ((double)VLC_AP_bandwidth / VLC_AP_subchannel) / 2.0;
        double second = log2(1 + VLC_SINR_matrix_3d[VLC_AP_index][subchannel_index][UE_index]);
        double data_rate = first * second;
        return data_rate;
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        return 0.0;
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }

    return 0.0;
}

/*
    RF Channel Gain
*/
void calculateRFChannelGain(NodeContainer &RF_AP_node,NodeContainer &UE_nodes,std::vector<MyUeNode> &my_UE_list, std::vector<double> &RF_channel_gain_vector,std::vector<double> &RF_ICI_channel_gain_vector){
	for(int i = 0;i<UE_num;i++){
        RF_channel_gain_vector[i] = estimateOneRFChannelGain(RF_AP_node.Get(0), UE_nodes.Get(i), my_UE_list[i]);
        RF_ICI_channel_gain_vector[i] = estimateOneRFICIChannelGain(RF_AP_node.Get(0) , UE_nodes.Get(i),my_UE_list[i]);
	}
}
double estimateOneRFChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node){
    if(PDSERT && !LASINR && !LAEQOS){
        double distance = getDistance(RF_AP, UE_node);
        double los = (18.7*log10(distance)) + 46.8 + (20*log10(RF_carrier_frequency/5.0));
        double path_loss_power = -( los )/10.0;
        //double path_loss_power = -(los + nlos)/10.0;
        double rf_los_channel_gain = pow(10,path_loss_power);
        return rf_los_channel_gain;
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        double distance = getDistance(RF_AP,UE_node);
        /*
        Gaussian distribution
            Xσ is a Gaussian random variable with zero mean and standard deviation σ
                σ before break_distance : 3 dB
                σ after break_distance : 5 dB
        */
        std::normal_distribution<double> Gaussian_before(0.0,3);  // Xσ (before)
        std::normal_distribution<double> Gaussian_after(0.0,5); // Xσ (after)
        std::default_random_engine gen(std::chrono::system_clock::now().time_since_epoch().count());
        /*
            !*-*-NOTICE*-*-! 2023/02/07 : rayleigh(σ) ,
            σ will affect RF SINR
            https://www.boost.org/doc/libs/1_37_0/libs/math/doc/sf_and_dist/html/math_toolkit/dist/dist_ref/dists/rayleigh.html
                             2023/02/09 : 0.29
        */
        boost::math::rayleigh_distribution<double> rayleigh(0.29);
        std::uniform_real_distribution<double> random_p(0.0, 1.0);
        if(distance <= breakpoint_distance){
            for(int i = 0; i<100000 ; i++){
                X += Gaussian_before(gen);
                p = random_p(gen);
                H += quantile(rayleigh,p);
            }
        }
        if(distance > breakpoint_distance){
            for(int i = 0; i<100000 ; i++){
                X += Gaussian_after(gen);
                p = random_p(gen);
                H += quantile(rayleigh,p);
            }
        }
        X /= 100000.0;
        H /= 100000.0;

        double L_d;

        if(distance <= breakpoint_distance){
            L_d = 20 * log10(distance * RF_carrier_frequency) - 147.5 + X;
        }
        else{
            L_d = 20 * log10(distance * RF_carrier_frequency) - 147.5 + 35 * log10((double)distance / breakpoint_distance) + X;
        }
        double rf_los_channel_gain = std::pow(H,2) * std::pow(10,((-1)*L_d)/10.0);
        return rf_los_channel_gain;
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }
}
double estimateOneRFICIChannelGain(Ptr<Node> RF_AP, Ptr<Node> UE, MyUeNode &UE_node){
    if(PDSERT && !LASINR && !LAEQOS){
        double distance = getDistance(RF_AP, UE_node);
        double nlos = (36.8*log10(distance)) + 43.8 + (20*log10(RF_carrier_frequency/5.0)) + (5 * (wall_num -1));
        double path_loss_power = -( nlos )/10.0;
        double rf_ICI_los_channel_gain = pow(10,path_loss_power);
        return rf_ICI_los_channel_gain;
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        return 0.0;
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }
}

/*
    RF SINR
*/
void calculateAllRFSINR(std::vector<double> &RF_SINR_vector,
                        std::vector<double> &RF_channel_gain_vector,
                        std::vector<std::vector<double>> &RF_SINR_vector_2d,
                        std::vector<std::vector<double>> &RF_allocated_power_2d,
                        std::vector<double> &RF_ICI_channel_gain_vector){
    std::vector<double> front_end_vector( effective_RF_subchannel + 1, 0.0);
    if(PDSERT && !LAEQOS && !LASINR){
        // pre-calculate front-end of all effective subcarriers
        RF_SINR_vector_2d = std::vector<std::vector<double>> (RF_AP_subchannel, std::vector<double> (UE_num, 0.0));
        for (int i = 1; i < effective_RF_subchannel + 1; i++){
            front_end_vector[i] = estimateOneRFFrontEnd(i);
        }

        for (int UE_index = 0; UE_index < UE_num; UE_index++) {
            double interference = RF_ICI_channel_gain_vector[UE_index];
            double lower_than_one_sinr_start_idx = 0.0;
            if (std::pow(RF_channel_gain_vector[UE_index], 2) - interference <= 0) {
                lower_than_one_sinr_start_idx = 1;
            }
            else {
                double first_term = RF_AP_subchannel * fitting_coefficient * RF_three_db_cutoff / (RF_AP_bandwidth * 2);
                double inner_numerator = RF_noise_power_spectral_density * RF_AP_bandwidth;
                double inner_denominator = std::pow(conversion_efficiency * 2, 2) * (std::pow(RF_channel_gain_vector[UE_index], 2) - interference);
                double second_term = log(inner_numerator / inner_denominator);
                lower_than_one_sinr_start_idx = ceil(-1 * first_term * second_term);
                /*
                    !*-*-NOTICE*-*-! : 2023/02/19 value is all 1
                */
            }
            for (int k = 1; k < (int)lower_than_one_sinr_start_idx; k++) {
                RF_SINR_vector_2d[k][UE_index] = estimateOneRFSINR(RF_channel_gain_vector, UE_index, front_end_vector, k , RF_allocated_power_2d);
            }
        }
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        RF_SINR_vector = std::vector<double> (UE_num,0.0);
        for( int i = 0; i < UE_num ; i++){
            RF_SINR_vector[i] = estimateOneRFSINR(RF_channel_gain_vector,i,front_end_vector,1,RF_allocated_power_2d);
        }
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
    }
}
double estimateOneRFSINR(std::vector<double> &RF_channel_gain_vector,
                         int UE_index ,
                         std::vector<double> &front_end_vector,
                         int subchannel_index,
                         std::vector<std::vector<double>> &RF_allocated_power_2d) {
    /*
        !*-*-NOTICE*-*-! : this RF SINR not take M (adjacent industrial factory) and I^RF,U_k,n (interference from competing technologies) into account yet !
    */

    if(PDSERT && !LASINR && !LAEQOS){
        double numerator = RF_allocated_power_2d[UE_index][subchannel_index] * RF_channel_gain_vector[UE_index];
        double denominator = RF_noise_power_spectral_density*((double)RF_AP_bandwidth / RF_AP_subchannel);
        double SINR = numerator / denominator;
        return SINR;
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        double numerator = RF_AP_power * RF_channel_gain_vector[UE_index];
        double denominator = RF_noise_power_spectral_density * RF_AP_bandwidth;
        double SINR = numerator / denominator;
        return SINR;
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }

}
double estimateOneRFFrontEnd(int subchannel_index){
    return exp((-1) * subchannel_index * (double)RF_AP_bandwidth / (RF_AP_subchannel * fitting_coefficient * RF_three_db_cutoff));
}

/*
    RF data rate
    !*-*-NOTICE*-*-! 2023/02/06 : Data rate can only be calculated after APS
*/
double estimateOneRFDataRate(std::vector<double> &RF_SINR_vector , int UE_index){
    /*if(PDSERT && !LASINR && !LAEQOS){
        double data_rate = (RF_AP_bandwidth / RF_AP_subchannel) * log2(1 + RF_SINR_vector[UE_index]);
        return data_rate;
    }
    else if((LASINR || LAEQOS) && !PDSERT){
        return 0.0;
    }
    else{
        std::cout<<"**global configuration about method is WRONG!**\n";
        return 0.0;
    }*/

    return 0.0;
}
void calculateALLRFDataRate(std::vector<double> &RF_SINR_vector,std::vector<double> &RF_data_rate_vector){
    /*
    !*-*-NOTICE*-*-! : ρ not take into account yet ! (subchannel)

    for (int i = 0; i < UE_num; i++) {
        RF_data_rate_vector[i] = estimateOneRFDataRate(RF_SINR_vector,i);
    }*/
}



void precalculation(NodeContainer  &RF_AP_node,
                      NodeContainer  &VLC_AP_nodes,
                      NodeContainer  &UE_nodes,
                      std::vector<std::vector<double>> &VLC_LOS_matrix,
                      std::vector<std::vector<double>> &VLC_SINR_matrix,
                      std::vector<std::vector<std::vector<double>>> &VLC_SINR_matrix_3d,
                      std::vector<std::vector<double>> &VLC_data_rate_matrix,
                      std::vector<std::vector<std::vector<double>>> &VLC_data_rate_matrix_3d,
                      std::vector<std::vector<std::vector<double>>> &VLC_allocated_power_3d,
                      std::vector<double> &RF_channel_gain_vector,
                      std::vector<double> &RF_SINR_vector,
                      std::vector<double> &RF_data_rate_vector,
                      std::vector<std::vector<double>> &RF_SINR_vector_2d,
                      std::vector<std::vector<double>> &RF_data_rate_vector_2d,
                      std::vector<std::vector<double>> &RF_allocated_power_2d,
                      std::vector<double> &RF_ICI_channel_gain_vector,
                      std::vector<MyUeNode> &my_UE_list)
{
    calculateAllVlcLightOfSight(VLC_AP_nodes, UE_nodes, my_UE_list, VLC_LOS_matrix);
    calculateAllVlcSINR(VLC_LOS_matrix, VLC_SINR_matrix, VLC_SINR_matrix_3d,VLC_allocated_power_3d);
    calculateAllVlcDataRate(VLC_SINR_matrix,VLC_SINR_matrix_3d,VLC_data_rate_matrix,VLC_data_rate_matrix_3d);

    //printVlcLosMatrix(VLC_LOS_matrix);
    //printVlcSinrMatrix3d(VLC_data_rate_matrix_3d);
    //printVlcDataRateMatrix3d(VLC_data_rate_matrix_3d);

    calculateRFChannelGain(RF_AP_node, UE_nodes, my_UE_list, RF_channel_gain_vector,RF_ICI_channel_gain_vector);
    calculateAllRFSINR(RF_SINR_vector, RF_channel_gain_vector,RF_SINR_vector_2d,RF_allocated_power_2d,RF_ICI_channel_gain_vector);

    //printRFChannelGainVector(RF_channel_gain_vector);

    //*printRFSINRVector2d(RF_SINR_vector_2d);

}
