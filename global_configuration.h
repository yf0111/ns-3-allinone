#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

/*

2023/01/05 : //* means liu_code exist
    /ref'1/ means that this func.(var.) is used for reference : Reinforcement Learning-Based Near-Optimal Load Balancing for Heterogeneous LiFi WiFi Network (also proposed)
    /ref'2/ means that this func.(var.) is used for reference : Improving the performance of Heterogeneous LiFi WiFi network using a novel Link Aggregation Framework
*/

#define DEBUG_MODE 0
#define PROPOSED_METHOD 0
#define LASINR 0
#define LAEQOS 1
#define SUPER_DYNAMIC 0

const double PI = boost::math::constants::pi<double>();
const double EE = boost::math::constants::e<double>();
const double room_size = 5; // m //
const double time_period = 0.5; // sec //
const int state_num = 1; // [ static : 1 ], [ dynamic : 100 ] //


/*  RF AP  */
const int RF_AP_num = 1;
const int RF_AP_height = 3; // m //
const int RF_AP_bandwidth = 20; // MHz //
const double RF_AP_power = 1; // W // origin:0.1
const double RF_noise_power_spectral_density = 3.98e-15 ; // A^2/MHz // @-ref'1- 3.98e-15@ , @-ref'2- 3.16e-11@

/*
dBm -> A^2

https://www.convertworld.com/zh-hant/power/dbm.html
dBm -> W = A^2 * Ω
( Ω = 1 )

ref'1 : -174 dBm/Hz = 3.98e-21 W/Hz = 3.98e-15 A^2/MHz
ref'2 : -75 dBm/MHz ~= 3.16e-11 A^2/MHz

*/


/*  VLC AP  */
const int VLC_AP_num = 4;
const int VLC_AP_per_row = 2;
const double VLC_AP_height = 3; // m //
const double VLC_AP_power = 3; // W //
const int VLC_AP_bandwidth = 40; // MHz //
const double VLC_noise_power_spectral_density = 1e-15; // A^2/MHz // @-ref'1- 1e-15@ , @-ref'2- 1e-24@
const double conversion_efficiency = 0.53; // A/W // optical to electrical conversion efficiency (τ) , PD’s responsivity (μ)


/*  UE  */
const int UE_num = 10;
const double UE_height = 1; // m //
const double avg_speed = 0.5; // m/s //
const double pause_time = 10.0; // s //


/*  VLC channel  */
const double field_of_view = 60.0; // semi degree //
const double PHI_half = 60.0; // semi-angle at half-illumination in degree
const double filter_gain = 1.0;
const double refractive_index = 1.5;
const double receiver_area = 1e-4; // 1 cm^2 = 0.0001 m^2 //

/*  RF Channel  */
const double RF_carrier_frequency = 2.4e9; // Hz //
const double breakpoint_distance = 0.05; // m //

/*  random orientation angle (UE random orientation)  */
const double coherence_time = 130.0; // ms //
const double sampling_time = 13.0; // ms //
const double angle_mean = 30.0; // degree //
const double angle_variance = 7.78; // degree //
const double c_1 = pow(0.05, sampling_time/coherence_time);
const double c_0 = (1.0 - c_1) * angle_mean;
const double noise_variance = (1.0 - c_1 * c_1) * angle_variance * angle_variance;


/*  ref'2 system parameter  */
const double la_overhead = 0.8;
const int LA_UE_num = UE_num;
const double require_data_rate_threshold = 25; // Mbps //


/*  ref'1 system parameter (useless) */
const double C_one = 100;
const double C_two = 1000;


/* proposed system parameter */
const double SINR_threshold = 5; // dB //
const int urllc_UE_num = UE_num / 2;
const double packet_size = 2000; // 250 byte = 2000 bits //
const double T_max = 1; // ms //
const int urllc_dataratea_lower_bound = 1; // Mbps //
const int urllc_dataratea_upper_bound = 20; // Mbps //
const int normal_data_rate_lower_bound = 3; // Mbps //
const int normal_data_rate_upper_bound = 100; // Mbps //
const double speed_threshold = 0.5; // m/s //
const int wifi_threshold = UE_num / 2 ;
const double eta_hho = 0.9; // from ref1
const double eta_vho = 0.6; // from ref1
#endif // GLOBAL_CONFIGURATION_H
