#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

/*

2023/01/05 : //* means liu_code exist
    /ref'1/ means that this func.(var.) is used for reference : Reinforcement Learning-Based Near-Optimal Load Balancing for Heterogeneous LiFi WiFi Network
    /ref'2/ means that this func.(var.) is used for reference : Improving the performance of Heterogeneous LiFi WiFi network using a novel Link Aggregation Framework
*/

#define DEBUG_MODE 0
#define PROPOSED_METHOD 0
#define LASINR 0
#define LAEQOS 0
#define RLLB 1
#define PCSBM 1 // can change


const double PI = boost::math::constants::pi<double>();
const double EE = boost::math::constants::e<double>();
const double room_size = 5; // m // @-ref'1- 5@ , @-ref'2- 5@
const double time_period = 0.5; // sec //
const int state_num = 1; // @-ref'1- ?@ , @-ref'2- 1@


/*  RF AP  */
const int RF_AP_num = 1;
const int RF_AP_height = 3; // m //
const int RF_AP_bandwidth = 20; // MHz //
const double RF_AP_power = 0.1; // W //
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
/*
ref'1 : 1e-21 A^2/Hz = 1e-15 A^2/MHz
ref'2 : -210 dBm/MHz ~= 1e-24 A^2/MHz
*/
const double conversion_efficiency = 0.53; // A/W // optical to electrical conversion efficiency (τ) , PD’s responsivity (μ) ,


/*  UE  */
const int UE_num = 10; // @-ref'1- ?@ , @-ref'2- 10@
const double UE_height = 1; // m // @-ref'1- 1@ , @-ref'2- 0@
const double avg_speed = 1.0; // m/s
const double pause_time = 10.0;
//* const int demand_upper_bound = 100;


/*  VLC channel  */
const double field_of_view = 60.0; // semi degree //
const double PHI_half = 60.0; // semi-angle at half-illumination in degree
const double filter_gain = 1.0;
const double refractive_index = 1.5;
const double receiver_area = 1e-4; // 1 cm^2 = 0.0001 m^2

/*  RF Channel  */
const double RF_carrier_frequency = 2.4e9; // Hz //
const double breakpoint_distance = 0.05; // m // @-ref'1- 0.05@ , @-ref'2- 5@
const double RF_three_db_cutoff = 1;


/*  random orientation angle (UE random orientation)  */
const double coherence_time = 130.0; // ms
const double sampling_time = 13.0; // ms
const double angle_mean = 30.0; // degree
const double angle_variance = 7.78; // degree
const double c_1 = pow(0.05, sampling_time/coherence_time);
const double c_0 = (1.0 - c_1) * angle_mean;
const double noise_variance = (1.0 - c_1 * c_1) * angle_variance * angle_variance;


/*  ref'2 system parameter  */
const double la_overhead = 0.8;
const int LA_UE_num = 4;
const double require_data_rate_threshold = 40; //Mbps


/*  ref'1 system parameter  */
const double reflection_coe = 0.8; // ρ ,walls reflectivity

#endif // GLOBAL_CONFIGURATION_H
