#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

/*

2023/01/05 : //* means liu_code exist
2023/01/10 :
    //1// means that this func.(var.) is used for reference : Learning-Based Energy-Efficient Resource Management by Heterogeneous RF/VLC for Ultra-Reliable Low-Latency Industrial IoT Networks
    //2// means that this func.(var.) is used for reference : Improving the performance of Heterogeneous LiFi-WiFi network using a novel Link Aggregation Framework
*/

#define DEBUG_MODE 1
#define PROPOSED_METHOD 0
#define PCSBM 1 // can change


const double PI = boost::math::constants::pi<double>();
const double room_size = 5;
const double time_period = 0.5; // sec
const int state_num = 1;


/*
    RF AP
*/
const int RF_AP_num = 1;
const int RF_AP_height = 3; // m
const int RF_AP_bandwidth = 20; //MHz
const int RF_AP_subchannel = 32; // number of sub channel
//1//const double RF_AP_power = 6.7; //W !*-*-TEMP*-*-!
const double RF_AP_power = 0.1; // W
//1//const double RF_noise_power_spectral_density = 173e-6; //N^RF_0 = 173 dBm/Hz = 173e-6 dBm/MHz
const double RF_noise_power_spectral_density =  3.16e-11; // -75 dBm/MHz ~= 3.16e-11 A^2/MHz

/*
dBm -> A^2

https://www.convertworld.com/zh-hant/power/dbm.html
dBm -> W = A^2 * Ω
( Ω = 1 )
*/


/*
    VLC AP
*/
const int VLC_AP_num = 4;
const int VLC_AP_per_row = 2;
const int VLC_AP_height = 3;
const int VLC_AP_power = 3; //*-*-TEMP*-*-! transmitted optical power of a LiFi AP(P_tx , unit:W) , indicates the allocated transmit electrical power on the nth subchannel of the cth VLC AP (P^VLC_n,c , ?)
const int VLC_AP_bandwidth = 40; // MHz
const int VLC_AP_subchannel = 16; // *-*-QUESTION*-*-! sub channel = sub carrier ?
//1//const double VLC_noise_power_spectral_density = 1e-15;  //N^VLC_0 = 1e-21 A^2/Hz = 1e-15 A^2/MHz
const double VLC_noise_power_spectral_density = 1e-24; // -210 dBm/MHz ~= 1e-24 A^2/MHz
const double conversion_efficiency = 0.53; // A/W  optical to electrical conversion efficiency (τ) , PD’s responsivity (μ) ,

// these values are found in "Resource Allocation in LiFi OFDMA Systems"
//* const int subcarrier_num = 40; // M = 64
//* const int effective_subcarrier_num = subcarrier_num / 2 - 1; // M_e = M/2 - 1
//* const int time_slot_num = 32; // K = 20


/*
    UE
    !*-*-NOTICE*-*-! 20230113 : demand_upper_bound NEED change
*/
const int UE_num = 10;
const int demand_upper_bound = 100;
const double UE_height = 0;
//* const double avg_speed = 1.0; // m/s
//* const double pause_time = 0.0;

/*
    VLC channel
*/
const double field_of_view = 90.0; // degree
const double PHI_half = 60.0; // semi-angle at half-illumination in degree
const double filter_gain = 1.0;
const double refractive_index = 1.5;
const double receiver_area = 1e-4; // 1 cm^2 = 0.0001 m^2
//* const double reflection_efficiency = 0.75;

/*
    RF Channel
*/
const double RF_carrier_frequency = 2.4; // GHz !*-*-NOICE*-*-! : USE where?
const int breakpoint_distance = 5; //m

/*
    2023/01/09 : benchmark don't have this !
*/
//* const double fitting_coefficient = 2.88;
//* const double three_dB_cutoff = 30; // MHz

/*
    random orientation angle
    2023/01/11 : UE random orientation
*/
const double coherence_time = 130.0; // ms
const double sampling_time = 13.0; // ms
const double angle_mean = 30.0; // degree
const double angle_variance = 7.78; // degree
const double c_1 = pow(0.05, sampling_time/coherence_time);
const double c_0 = (1.0 - c_1) * angle_mean;
const double noise_variance = (1.0 - c_1 * c_1) * angle_variance * angle_variance;


/*
    parameters related to demand discounting ratio
*/
//* const double initial_discount = 0.8;
//* const double delta_p = 0.05;
//* const double expel_ratio = 0.5;


/*
    the period of PCSBM (in states)
*/
const int complete_config_period = state_num;


/*
    for ref 2
    //2//
*/
const double la_overhead = 0.8;
const int LA_UE_num = 4;
#endif // GLOBAL_CONFIGURATION_H
