#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

/*

2023/01/05 : //* means liu_code exist
2023/01/10 :
    /ref'1/ means that this func.(var.) is used for reference : Learning-Based Energy-Efficient Resource Management by Heterogeneous RF/VLC for Ultra-Reliable Low-Latency Industrial IoT Networks
    /ref'2/ means that this func.(var.) is used for reference : Improving the performance of Heterogeneous LiFi-WiFi network using a novel Link Aggregation Framework
*/

#define DEBUG_MODE 0
#define PROPOSED_METHOD 0
#define LASINR 0
#define LAEQOS 1
#define PDSERT 0
#define PCSBM 1 // can change


const double PI = boost::math::constants::pi<double>();
const double room_size = 5; // m // @-ref'1- 24@ , @-ref'2- 5@
const double time_period = 0.5; // sec //
const int state_num = 1;


/*
    RF AP
*/
const int RF_AP_num = 1;
const int RF_AP_height = 3; // m //  @-ref'1- 5@ , @-ref'2- 3@
const int RF_AP_bandwidth = 20; // MHz // @-ref'1- 10@ , @-ref'2- 20@
const int RF_AP_subchannel = 32; // number of sub channel // @-ref'1- 32@ , @-ref'2- none@
const double RF_AP_power = 0.1; // W // @-ref'1- 0.03@ , @-ref'2- 0.1@
const double RF_noise_power_spectral_density =  3.16e-11; // A^2/MHz // @-ref'1- 1e-3@ , @-ref'2- 3.16e-11@

/*
dBm -> A^2

https://www.convertworld.com/zh-hant/power/dbm.html
dBm -> W = A^2 * Ω
( Ω = 1 )

ref'1 : N^RF_0 = 173 dBm/Hz = 1e-3 A^2/MHz
ref'2 : -75 dBm/MHz ~= 3.16e-11 A^2/MHz

*/


/*
    VLC AP
*/
const int VLC_AP_num = 4; // @-ref'1- 36@ , @-ref'2- 4@
const int VLC_AP_per_row = 2; // @-ref'1- 6@ , @-ref'2- 2@
const int VLC_AP_height = 3; // m // @-ref'1- 5@ , @-ref'2- 3@
const int VLC_AP_power = 3; // W // @-ref'1- calculate@ , @-ref'2- 3@
const int VLC_AP_bandwidth = 40; // MHz // @-ref'1- 20@ , @-ref'2- 40@
const int VLC_AP_subchannel = 16; // @-ref'1- 16@ , @-ref'2- none@
const double VLC_noise_power_spectral_density = 1e-24; // A^2/MHz // @-ref'1- 1e-15@ , @-ref'2- 1e-24@
/*
ref'1 : 1e-21 A^2/Hz = 1e-15 A^2/MHz
ref'2 : -210 dBm/MHz ~= 1e-24 A^2/MHz
*/
const double conversion_efficiency = 0.53; // A/W // @-ref'1- 0.5@ , @-ref'2- 0.53@  optical to electrical conversion efficiency (τ) , PD’s responsivity (μ) ,


// these values are found in "Resource Allocation in LiFi OFDMA Systems"
//* const int subcarrier_num = 40; // M = 64
//* const int effective_subcarrier_num = subcarrier_num / 2 - 1; // M_e = M/2 - 1
//* const int time_slot_num = 32; // K = 20


/*
    UE
    !*-*-NOTICE*-*-! 20230113 : demand_upper_bound NEED change
*/
const int UE_num = 10; // @-ref'1- 120@ , @-ref'2- 10@
const double UE_height = 0; // m // @-ref'1- calculate@ , @-ref'2- 0@
/*
!*-*-TODO*-*-! : ref'1 need calculate

UE_height = a number of devices are randomly distributed at four different heights : 0.5 , 1 , 1.5 , 2 m

*/


//* const int demand_upper_bound = 100;
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
// these parameters are found in senior's code(liu)
const double fitting_coefficient = 2.88;
const double three_dB_cutoff = 2; // MHz
//* const double reflection_efficiency = 0.75;

/*
    RF Channel
*/
const double RF_carrier_frequency = 2.4e9; // GHz //
const int breakpoint_distance = 5; // m // @-ref'1- none@ , @-ref'2- 5@

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
// const int complete_config_period = state_num;


/*
    ref'2 system parameter
*/
const double la_overhead = 0.8;
const int LA_UE_num = 4;
const double require_data_rate_threshold = 25; //Mbps

/*
    ref'1 system parameter
*/
const int effective_VLC_subchannel = VLC_AP_subchannel / 2 - 1; // The scaling factor 1/2 is due to the Hermitian symmetry
const int effective_RF_subchannel = RF_AP_subchannel / 2 - 1;
const int wall_num = 5;

#endif // GLOBAL_CONFIGURATION_H
