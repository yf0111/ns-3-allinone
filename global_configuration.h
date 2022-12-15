#ifndef GLOBAL_CONFIGURATION_H
#define GLOBAL_CONFIGURATION_H

#include <cmath>
#include <boost/math/constants/constants.hpp>

#define DEBUG_MODE 0
#define PROPOSED_METHOD 1
#define PCSBM 1


const double PI = boost::math::constants::pi<double>();
const double room_size = 16;
const double time_period = 0.5; // sec
const int state_num = 1000;


/*
    RF AP
*/
const int RF_AP_num = 1;
const int RF_AP_height = 5; // m
const int RF_AP_bandwidth = 10; //MHz
const int RF_AP_subchannel = 32; // number of subchannel

/*
    VLC AP
*/
const int VLC_AP_num = 36;
const int VLC_AP_per_row = 6;
const int VLC_AP_height = 5;
//*const int VLC_AP_power = 9; //optical transmit power in lifi system (w)
const int VLC_AP_bandwidth = 20; // MHz
const int VLC_AP_subchannel = 16;
const double noise_power_spectral_density = 1e-15;  //N^VLC_0 = 1e-21 A^2/Hz = 1e-15 A^2/MHz
//*const double conversion_efficiency = 0.53; // optical to electrical conversion efficiency
//*const double optical_to_electric_power_ratio = 3.0; // κ

// these values are found in "Resource Allocation in LiFi OFDMA Systems"
//*const int subcarrier_num = 40; // M = 64
//*const int effective_subcarrier_num = subcarrier_num / 2 - 1; // M_e = M/2 - 1
//*const int time_slot_num = 32; // K = 20


/*
    UE
*/
const int UE_num = 150;
const int demand_upper_bound = 100;
const double UE_height = 1.5;
const double avg_speed = 1.0; // m/s
const double pause_time = 0.0;

/*
    VLC channel
*/
const double field_of_view = 90.0; // degree
//*const double PHI_half = 60.0; // semi-angle at half-illumination in degree
const double filter_gain = 1.0;
const double refractive_index = 1.5;
const double receiver_area = 1e-4; // 1 cm^2 = 0.0001 m^2
//*const double reflection_efficiency = 0.75;
//*const double fitting_coefficient = 2.88;
//*const double three_dB_cutoff = 30; // MHz

/*
    handover
*/
//*const double VHO_efficiency = 0.6;
//*const double HHO_efficiency = 0.9;


/*
    RF channel
*/
//*const int max_backoff_stage = 1024;
//*const double channel_bit_rate = 1732.0 * 5;
//*const double RTS_time = 160.0; // µs
//*const double CTS_time = 140.0; // µs
//*const double header_time = 230.0; // µs
//*const double ACK_time = 140.0; // µs
//*const double SIFS_time = 28.0; // µs
//*const double PIFS_time = 80.0; // µs
//*const double DIFS_time = 128.0; // µs
//*const double slot_time = 52.0; // It is not given in benchmark paper. I infer this value by PIFS = SIFS + slot time based on ieee 802.11.
//*const double propagation_delay = 1000.0; // µs
//*const double utilization_ratio = 2.0; // ε

/*
    random orientation angle
*/
//*const double coherence_time = 130.0; // ms
//*const double sampling_time = 13.0; // ms
//*const double angle_mean = 30.0; // degree
//*const double angle_variance = 7.78; // degree
//*const double c_1 = pow(0.05, sampling_time/coherence_time);
//*const double c_0 = (1.0 - c_1) * angle_mean;
//*const double noise_variance = (1.0 - c_1 * c_1) * angle_variance * angle_variance;


/*
    utility function
*/
//*const double beta = 1.0; // fairness coefficient


/*
    parameters related to demand discounting ratio
*/
//*const double initial_discount = 0.8;
//*const double delta_p = 0.05;
//*const double expel_ratio = 0.5;


/*
    the period of PCSBM (in states)
*/
//*const int complete_config_period = state_num;

#endif // GLOBAL_CONFIGURATION_H
