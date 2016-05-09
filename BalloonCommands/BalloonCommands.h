// Written by Andrew Donelick
// <adonelick@hmc.edu>
// 17 June 2014

// Defines the commands to be used in the SPARKY balloon system

#define CUTDOWN                     0
#define CHANGE_TRANSMIT_RATE        1 // Expects a transmission value

// Heater commands
#define TURN_HEATER_ON              2
#define TURN_HEATER_OFF             3
#define MANUAL_HEATER_CONTROL       4 // Expects a transmission value

// Attitude control commands
#define TOGGLE_ATTITUDE_CONTROL     5 // Expects a transmission value
#define SET_YAW                     6 // Expects a transmission value

// Relay controls
#define SWITCH_RELAYS           100

// Checks whether radio communication is possible
// between the ground and the balloon
#define CHECK_RADIO_CONNECTION  0xFFFF	

