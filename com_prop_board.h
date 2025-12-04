/*
    File: com_prop_board.h
    Author: Axel Juaneda
    Organization: EPFL Rocket Team
    Version : 1.0
*/
#ifndef COM_PROP_BOARD_H
#define COM_PROP_BOARD_H

#include <Arduino.h>

#include <capsule.h>
#include <object_dictionary.h>

#define AV_RS232 Serial3
#define PROP_BOARD_ID 46

#if defined(ICARUS_PROP)
void begin_com_avionics(uint32_t baudrate = 115200);
void write_avionics(ObjectDictionary &objDict);
void read_avionics(ObjectDictionary &objDict);
#elif defined(ICARUS_AV) 
void begin_com_prop_board(uint32_t baudrate = 115200);
bool write_prop_board(ObjectDictionary &objDict);
void read_prop_board(ObjectDictionary &objDict);
#endif

typedef struct __attribute__((__packed__)) {
    uint16_t main_ETH = 0;       // [0;100]
    uint16_t main_N2O = 0;       // [0;100]
    uint8_t sol_ETH = false;
    uint8_t sol_N2O = false;

    uint16_t gimbal_x = 0;      // [-15.0;15.0]
    uint16_t gimbal_y = 0;      // [-15.0;15.0]

    uint8_t ETH_main_valves_homing = false;
    uint8_t N2O_main_valves_homing = false;

    uint8_t thrust_control = false;

    uint8_t vent_ETH = false;
    uint8_t vent_N2O = false;
    uint8_t sol_N2 = false;
} prop_board_downlink_packet;

#ifdef __cplusplus
const uint32_t prop_board_downlink_packet_size = sizeof(prop_board_downlink_packet);
#endif

typedef struct __attribute__((__packed__)) {
    uint16_t pressure_tank_ETH = 0;      // [0.0;50.0]
    uint16_t pressure_tank_N2O = 0;      // [0.0;50.0]
    uint16_t pressure_inj_ETH = 0;  // [0.0;50.0]
    uint16_t pressure_inj_N2O = 0;  // [0.0;50.0]
    uint16_t pressure_line_ETH = 0;  // [0.0;50.0]
    uint16_t pressure_line_N2O = 0;  // [0.0;50.0]
    uint16_t chamber_pressure = 0;  // [0.0;50.0]

    uint16_t N2O_main_valve_position = 0;   // [0.0; 100.0]
    uint16_t ETH_main_valve_position = 0;   // [0.0; 100.0]
    uint16_t gimbal_x_position = 0;    // [0.0; 100.0]
    uint16_t gimbal_y_position = 0;    // [0.0; 100.0]


    uint8_t ETH_main_valves_homing_done = false;
    uint8_t N2O_main_valves_homing_done = false;
    

    uint16_t temp_N2O = 0;          // [-70.0;40.0]

    uint16_t hv_voltage = 0;        // [0.0;26.0]
} prop_board_uplink_packet;

#ifdef __cplusplus
const uint32_t prop_board_uplink_packet_size = sizeof(prop_board_uplink_packet);
#endif

#endif