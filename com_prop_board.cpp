/*
    File: com_prop_board.cpp
    Author: Axel Juaneda
    Organization: EPFL Rocket Team
    Version : 1.0
*/

#include "com_prop_board.h"

static void handleSerialCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

static CapsuleStatic SerialCapsule(handleSerialCapsule);

static prop_board_downlink_packet downlink_packet;
static prop_board_uplink_packet* uplink_packet;

static uint16_t float_to_fixed16(float value) 
{
    // Scale by 2^6 (because 6 fractional bits)
    int32_t scaled = (int32_t)roundf(value * (1 << 6));

    // Range check: signed 16-bit fixed Q9.6 covers -32768/64 to 32767/64
    if (scaled > 0x7FFF) {   // clamp to max positive
        scaled = 0x7FFF;
    } else if (scaled < -0x8000) { // clamp to min negative
        scaled = -0x8000;
    }

    // Store as uint16_t, but bit pattern is signed two’s complement
    return (uint16_t)(scaled & 0xFFFF);
}

static float fixed16_to_float(uint16_t fixed) 
{
    // Interpret the bits as a signed 16-bit integer
    int16_t signed_val = (int16_t)fixed;

    // Divide by 2^6 (64) to restore fractional scaling
    return (float)signed_val / (1 << 6);
}

void begin_com_prop_board(uint32_t baudrate)
{
    AV_RS232.begin(baudrate);
    uplink_packet = (prop_board_uplink_packet*)calloc(1, sizeof(prop_board_uplink_packet));
}

void write_prop_board(ObjectDictionary &objDict)
{   
    uint32_t codedLen = SerialCapsule.getCodedLen(prop_board_downlink_packet_size);
    if (AV_RS232.availableForWrite() >= (int)codedLen)
    {
        uint8_t packetID = PROP_BOARD_ID;

        downlink_packet.gimbal_x = float_to_fixed16(objDict.gimbal_x);
        downlink_packet.gimbal_y = float_to_fixed16(objDict.gimbal_y);

        downlink_packet.main_ETH = float_to_fixed16(objDict.main_ETH);
        downlink_packet.main_N2O = float_to_fixed16(objDict.main_N2O);
        downlink_packet.sol_ETH = (uint8_t)objDict.sol_ETH;
        downlink_packet.sol_N2O = (uint8_t)objDict.sol_N2O;

        downlink_packet.vent_ETH = (uint8_t)objDict.vent_ETH;
        downlink_packet.vent_N2O = (uint8_t)objDict.vent_N2O;

        downlink_packet.sol_N2 = (uint8_t)objDict.sol_N2;

        downlink_packet.main_valves_homing = (uint8_t)objDict.main_valves_homing;
        downlink_packet.gimbal_homing = (uint8_t)objDict.gimbal_homing;

        uint8_t *packetData = reinterpret_cast<uint8_t *>(&downlink_packet);

        uint8_t *packetToSend = SerialCapsule.encode(packetID, packetData, prop_board_downlink_packet_size);
        
        AV_RS232.write(packetToSend, codedLen);
        delete [] packetToSend;
    }
}

void read_prop_board(ObjectDictionary &objDict)
{
    if (AV_RS232.available() < SerialCapsule.getCodedLen(prop_board_uplink_packet_size))
        return;

    while (AV_RS232.available())
    {
        uint8_t byte = AV_RS232.read();
        SerialCapsule.decode(byte);
    }

    // Update object dictionary from uplink packet if available
    if (uplink_packet) 
    {
        objDict.pressure_ETH        = fixed16_to_float(uplink_packet->pressure_ETH);
        objDict.pressure_N2O        = fixed16_to_float(uplink_packet->pressure_N2O);
        objDict.pressure_inj_ETH    = fixed16_to_float(uplink_packet->pressure_inj_ETH);
        objDict.pressure_inj_N2O    = fixed16_to_float(uplink_packet->pressure_inj_N2O);
        objDict.chamber_pressure    = fixed16_to_float(uplink_packet->chamber_pressure);
        objDict.temp_N2O            = fixed16_to_float(uplink_packet->temp_N2O);
        objDict.hv_voltage          = fixed16_to_float(uplink_packet->hv_voltage);
        objDict.main_valves_homing_done = (bool)uplink_packet->main_valves_homing_done;
        objDict.gimbal_homing_done = (bool)uplink_packet->gimbal_homing_done;
    }
}

void handleSerialCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len)
{   
   if (len == prop_board_uplink_packet_size)
        memcpy(uplink_packet, dataIn, prop_board_uplink_packet_size);
}