#ifndef OBJECT_DICTIONARY_H
#define OBJECT_DICTIONARY_H


#include <sstream>
#ifdef ARDUINO
#include <Arduino.h>
#endif // ARDUINO

enum class FSM 
{
    IDLE,
    GIMBAL_HOMING,
    MAIN_VALVES_HOMING,
    TARE_PRESSURES,
    TARE_ORIENTATION,
    ARMED,
    LAUNCH,
    ABORT
};

#ifndef FLOAT_TO_FIXED16
#define FLOAT_TO_FIXED16
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
#endif

#ifndef FIXED16_TO_FLOAT
#define FIXED16_TO_FLOAT
static float fixed16_to_float(uint16_t fixed) 
{
    // Interpret the bits as a signed 16-bit integer
    int16_t signed_val = (int16_t)fixed;

    // Divide by 2^6 (64) to restore fractional scaling
    return (float)signed_val / (1 << 6);
}
#endif
#pragma pack(push, 1)

// Define a struct for your dictionary, if modified, update printObjectDictionary(), 
// On 04.12.2025 the object dictionary was 152 bytes long and with a baudrate of 500000
// beetwen the teensy and the rpi, the logging frequency reach a maximum 200Hz.
struct ObjectDictionary 
{
    float gyro_x = NAN;
    float gyro_y = NAN;
    float gyro_z = NAN;

    float acc_x = NAN;
    float acc_y = NAN;
    float acc_z = NAN;

    float baro = NAN;

    float kalman_yaw = NAN;
    float kalman_pitch = NAN;
    float kalman_roll = NAN;

    float gimbal_x = NAN;
    float gimbal_y = NAN;

    float hv_voltage = NAN;
    float lv_voltage = NAN;

    float chamber_pressure = NAN;
    float pressure_tank_ETH = NAN;
    float pressure_tank_N2O = NAN;
    float pressure_inj_ETH = NAN;
    float pressure_inj_N2O = NAN;
    float pressure_line_ETH = NAN;
    float pressure_line_N2O = NAN;

    float current_ETH_main_valve = NAN; // current as in Amps
    float current_N2O_main_valve = NAN;

    float position_ETH_main_valve = NAN;
    float position_N2O_main_valve = NAN;

    float gimbal_x_current = NAN; // current as in Amps
    float gimbal_y_current = NAN;
    
    float position_gimbal_x = NAN;
    float position_gimbal_y = NAN;

    float temp_N2O = NAN;

    bool vent_ETH = true;
    bool vent_N2O = true;
    bool sol_N2 = false;

    float main_ETH = 0.0;
    float main_N2O = 0.0;

    bool sol_ETH = false;
    bool sol_N2O = false;

    bool igniter = false;

    bool sequence_finished = false;

    bool ETH_main_valves_homing = false;
    bool ETH_main_valves_homing_done = false;

    bool N2O_main_valves_homing = false;
    bool N2O_main_valves_homing_done = false;

    bool gimbal_homing = false;
    bool gimbal_homing_done = false;

    bool thrust_control = false;
    
    bool cmd_idle = false;
    bool cmd_arm = false;
    bool cmd_launch = false;
    bool cmd_abort = false;
    bool cmd_tare_orientation = false;
    bool cmd_tare_pressures = false;

    FSM hopper_state = FSM::IDLE;
};

#ifdef __cplusplus
const size_t object_dictionary_size = sizeof(ObjectDictionary);
#endif

// Create a global instance
extern ObjectDictionary objDict;

#ifdef ARDUINO
inline void printObjectDictionary(const ObjectDictionary &obj) 
{
    Serial.println(F("===== ObjectDictionary ====="));

    // IMU
    Serial.print(F("Gyro X: ")); Serial.println(isnan(obj.gyro_x) ? F("N/A") : String(obj.gyro_x, 3));
    Serial.print(F("Gyro Y: ")); Serial.println(isnan(obj.gyro_y) ? F("N/A") : String(obj.gyro_y, 3));
    Serial.print(F("Gyro Z: ")); Serial.println(isnan(obj.gyro_z) ? F("N/A") : String(obj.gyro_z, 3));

    Serial.print(F("Acc X: ")); Serial.println(isnan(obj.acc_x) ? F("N/A") : String(obj.acc_x, 3));
    Serial.print(F("Acc Y: ")); Serial.println(isnan(obj.acc_y) ? F("N/A") : String(obj.acc_y, 3));
    Serial.print(F("Acc Z: ")); Serial.println(isnan(obj.acc_z) ? F("N/A") : String(obj.acc_z, 3));

    Serial.print(F("Baro: ")); Serial.println(isnan(obj.baro) ? F("N/A") : String(obj.baro, 3));

    Serial.print(F("Kalman Yaw: "));   Serial.println(isnan(obj.kalman_yaw) ? F("N/A") : String(obj.kalman_yaw, 3));
    Serial.print(F("Kalman Pitch: ")); Serial.println(isnan(obj.kalman_pitch) ? F("N/A") : String(obj.kalman_pitch, 3));
    Serial.print(F("Kalman Roll: "));  Serial.println(isnan(obj.kalman_roll) ? F("N/A") : String(obj.kalman_roll, 3));

    // Gimbal
    Serial.print(F("Gimbal X: ")); Serial.println(isnan(obj.gimbal_x) ? F("N/A") : String(obj.gimbal_x, 3));
    Serial.print(F("Gimbal Y: ")); Serial.println(isnan(obj.gimbal_y) ? F("N/A") : String(obj.gimbal_y, 3));

    // Power
    Serial.print(F("HV Voltage: ")); Serial.println(isnan(obj.hv_voltage) ? F("N/A") : String(obj.hv_voltage, 3));
    Serial.print(F("LV Voltage: ")); Serial.println(isnan(obj.lv_voltage) ? F("N/A") : String(obj.lv_voltage, 3));

    // Pressures
    Serial.print(F("Chamber Pressure: ")); Serial.println(isnan(obj.chamber_pressure) ? F("N/A") : String(obj.chamber_pressure, 3));
    Serial.print(F("Pressure Tank ETH: ")); Serial.println(isnan(obj.pressure_tank_ETH) ? F("N/A") : String(obj.pressure_tank_ETH, 3));
    Serial.print(F("Pressure Tank N2O: ")); Serial.println(isnan(obj.pressure_tank_N2O) ? F("N/A") : String(obj.pressure_tank_N2O, 3));
    Serial.print(F("Pressure Inj ETH: ")); Serial.println(isnan(obj.pressure_inj_ETH) ? F("N/A") : String(obj.pressure_inj_ETH, 3));
    Serial.print(F("Pressure Inj N2O: ")); Serial.println(isnan(obj.pressure_inj_N2O) ? F("N/A") : String(obj.pressure_inj_N2O, 3));
    Serial.print(F("Pressure Line ETH: ")); Serial.println(isnan(obj.pressure_line_ETH) ? F("N/A") : String(obj.pressure_line_ETH, 3));
    Serial.print(F("Pressure Line N2O: ")); Serial.println(isnan(obj.pressure_line_N2O) ? F("N/A") : String(obj.pressure_line_N2O, 3));
    Serial.print(F("Temp N2O: "));         Serial.println(isnan(obj.temp_N2O) ? F("N/A") : String(obj.temp_N2O, 3));

    // Valves / Solenoids
    Serial.print(F("Vent ETH: "));     Serial.println(obj.vent_ETH ? F("true") : F("false"));
    Serial.print(F("Vent N2O: "));     Serial.println(obj.vent_N2O ? F("true") : F("false"));
    Serial.print(F("Solenoid N2: "));  Serial.println(obj.sol_N2 ? F("true") : F("false"));
    Serial.print(F("Sol ETH: "));      Serial.println(obj.sol_ETH ? F("true") : F("false"));
    Serial.print(F("Sol N2O: "));      Serial.println(obj.sol_N2O ? F("true") : F("false"));

    Serial.print(F("Main ETH: ")); Serial.println(isnan(obj.main_ETH) ? F("N/A") : String(obj.main_ETH, 3));
    Serial.print(F("Main N2O: ")); Serial.println(isnan(obj.main_N2O) ? F("N/A") : String(obj.main_N2O, 3));

    Serial.print(F("Current ETH Main Valve: ")); Serial.println(isnan(obj.current_ETH_main_valve) ? F("N/A") : String(obj.current_ETH_main_valve, 3));
    Serial.print(F("Current N2O Main Valve: ")); Serial.println(isnan(obj.current_N2O_main_valve) ? F("N/A") : String(obj.current_N2O_main_valve, 3));

    Serial.print(F("Position ETH Main Valve: ")); Serial.println(isnan(obj.position_ETH_main_valve) ? F("N/A") : String(obj.position_ETH_main_valve, 3));
    Serial.print(F("Position N2O Main Valve: ")); Serial.println(isnan(obj.position_N2O_main_valve) ? F("N/A") : String(obj.position_N2O_main_valve, 3));

    Serial.print(F("Gimbal X Current: ")); Serial.println(isnan(obj.gimbal_x_current) ? F("N/A") : String(obj.gimbal_x_current, 3));
    Serial.print(F("Gimbal Y Current: ")); Serial.println(isnan(obj.gimbal_y_current) ? F("N/A") : String(obj.gimbal_y_current, 3));

    Serial.print(F("Position Gimbal X: ")); Serial.println(isnan(obj.position_gimbal_x) ? F("N/A") : String(obj.position_gimbal_x, 3));
    Serial.print(F("Position Gimbal Y: ")); Serial.println(isnan(obj.position_gimbal_y) ? F("N/A") : String(obj.position_gimbal_y, 3));


    // Homing
    Serial.print(F("ETH Main Valves Homing: "));      Serial.println(obj.ETH_main_valves_homing ? F("true") : F("false"));
    Serial.print(F("N2O Main Valves Homing: "));   Serial.println(obj.N2O_main_valves_homing ? F("true") : F("false"));
    Serial.print(F("ETH Main Valves Homing Done: ")); Serial.println(obj.ETH_main_valves_homing_done ? F("true") : F("false"));
    Serial.print(F("N2O Main Valves Homing Done: ")); Serial.println(obj.N2O_main_valves_homing_done ? F("true") : F("false"));

    // Sequence
    Serial.print(F("Sequence Finished: ")); Serial.println(obj.sequence_finished ? F("true") : F("false"));

    // Commands
    Serial.print(F("CMD Arm: "));              Serial.println(obj.cmd_arm ? F("true") : F("false"));
    Serial.print(F("CMD Launch: "));           Serial.println(obj.cmd_launch ? F("true") : F("false"));
    Serial.print(F("CMD Abort: "));            Serial.println(obj.cmd_abort ? F("true") : F("false"));
    Serial.print(F("CMD Tare Orientation: "));        Serial.println(obj.cmd_tare_orientation ? F("true") : F("false"));
    Serial.print(F("CMD Tare Pressures: "));             Serial.println(obj.cmd_tare_pressures ? F("true") : F("false"));

    Serial.println(F("============================"));
}
#endif // ARDUINO

inline std::string objectDictionaryCSV(const ObjectDictionary &obj)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(9);
    ss  << float(obj.gyro_x) << "," << float(obj.gyro_y) << "," << float(obj.gyro_z) << ","
        << float(obj.acc_x) << "," << float(obj.acc_y) << "," << float(obj.acc_z) << ","
        << float(obj.baro) << ","
        << float(obj.kalman_yaw) << "," << float(obj.kalman_pitch) << "," << float(obj.kalman_roll) << ","
        << float(obj.gimbal_x) << "," << float(obj.gimbal_x_current) << "," << float(obj.position_gimbal_x) << ","
        << float(obj.gimbal_y) << "," << float(obj.gimbal_y_current) << "," << float(obj.position_gimbal_y) << ","
        << float(obj.hv_voltage) << "," << float(obj.lv_voltage) << ","
        << float(obj.chamber_pressure) << "," << float(obj.pressure_tank_ETH) << "," << float(obj.pressure_tank_N2O) << "," 
        << float(obj.pressure_line_ETH) << "," << float(obj.pressure_line_N2O) << ","
        << float(obj.pressure_inj_ETH) << "," << float(obj.pressure_inj_N2O) << "," << float(obj.temp_N2O) << ","
        << float(obj.vent_ETH) << "," << float(obj.vent_N2O) << "," << float(obj.sol_N2) << ","
        << float(obj.main_ETH) << "," << float(obj.current_ETH_main_valve) << "," << float(obj.position_ETH_main_valve) << "," 
        << float(obj.main_N2O) << "," << float(obj.current_N2O_main_valve) << "," << float(obj.position_N2O_main_valve) << ","
        << float(obj.sol_ETH) << "," << float(obj.sol_N2O) << ","
        << float(obj.igniter) << "," << float(obj.sequence_finished) << ","
        << float(obj.ETH_main_valves_homing) << "," << float(obj.ETH_main_valves_homing_done) << ","
        << float(obj.N2O_main_valves_homing) << "," << float(obj.N2O_main_valves_homing_done) << ","
        << float(obj.gimbal_homing) << "," << float(obj.gimbal_homing_done) << ","
        << float(obj.cmd_idle) << "," << float(obj.cmd_arm) << "," << float(obj.cmd_launch) << "," << float(obj.cmd_abort) << ","
        << float(obj.cmd_tare_orientation) << "," << float(obj.cmd_tare_pressures) << ","
        << float(obj.hopper_state);
    return ss.str();
}

inline std::string objectDictionaryCSVHeader() {
    return "gyro_x,gyro_y,gyro_z,"
           "acc_x,acc_y,acc_z,"
           "baro,"
           "kalman_yaw,kalman_pitch,kalman_roll,"
           "gimbal_x,gimbal_x_current,position_gimbal_x,"
           "gimbal_y,gimbal_y_current,position_gimbal_y,"
           "hv_voltage,lv_voltage,"
           "chamber_pressure,pressure_tank_ETH,pressure_tank_N2O,"
           "pressure_line_ETH,pressure_line_N2O,"
           "pressure_inj_ETH,pressure_inj_N2O,"
           "temp_N2O,"
           "vent_ETH,vent_N2O,sol_N2,"
           "main_ETH,current_ETH_main_valve, position_ETH_main_valve,"
           "main_N2O,current_N2O_main_valve, position_N2O_main_valve,"
           "sol_ETH,sol_N2O,"
           "igniter,sequence_finished,"
           "eth_main_valves_homing,eth_main_valves_homing_done,"
           "n2o_main_valves_homing,n2o_main_valves_homing_done,"
           "gimbal_homing,gimbal_homing_done,"
           "cmd_idle,cmd_arm,cmd_launch,cmd_abort,"
           "cmd_tare_orientation,cmd_tare_pressures,"
           "hopper_state";
}

#endif // OBJECT_DICTIONARY_H