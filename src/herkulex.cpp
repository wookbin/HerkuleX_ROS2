//HerkuleX_node
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/byte.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <csignal>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <atomic>
#include <list>
#include <functional>

#include "herkulex.h"
#include "herkulex/msg/msg_herkulex_ram.hpp" //MSG
#include "herkulex/msg/msg_herkulex_eep.hpp" //MSG
#include "herkulex/msg/herkulex_info_ram.hpp" //MSG_Array
#include "herkulex/msg/herkulex_info_eep.hpp" //MSG_Array

#include "herkulex/srv/herkulex_register_command.hpp" //SRV
#include "herkulex/srv/herkulex_position_move.hpp" //SRV
#include "herkulex/srv/herkulex_velocity_move.hpp" //SRV
#include "herkulex/srv/herkulex_sjog_move.hpp" //SRV
#include "herkulex/srv/herkulex_ijog_move.hpp" //SRV
#include "herkulex/srv/herkulex_angle_move.hpp" //SRV


using namespace std;
using namespace LibSerial;
using std::placeholders::_1;
using std::placeholders::_2;

/********************************************************************************/
std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}
/********************************************************************************/

/*********************************************************************************************************************************/
unsigned char EEP_REG_SIZE[] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0 };
unsigned char RAM_REG_SIZE[] = { 1,1,1,1,1,1,1,1,1,1,1,1,2,0,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0 };

int m_iModel = 0; //0: DRS-0101, 1: DRS-0102, 2: DRS-0201, 3: DRS-0401, 4: DRS-0402, 5: DRS-0601, 6: DRS-0602
// HerkuleX ID number: ex)1 ~ 253
unsigned char szIDs[] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
						  0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,
						  0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,
						  0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,
						  0x3d,0x3e,0x3f,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,
						  0x4c,0x4d,0x4e,0x4f,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,
						  0x5b,0x5c,0x5d,0x5e,0x5f,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
						  0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,
						  0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,
						  0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x96,
						  0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,
						  0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
						  0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,0xc0,0xc1,0xc2,0xc3,
						  0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,
						  0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,0xe0,0xe1,
						  0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,0xf0,
						  0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe };

#define EEPRegSize(addr) (EEP_REG_SIZE[addr])
#define RAMRegSize(addr) (RAM_REG_SIZE[addr])

SJog			sjogs[255];
IJog			ijogs[255];
RAMRegisterMap	RAM[255];
EEPRegisterMap	EEP[255];
CMD_SJog		sjogArr[255];
CMD_IJog		ijogArr[255];

unsigned char szSendBuffer[1024] = {0, };//send buffer
int  m_idata_size = 0;
int	 nPacketLength = 0;
unsigned char ucValue = 0;
bool m_bRAM_ReadAll_Flag = false;
bool m_bEEP_ReadAll_Flag = false;
int m_iTotal_Axis = 1;

/*********************************************************************************************************************************/
eep_write_ack* _eep_write_ack_callback = 0;
eep_map_write_ack* _eep_map_write_ack_callback = 0;
eep_read_ack* _eep_read_ack_callback = 0;
eep_map_read_ack* _eep_map_read_ack_callback = 0;

ram_write_ack* _ram_write_ack_callback = 0;
ram_write_ack* _ram_map_write_ack_callback = 0;
ram_read_ack* _ram_read_ack_callback = 0;
ram_map_read_ack* _ram_map_read_ack_callback = 0;

i_jog_ack* _i_jog_ack_callback = 0;
s_jog_ack* _s_jog_ack_callback = 0;
stat_ack* _stat_ack_callback = 0;
rollback_ack* _rollback_ack_callback = 0;
reboot_ack* _reboot_ack_callback = 0;

RAMRegisterMap _ramMap;
EEPRegisterMap _eepMap;

//*define///////////////////////////////////////////////////////////////////*/
//Array RAM & EEP msgs//
herkulex::msg::HerkulexInfoRam RAM_Array;
herkulex::msg::HerkulexInfoEep EEP_Array;
//Array Publish//
rclcpp::Publisher<herkulex::msg::HerkulexInfoRam>::SharedPtr RAM_Array_pub[255];
rclcpp::Publisher<herkulex::msg::HerkulexInfoEep>::SharedPtr EEP_Array_pub[255];
//RAM & EEP msg
herkulex::msg::MsgHerkulexRam RAM_msg;
herkulex::msg::MsgHerkulexEep EEP_msg;
//std_msgs read & write buffer
std_msgs::msg::UInt8MultiArray r_buffer;
std_msgs::msg::UInt8MultiArray w_buffer;

void CALLING_CONVENTION EEPWriteAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("EEPWriteAckCallback...");
}
void CALLING_CONVENTION EEPReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail)
{
    //printf("EEPReadAckCallback...");
}
void CALLING_CONVENTION RAMWriteAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("RAMWriteAckCallback...");
}
void CALLING_CONVENTION RAMReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, void* value, unsigned char status_error, unsigned char status_detail)
{
    //printf("RAMReadAckCallback...");
}
void CALLING_CONVENTION StatAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("StatAckCallback...");
}
void CALLING_CONVENTION IJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("IJogAckCallback...");
}
void CALLING_CONVENTION SJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("SJogAckCallback...");
}
void CALLING_CONVENTION MIJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("MIJogAckCallback...");
}
void CALLING_CONVENTION MSJogAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("MSJogAckCallback...");
}
void CALLING_CONVENTION RollbackAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("RollbackAckCallback...");
}
void CALLING_CONVENTION RebootAckCallback(unsigned char sid, unsigned char status_error, unsigned char status_detail)
{
    //printf("RebootAckCallback...");
}

void CALLING_CONVENTION regist_ack_callback_eep_write(eep_write_ack* callback)
{
    _eep_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_map_write(eep_map_write_ack* callback)
{
    _eep_map_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_read(eep_read_ack* callback)
{
    _eep_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_eep_map_read(eep_map_read_ack* callback)
{
    _eep_map_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_write(ram_write_ack* callback)
{
    _ram_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_map_write(ram_map_write_ack* callback)
{
    _ram_map_write_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_read(ram_read_ack* callback)
{
    _ram_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_ram_map_read(ram_map_read_ack* callback)
{
    _ram_map_read_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_i_jog(i_jog_ack* callback)
{
    _i_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_s_jog(s_jog_ack* callback)
{
    _s_jog_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_stat(stat_ack* callback)
{
    _stat_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_rollback(rollback_ack* callback)
{
    _rollback_ack_callback = callback;
}

void CALLING_CONVENTION regist_ack_callback_reboot(reboot_ack* callback)
{
    _reboot_ack_callback = callback;
}

void CALLING_CONVENTION EEPMapReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, EEPRegisterMap map, unsigned char status_error, unsigned char status_detail)
{
    if(m_bEEP_ReadAll_Flag)
    {
        memcpy(&(EEP[sid]), &map, sizeof(EEPRegisterMap));
        memcpy(&EEP_msg, &map, sizeof(EEPRegisterMap));
        m_bEEP_ReadAll_Flag = false;
    }
    else
    {
        switch (address)
        {
        case 0:
            EEP_msg.modelno1 = EEP[sid].ucModelNo1 = map.ucModelNo1;
            break;
        case 1:
            EEP_msg.modelno2 = EEP[sid].ucModelNo2 = map.ucModelNo2;
            break;
        case 2:
            EEP_msg.version1 = EEP[sid].ucVersion1 = map.ucVersion1;
            break;
        case 3:
            EEP_msg.version2 = EEP[sid].ucVersion2 = map.ucVersion2;
            break;
        case 4:
            EEP_msg.baudrate = EEP[sid].ucBaudRate = map.ucBaudRate;
            break;
        case 5:
            EEP_msg.reaserved1 = EEP[sid].ucReaserved1 = map.ucReaserved1;
            break;
        case 6:
            EEP_msg.id = EEP[sid].ucID = map.ucID;
            break;
        case 7:
            EEP_msg.ackpolicy = EEP[sid].ucAckPolicy = map.ucAckPolicy;
            break;
        case 8:
            EEP_msg.alarmledpolicy = EEP[sid].ucAlarmLEDPolicy = map.ucAlarmLEDPolicy;
            break;
        case 9:
            EEP_msg.torquepolicy = EEP[sid].ucTorquePolicy = map.ucTorquePolicy;
            break;
        case 10:
            EEP_msg.reserved2 = EEP[sid].ucReserved2 = map.ucReserved2;
            break;
        case 11:
            EEP_msg.maxtemperature = EEP[sid].ucMaxTemperature = map.ucMaxTemperature;
            break;
        case 12:
            EEP_msg.minvoltage = EEP[sid].ucMinVoltage = map.ucMinVoltage;
            break;
        case 13:
            EEP_msg.maxvoltage = EEP[sid].ucMaxVoltage = map.ucMaxVoltage;
            break;
        case 14:
            EEP_msg.accelerationratio = EEP[sid].ucAccelerationRatio = map.ucAccelerationRatio;
            break;
        case 15:
            EEP_msg.maxaccelerationtime = EEP[sid].ucMaxAccelerationTime = map.ucMaxAccelerationTime;
            break;
        case 16:
            EEP_msg.deadzone = EEP[sid].ucDeadZone = map.ucDeadZone;
            break;
        case 17:
            EEP_msg.saturatoroffset = EEP[sid].ucSaturatorOffset = map.ucSaturatorOffset;
            break;
        case 18:
            EEP_msg.saturatorslope = EEP[sid].usSaturatorSlope = map.usSaturatorSlope;
            break;
        case 20:
            EEP_msg.pwmoffset = EEP[sid].cPWMOffset = map.cPWMOffset;
            break;
        case 21:
            EEP_msg.minpwm = EEP[sid].ucMinPWM = map.ucMinPWM;
            break;
        case 22:
            EEP_msg.maxpwm = EEP[sid].usMaxPWM = map.usMaxPWM;
            break;
        case 24:
            EEP_msg.overloadpwmthreshold = EEP[sid].usOverloadPWMThreshold = map.usOverloadPWMThreshold;
            break;
        case 26:
            EEP_msg.minposition = EEP[sid].usMinPosition = map.usMinPosition;
            break;
        case 28:
            EEP_msg.maxposition = EEP[sid].usMaxPosition = map.usMaxPosition;
            break;
        case 30: 
            EEP_msg.positionkp = EEP[sid].usPositionKp = map.usPositionKp;
            break;
        case 32:
            EEP_msg.positionkd = EEP[sid].usPositionKd = map.usPositionKd;
            break;
        case 34:
            EEP_msg.positionki = EEP[sid].usPositionKi = map.usPositionKi;
            break;
        case 36:
            EEP_msg.positionfeedforward1stgain = EEP[sid].usPositionFeedforward1stGain = map.usPositionFeedforward1stGain;
            break;
        case 38: //usVelocityKp
            EEP_msg.positionfeedforward2ndgain = EEP[sid].usPositionFeedforward2ndGain = map.usPositionFeedforward2ndGain;
            break;
        case 40: 
            EEP_msg.velocitykd = EEP[sid].usVelocityKd = map.usVelocityKd;
            break;
        case 42: 
            EEP_msg.velocityki = EEP[sid].usVelocityKi = map.usVelocityKi;
            break;
        case 44:
            EEP_msg.ledblinkperiod = EEP[sid].ucLEDBlinkPeriod = map.ucLEDBlinkPeriod;
            break;
        case 45:
            EEP_msg.adcfaultcheckperiod = EEP[sid].ucADCFaultCheckPeriod = map.ucADCFaultCheckPeriod;
            break;
        case 46:
            EEP_msg.packetgarbagecheckperiod = EEP[sid].ucPacketGarbageCheckPeriod = map.ucPacketGarbageCheckPeriod;
            break;
        case 47:
            EEP_msg.stopdetectionperiod = EEP[sid].ucStopDetectionPeriod = map.ucStopDetectionPeriod;
            break;
        case 48:
            EEP_msg.overloaddetectionperiod = EEP[sid].ucOverloadDetectionPeriod = map.ucOverloadDetectionPeriod;
            break;
        case 49:
            EEP_msg.stopthreshold = EEP[sid].ucStopThreshold = map.ucStopThreshold;
            break;
        case 50:
            EEP_msg.inpositionmargin = EEP[sid].ucInpositionMargin = map.ucInpositionMargin;
            break;
        case 51:
            EEP_msg.reserved5 = EEP[sid].ucReserved5 = map.ucReserved5;
            break;
        case 52:
            EEP_msg.calibrationdifference_l = EEP[sid].cCalibrationDifference_L = map.cCalibrationDifference_L;
            break;
        case 53:
            EEP_msg.calibrationdifference_h = EEP[sid].cCalibrationDifference_H = map.cCalibrationDifference_H;
            break;
        default:
            printf("Error: Not defined EEP Adress... \n");
            break;
        }
    }

    EEP_Array.index = sid;
    EEP_Array.pherkulex_eep.push_back(EEP_msg);
    EEP_Array_pub[sid]->publish(EEP_Array);
    EEP_Array.pherkulex_eep.clear();
}

void CALLING_CONVENTION RAMMapReadAckCallback(unsigned char sid, unsigned char address, unsigned char length, RAMRegisterMap map, unsigned char status_error, unsigned char status_detail)
{
    if(m_bRAM_ReadAll_Flag)
    {
        memcpy(&(RAM[sid]), &map, sizeof(RAMRegisterMap));
        memcpy(&RAM_msg, &map, sizeof(RAMRegisterMap));
        printf("[in loop]: RAMMapReadAckCallback! \n");
        switch(m_iModel)
        {
            case 0: //0101
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
                break;
            case 1: //0102
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                break;
            case 2: //0201
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
                break;
            case 3: //0401
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                break;
            case 4: //0402
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                break;
            case 5: //0601
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                break;
            case 6: //0602
                RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                break;
            default:
                printf("Error: Not defined Model... \n");
                break;
        }

        m_bRAM_ReadAll_Flag = false;
    }
    else
    {
        switch (address)
        {
        case 0: //ID
            RAM_msg.id = RAM[sid].ucID = map.ucID;
            break;
        case 1: //ACK POLICY
            RAM_msg.ackpolicy = RAM[sid].ucAckPolicy = map.ucAckPolicy;
            break;
        case 2:
            RAM_msg.alarmledpolicy = RAM[sid].ucAlarmLEDPolicy = map.ucAlarmLEDPolicy;
            break;
        case 3:
            RAM_msg.torquepolicy = RAM[sid].ucTorquePolicy = map.ucTorquePolicy;
            break;
        case 4:
            RAM_msg.reserved2 = RAM[sid].ucReserved2 = map.ucReserved2;
            break;
        case 5:
            RAM_msg.maxtemperature = RAM[sid].ucMaxTemperature = map.ucMaxTemperature;
            break;
        case 6:
            RAM_msg.minvoltage = RAM[sid].ucMinVoltage = map.ucMinVoltage;
            break;
        case 7:
            RAM_msg.maxvoltage = RAM[sid].ucMaxVoltage = map.ucMaxVoltage;
            break;
        case 8:
            RAM_msg.accelerationratio = RAM[sid].ucAccelerationRatio = map.ucAccelerationRatio;
            break;
        case 9:
            RAM_msg.maxaccelerationtime = RAM[sid].ucMaxAccelerationTime = map.ucMaxAccelerationTime;
            break;
        case 10:
            RAM_msg.deadzone = RAM[sid].ucDeadZone = map.ucDeadZone;
            break;
        case 11:
            RAM_msg.saturatoroffset = RAM[sid].ucSaturatorOffset = map.ucSaturatorOffset;
            break;
        case 12:
            RAM_msg.saturatorslope = RAM[sid].usSaturatorSlope = map.usSaturatorSlope;
            break;
        case 14:
            RAM_msg.pwmoffset = RAM[sid].cPWMOffset = map.cPWMOffset;
            break;
        case 15:
            RAM_msg.minpwm = RAM[sid].ucMinPWM = map.ucMinPWM;
            break;
        case 16:
            RAM_msg.maxpwm = RAM[sid].usMaxPWM = map.usMaxPWM;
            break;
        case 18:
            RAM_msg.overloadpwmthreshold = RAM[sid].usOverloadPWMThreshold = map.usOverloadPWMThreshold;
            break;
        case 20: //GET_MinPosition
            RAM_msg.minposition = RAM[sid].usMinPosition = map.usMinPosition;
            break;
        case 22: //GET_MaxPosition
            RAM_msg.maxposition = RAM[sid].usMaxPosition = map.usMaxPosition;
            break;
        case 24:
            RAM_msg.positionkp = RAM[sid].usPositionKp = map.usPositionKp;
            break;
        case 26:
            RAM_msg.positionkd = RAM[sid].usPositionKd = map.usPositionKd;
            break;
        case 28:
            RAM_msg.positionki = RAM[sid].usPositionKi = map.usPositionKi;
            break;
        case 30:
            RAM_msg.positionfeedforward1stgain = RAM[sid].usPositionFeedforward1stGain = map.usPositionFeedforward1stGain;
            break;
        case 32:
            RAM_msg.positionfeedforward2ndgain = RAM[sid].usPositionFeedforward2ndGain = map.usPositionFeedforward2ndGain;
            break;
        case 34:
            RAM_msg.velocitykd = RAM[sid].usVelocityKd = map.usVelocityKd;
            break;
        case 36:
            RAM_msg.velocityki = RAM[sid].usVelocityKi = map.usVelocityKi;
            break;
        case 38:
            RAM_msg.ledblinkperiod = RAM[sid].ucLEDBlinkPeriod = map.ucLEDBlinkPeriod;
            break;
        case 39:
            RAM_msg.adcfaultcheckperiod = RAM[sid].ucADCFaultCheckPeriod = map.ucADCFaultCheckPeriod;
            break;
        case 40:
            RAM_msg.packetgarbagecheckperiod = RAM[sid].ucPacketGarbageCheckPeriod = map.ucPacketGarbageCheckPeriod;
            break;
        case 41:
            RAM_msg.stopdetectionperiod = RAM[sid].ucStopDetectionPeriod = map.ucStopDetectionPeriod;
            break;
        case 42:
            RAM_msg.overloaddetectionperiod = RAM[sid].ucOverloadDetectionPeriod = map.ucOverloadDetectionPeriod;
            break;
        case 43:
            RAM_msg.stopthreshold = RAM[sid].ucStopThreshold = map.ucStopThreshold;
            break;
        case 44:
            RAM_msg.inpositionmargin = RAM[sid].ucInpositionMargin = map.ucInpositionMargin;
            break;
        case 45:
            RAM_msg.turn = RAM[sid].ucTurn = map.ucTurn;
            break;
        case 46:
            RAM_msg.calibrationdifference_l = RAM[sid].cCalibrationDifference_L = map.cCalibrationDifference_L;
            break;
        case 47:
            RAM_msg.calibrationdifference_h = RAM[sid].cCalibrationDifference_H = map.cCalibrationDifference_H;
            break;
        case 48:
            RAM_msg.statuserror = RAM[sid].ucStatusError = map.ucStatusError;
            break;
        case 49:
            RAM_msg.statusdetail = RAM[sid].ucStatusDetail = map.ucStatusDetail;
            break;
        case 50:
            RAM_msg.reserved7 = RAM[sid].ucReserved7 = map.ucReserved7;
            break;
        case 51:
            RAM_msg.reserved8 = RAM[sid].ucReserved8 = map.ucReserved8;
            break;
        case 52:
            RAM_msg.torquecontrol = RAM[sid].ucTorqueControl = map.ucTorqueControl;
            break;
        case 53:
            RAM_msg.ledcontrol = RAM[sid].ucLEDControl = map.ucLEDControl;
            break;
        case 54:
            RAM_msg.voltage = RAM[sid].ucVoltage = map.ucVoltage;
            break;
        case 55:
            RAM_msg.temperature = RAM[sid].ucTemperature = map.ucTemperature;
            break;
        case 56:
            RAM_msg.currentcontrolmode = RAM[sid].ucCurrentControlMode = map.ucCurrentControlMode;
            break;
        case 57:
            RAM_msg.tick = RAM[sid].ucTick = map.ucTick;
            break;
        case 58:
            switch(m_iModel)
            {
                case 0: //DRS-0101
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
                    break;
                case 1: //DRS-0102
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                    break;
                case 2: //DRS-0201
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x1FFF;
                    break;
                case 3: //DRS-0401
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                    break;
                case 4: //DRS-0402
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                    break;
                case 5: //DRS-0601
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                    break;
                case 6: //DRS-0602
                    RAM_msg.calibratedposition = RAM[sid].usCalibratedPosition = map.usCalibratedPosition & 0x7FFF;
                    break;
            }
            break;
        case 60:
            RAM_msg.absoluteposition = RAM[sid].usAbsolutePosition = map.usAbsolutePosition;
            break;
        case 62:
            RAM_msg.differentialposition = RAM[sid].sDifferentialPosition = map.sDifferentialPosition;
            break;
        case 64:
            RAM_msg.pwm = RAM[sid].usPWM = map.usPWM;
            break;
        case 66:
            RAM_msg.reserved9 = RAM[sid].usReserved9 = map.usReserved9;
            break;
        case 68:
            RAM_msg.absolutegoalposition = RAM[sid].usAbsoluteGoalPosition = map.usAbsoluteGoalPosition;
            break;
        case 70:
            RAM_msg.absolutedesiredtrajectoryposition = RAM[sid].usAbsoluteDesiredTrajectoryPosition = map.usAbsoluteDesiredTrajectoryPosition;
            break;
        case 72:
            RAM_msg.desiredvelocity = RAM[sid].sDesiredVelocity = map.sDesiredVelocity;
            break;
        default:
            printf("Error: Not defined RAM Adress... \n");
            break;
        }
    }

    RAM_Array.index = sid;
    RAM_Array.pherkulex_ram.push_back(RAM_msg);
    RAM_Array_pub[sid]->publish(RAM_Array);
    RAM_Array.pherkulex_ram.clear();
}


class HerkuleXNode : public rclcpp::Node 
{
    public:
        HerkuleXNode() : Node("herkulex_node"), serial_port_(std::make_shared<SerialPort>()) 
        {
            this->declare_parameter<std::string>("port", "/dev/HerkuleX"); //dev/ttyUSB0
            this->declare_parameter<int>("baud_rate", 115200);
            this->declare_parameter("total_axis", rclcpp::PARAMETER_INTEGER);
            
            std::string port;
            int baud_rate;
            this->get_parameter("port", port);
            this->get_parameter("baud_rate", baud_rate);
            this->get_parameter("total_axis", m_iTotal_Axis);

            printf("[Total_Axis: %d] HerkuleX node init!\n", m_iTotal_Axis);
    
            try 
            {
                serial_port_->Open(port);
                serial_port_->SetBaudRate(BaudRate::BAUD_115200);
                serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
                serial_port_->SetStopBits(StopBits::STOP_BITS_1);
                serial_port_->SetParity(Parity::PARITY_NONE);
                serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud.", port.c_str(), baud_rate);
                printf("Serial Port Open! \n");
            } 
            catch (const OpenFailed &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
                rclcpp::shutdown();
            }
    

            //Serial_Publish/////////////////////////////////////////////////////////////////////////////////////////////
            write_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("write_data", 10);
            read_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("read_data", 10);
            /****************************************************************************************************************/
            //Publish/////////////////////////////////////////////////////////////////////////////////////////////
            string str_ram_pub;
	        string str_eep_pub;
            for(int i=0; i<m_iTotal_Axis; i++)
            {
                str_ram_pub = "Info_RAM_ID_" + to_string(i+1);
                RAM_Array_pub[i+1] = this->create_publisher<herkulex::msg::HerkulexInfoRam>(str_ram_pub, 10);

                str_eep_pub = "Info_EEP_ID_" + to_string(i+1);
                EEP_Array_pub[i+1] = this->create_publisher<herkulex::msg::HerkulexInfoEep>(str_eep_pub, 10);
                
                //error clear
                Herkulex_ErrorClear(i+1);
            }

            //servo On Loop
            for(int i=0; i<m_iTotal_Axis; i++)
            {
                //servo On
                Herkulex_Servo_Enable(i+1, 1);
            }
            
            //Subscribe///////////////////////////////////////////////////////////////////////////////////////////
            //Serial_Subscribe///////////////////////////////////////////////////////////////////////////////////////////
            write_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
                "write_data", 10, std::bind(&HerkuleXNode::writeSerialCallback, this, _1));

            //Service/////////////////////////////////////////////////////////////////////////////////////////////
            Register_service = create_service<herkulex::srv::HerkulexRegisterCommand>(
                "Register_cmd", 
            std::bind(&HerkuleXNode::Register_Command, this, std::placeholders::_1, std::placeholders::_2));

            PositionMove_service = create_service<herkulex::srv::HerkulexPositionMove>(
                "PositionMove_cmd", 
            std::bind(&HerkuleXNode::PositionMove_Command, this, std::placeholders::_1, std::placeholders::_2));

            VelocityMove_service = create_service<herkulex::srv::HerkulexVelocityMove>(
                "VelocityMove_cmd", 
            std::bind(&HerkuleXNode::VelocityMove_Command, this, std::placeholders::_1, std::placeholders::_2));

            SJOG_Move_service = create_service<herkulex::srv::HerkulexSjogMove>(
                "SjogMove_cmd", 
            std::bind(&HerkuleXNode::SJOG_Move_Command, this, std::placeholders::_1, std::placeholders::_2));
            
            IJOG_Move_service = create_service<herkulex::srv::HerkulexIjogMove>(
                "SjogMove_cmd", 
            std::bind(&HerkuleXNode::IJOG_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

            AngleMove_service = create_service<herkulex::srv::HerkulexAngleMove>(
                "AngleMove_cmd", 
            std::bind(&HerkuleXNode::AngleMove_Command, this, std::placeholders::_1, std::placeholders::_2));


            //Timer
            read_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&HerkuleXNode::readSerialCallback, this));

        }

        ///*functions///////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
        //HerkuleX Command Service call//
        bool Register_Command(const std::shared_ptr<herkulex::srv::HerkulexRegisterCommand::Request> request, 
                              const std::shared_ptr<herkulex::srv::HerkulexRegisterCommand::Response> response)
        {
            bool bResult = false;
            m_iModel = request->model_num; //Model Check
            //0:DRS-0101, 1:DRS-0102, 2:DRS-0201, 3:DRS-0401, 4:DRS-0402, 5:DRS-0601, 6:DRS-0602 

            if(request->command == "RAM_RegisterData_Read_All")
            {
                RAM_RegisterData_Read_ALL(request->id, RAM_ID, RAM_LAST);
                m_bRAM_ReadAll_Flag = true;
            }
            else if(request->command == "EEP_RegisterData_Read_All")
            {
                EEP_RegisterData_Read_ALL(request->id, EEP_MODEL_NO_1, EEP_LAST);
                m_bEEP_ReadAll_Flag = true;
            }
            else if(request->command == "RAM_RegisterData_Read")
            {
                RAM_RegisterData_Read(request->id, request->addr);
                m_bRAM_ReadAll_Flag = false;
            }
            else if(request->command == "EEP_RegisterData_Read")
            {
                EEP_RegisterData_Read(request->id, request->addr);
                m_bEEP_ReadAll_Flag = false;
            }
            else if(request->command == "RAM_RegisterData_Write")
            {
                RAM_RegisterData_Write(request->id, request->addr, request->value);
            }
            else if(request->command == "EEP_RegisterData_Write")
            {
                EEP_RegisterData_Write(request->id, request->addr, request->value);
            }
            else if(request->command == "SERVO_ON")
            {
                Herkulex_Servo_Enable(request->id, 1);
                printf("SERVO_ON Call! \n");
            }
            else if(request->command == "SERVO_OFF")
            {
                Herkulex_Servo_Enable(request->id, 0);
                printf("SERVO_OFF Call! \n");
            }
            else if(request->command == "BRAKE_ON")
            {
                Herkulex_Servo_Enable(request->id, 2);
                printf("BRAKE_ON Call! \n");
            }
            else if(request->command == "ERROR_CLEAR")
            {
                Herkulex_Servo_Enable(request->id, 0);
                printf("SERVO_OFF Call! \n");

                Herkulex_ErrorClear(request->id);
                printf("ERROR_CLEAR Call! \n");
            }

            /*
            string command 
            #0:DRS-0101, 1:DRS-0102, 2:DRS-0201, 3:DRS-0401, 4:DRS-0402, 5:DRS-0601, 6:DRS-0602 
            int8 model_num
            int8 id
            int16 addr
            int16 value
            ---
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;
        }

        bool PositionMove_Command(const std::shared_ptr<herkulex::srv::HerkulexPositionMove::Request>  request, 
                                  const std::shared_ptr<herkulex::srv::HerkulexPositionMove::Response> response)
        {
            bool bResult = false;

            Position_Move(request->id, request->led, request->playtime, request->targetposition, request->jogmode);
            response->command = "Position_Move";
            /*
            int8 id
            int8 led
            int16 playtime
            int16 targetposition
            int16 jogmode
            ---
            string command 
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;

        }

        bool VelocityMove_Command(const std::shared_ptr<herkulex::srv::HerkulexVelocityMove::Request>  request, 
                                  const std::shared_ptr<herkulex::srv::HerkulexVelocityMove::Response> response)
        {
            bool bResult = false;

            Velocity_Move(request->id, request->led, request->targetvelocity, request->jogmode);
            response->command = "Velocity_Move";
            /*
            int8 id
            int8 led
            int16 targetvelocity
            int16 jogmode
            #int16 profile
            #int16 multi
            ---
            string command 
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;
        }

        
        bool SJOG_Move_Command(const std::shared_ptr<herkulex::srv::HerkulexSjogMove::Request>  request, 
                               const std::shared_ptr<herkulex::srv::HerkulexSjogMove::Response> response)
        {
            bool bResult = false;

            for(int i=0; i<request->total_axis; i++)
            {
                sjogArr[i].Stop = 0;
                sjogArr[i].LED = request->led_arr[i];
                sjogArr[i].NoAction = 0;
                sjogArr[i].ID = request->id_arr[i];
                sjogArr[i].InfiniteTurn = 0x00; //Position Move
                sjogArr[i].Profile = 0;
                sjogArr[i].Value = request->targetposition_arr[i];
            }
        
            S_JOG_MOVE(request->playtime, request->total_axis, sjogArr);
            response->command = "SJOG_Move";
            /*
            int8[]  id_arr
            int8[]  led_arr
            int16[] targetposition_arr
            int16 playtime
            int16 total_axis
            ---
            string command 
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;
        }
        
        bool IJOG_Move_Command(const std::shared_ptr<herkulex::srv::HerkulexIjogMove::Request>  request, 
                               const std::shared_ptr<herkulex::srv::HerkulexIjogMove::Response> response)
        {
            bool bResult = false;

            for(int i=0; i<request->total_axis; i++)
            {
                ijogArr[i].Stop = 0;
                ijogArr[i].LED = request->led_arr[i];
                ijogArr[i].NoAction = 0;
                ijogArr[i].ID = request->id_arr[i];
                ijogArr[i].InfiniteTurn = 0x00; //Position Move
                ijogArr[i].Profile = 0;
                ijogArr[i].Value = request->targetposition_arr[i];
                ijogArr[i].PlayTime_ms = request->playtime[i];

            }

            I_JOG_MOVE(request->total_axis, ijogArr);
            response->command = "IJOG_Move";
            /*
            int8[]  id_arr
            int8[]  led_arr
            int16[] targetposition_arr
            int16[] playtime
            int16 total_axis
            ---
            string command 
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;
        }

        bool AngleMove_Command(const std::shared_ptr<herkulex::srv::HerkulexAngleMove::Request>  request, 
                               const std::shared_ptr<herkulex::srv::HerkulexAngleMove::Response> response)
        {
            bool bResult = false;
            int m_iTargetPosition = 0;

            if(request->unit) //degree
            {
                m_iTargetPosition = Set_DegreeToCount(request->model_num, request->target_angle);
                response->command = "Degree_Move";
            }
            else //radian
            {
                m_iTargetPosition = Set_RadianToCount(request->model_num, request->target_angle);
                response->command = "Radian_Move";
            }

            Position_Move(request->id, request->led, request->playtime, m_iTargetPosition, request->jogmode);
            
            /*
            int8 model_num  ## 0:DRS-0101, 1:DRS-0102, 2:DRS-0201, 3:DRS-0401, 4:DRS-0402, 5:DRS-0601, 6:DRS-0602
            int8 id
            int8 led
            int16 playtime
            float64 target_angle
            bool unit ## true: degree / false: radian
            int16 jogmode
            ---
            string command 
            bool command_result
            */
            bResult = true;
            response->command_result = bResult;
            return bResult;

        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        DrsData* get_packet_data(void* packet)
        {
            DrsPacket* pPacket = (DrsPacket*)packet;

            return &pPacket->unData;
        }

        unsigned char set_packet(void* buffer, unsigned char id, unsigned char cmd, int data_length)
        {
            if (buffer == 0)
                return MIN_PACKET_SIZE + data_length;

            DrsPacket* packet = (DrsPacket*)buffer;
            unsigned char i;

            //herder input
            packet->ucHeader = HEADER;
            packet->ucPacketSize = (unsigned char)(MIN_PACKET_SIZE + data_length);
            packet->ucChipID = id;
            packet->ucCmd = cmd;

            //CheckSum calc after input
            packet->ucCheckSum1 = packet->ucPacketSize ^ packet->ucChipID ^ packet->ucCmd;
            for (i = 0; i < (packet->ucPacketSize - MIN_PACKET_SIZE); i++)
                packet->ucCheckSum1 ^= packet->unData.ucData[i];

            packet->ucCheckSum2 = (unsigned char)(~(packet->ucCheckSum1));
            packet->ucCheckSum1 &= CHKSUM_MASK;
            packet->ucCheckSum2 &= CHKSUM_MASK;

            return packet->ucPacketSize;
        }

        DrsPacket* get_ack_packet(void* buffer, int size, int* pos)
        {
            int i, j;
            unsigned char check1, check2;
            unsigned char* buff = (unsigned char*)buffer;

            if (size < MIN_ACK_PACKET_SIZE)
                return 0;

            for (i = 0; i <= size - MIN_ACK_PACKET_SIZE; i++)
            {
                DrsPacket* packet = (DrsPacket*)(buff + i);
                if (packet->ucHeader != HEADER)
                    continue;
                if (packet->ucPacketSize > size - i)
                    continue;

                check1 = packet->ucPacketSize ^ packet->ucChipID ^ packet->ucCmd;
                for (j = 0; j < (packet->ucPacketSize - MIN_PACKET_SIZE); j++)
                    check1 ^= packet->unData.ucData[j];

                check2 = (unsigned char)(~(check1));
                check1 &= CHKSUM_MASK;
                check2 &= CHKSUM_MASK;
                if (packet->ucCheckSum1 != check1 || packet->ucCheckSum2 != check2)
                    continue;

                *pos = i + packet->ucPacketSize;

                return packet;
            }

            return 0;
        }

        DrsStatData* get_ack_status(DrsPacket* packet)
        {
            DrsStatData* pStat = 0;
            if (packet->ucCmd == CMD_EEP_READ_ACK || packet->ucCmd == CMD_RAM_READ_ACK)
            {
                pStat = (DrsStatData*)& packet->unData.ucData[packet->unData.stRWData.ucLen];
            }
            else
            {
                pStat = &packet->unData.stStatData;
            }

            return pStat;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        int CALLING_CONVENTION set_ram_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value)
        {
            DrsRWData* data = 0;
            unsigned char size = RAMRegSize(address);

            if (address > RAM_DESIRED_VELOCITY)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;
            memcpy(data->ucData, value, size);

            return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));
        }

        int CALLING_CONVENTION set_ram_map_write_cmd(void* buffer, unsigned char sid, RAMRegisterMap map, unsigned char address, unsigned char count)
        {
            DrsRWData* data = 0;
            unsigned char size = 0;
            unsigned char regSize = 0;
            for (unsigned char i = address; i <= RAM_LAST && count > 0; i++)
            {
                regSize = RAMRegSize(i);
                size += regSize;
                if (regSize > 0)
                    count--;
            }

            if (address > RAM_DESIRED_VELOCITY)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;
            memcpy(data->ucData, (char*)& map + address, size);

            return set_packet(buffer, sid, CMD_RAM_WRITE, (int)(sizeof(DrsRWData) + size));
        }

        int CALLING_CONVENTION set_ram_read_cmd(void * buffer, unsigned char sid, unsigned char address)
        {
            unsigned char size = RAMRegSize(address);
            DrsRWData* data = 0;

            if (address > RAM_LAST)
                return -1;
            if (buffer == 0) 
                return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;

            return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));
        }

        int CALLING_CONVENTION set_ram_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count)
        {
            DrsRWData* data = 0;
            unsigned char size = 0;
            unsigned char regSize = 0;
            for (unsigned char i = address; i <= RAM_LAST && count > 0; i++)
            {
                regSize = RAMRegSize(i);
                size += regSize;
                if (regSize > 0) count--;
            }

            if (address > RAM_LAST)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;

            return set_packet(buffer, sid, CMD_RAM_READ, sizeof(DrsRWData));
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        API int CALLING_CONVENTION get_eep_register_size(unsigned char eep_reg)
        {
            return EEPRegSize(eep_reg);
        }

        API int CALLING_CONVENTION get_ram_register_size(unsigned char ram_reg)
        {
            return RAMRegSize(ram_reg);
        }

        int CALLING_CONVENTION set_eep_write_cmd(void* buffer, unsigned char sid, unsigned char address, void* value)
        {
            DrsRWData* data = 0;
            unsigned char size = EEPRegSize(address);

            if (address > EEP_LAST)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;
            memcpy(data->ucData, value, size);

            return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));
        }

        int CALLING_CONVENTION set_eep_map_write_cmd(void* buffer, unsigned char sid, EEPRegisterMap map, unsigned char address, unsigned char count)
        {
            DrsRWData* data = 0;
            unsigned char size = 0;
            unsigned char regSize = 0;
            for (unsigned char i = address; i <= EEP_LAST && count > 0; i++)
            {
                regSize = EEPRegSize(i);
                size += regSize;
                if (regSize > 0)
                    count--;
            }

            if (address > EEP_LAST)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));

            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;
            memcpy(data->ucData, (char*)& map + address, size);

            return set_packet(buffer, sid, CMD_EEP_WRITE, (int)(sizeof(DrsRWData) + size));
        }

        int CALLING_CONVENTION set_eep_read_cmd(void* buffer, unsigned char sid, unsigned char address)
        {
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));

            if (address > EEP_CALIBRATION_DIFFERENCE)
                return -1;

            unsigned char len = 0;
            for (unsigned char i = address; i <= EEP_CALIBRATION_DIFFERENCE; i++)
            {
                len += EEPRegSize(i);
            }

            DrsRWData* data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = len;

            return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));
        }

        int CALLING_CONVENTION set_eep_map_read_cmd(void* buffer, unsigned char sid, unsigned char address, unsigned char count)
        {
            DrsRWData* data = 0;
            unsigned char size = 0;
            unsigned char regSize = 0;
            for (unsigned char i = address; i <= EEP_LAST && count > 0; i++)
            {
                regSize = EEPRegSize(i);
                size += regSize;
                if (regSize > 0) count--;
            }

            if (address > EEP_LAST)
                return -1;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));


            data = &(get_packet_data(buffer)->stRWData);
            data->ucAddress = address;
            data->ucLen = size;

            //printf("count %d, size : %d\n", count, size);
            return set_packet(buffer, sid, CMD_EEP_READ, sizeof(DrsRWData));
        }

        int CALLING_CONVENTION set_s_jog_cmd(void* buffer, unsigned char sid, unsigned char time_ms, SJog* jogs, int count)
        {
            DrsSJogData* data = 0;
            DrsSJog* sjog = 0;

            if (buffer == 0)
                return set_packet(buffer, sid, CMD_S_JOG, (int)sizeof(DrsSJogData) + (int)sizeof(DrsSJog) * count);

            data = &(get_packet_data(buffer)->stSJogData);
            sjog = data->stSJog;
            for (int i = 0; i < count; i++)
            {
                if (jogs[i].InfiniteTurn)
                {
                    sjog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
                    sjog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
                    sjog[i].stJog.Infinite.reserved = 0;
                }
                else 
                {
                    sjog[i].stJog.Position.iValue = (jogs[i].Value);
                }

                sjog[i].stSet.ucStopFlag = jogs[i].Stop;
                sjog[i].stSet.ucMode = jogs[i].InfiniteTurn;
                sjog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
                sjog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
                sjog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
                sjog[i].stSet.ucJogInvalid = jogs[i].NoAction;
                sjog[i].stSet.ucProfile = jogs[i].Profile;

                sjog[i].ucId = jogs[i].ID;
            }
            data->ucPlayTime = time_ms;
            return set_packet(buffer, sid, CMD_S_JOG, (int)(sizeof(DrsSJogData) + sizeof(DrsSJog) * (unsigned int)count));
        }

        int CALLING_CONVENTION set_i_jog_cmd(void* buffer, unsigned char sid, IJog* jogs, int count)
        {
            DrsIJog* ijog = 0;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_I_JOG, (int)sizeof(DrsIJog) * count);

            ijog = get_packet_data(buffer)->stIJog;
            for (int i = 0; i < count; i++)
            {
                if (jogs[i].InfiniteTurn)
                {
                    ijog[i].stJog.Infinite.uiValue = abs(jogs[i].Value);
                    ijog[i].stJog.Infinite.Direction = (jogs[i].Value < 0 ? 1 : 0);
                    ijog[i].stJog.Infinite.reserved = 0;
                }
                else 
                {
                    ijog[i].stJog.Position.iValue = (jogs[i].Value);
                }

                ijog[i].stSet.ucStopFlag = jogs[i].Stop;
                ijog[i].stSet.ucMode = jogs[i].InfiniteTurn;
                ijog[i].stSet.ucLedGreen = ((jogs[i].LED & LED_GREEN) == LED_GREEN);
                ijog[i].stSet.ucLedBlue = ((jogs[i].LED & LED_BLUE) == LED_BLUE);
                ijog[i].stSet.ucLedRed = ((jogs[i].LED & LED_RED) == LED_RED);
                ijog[i].stSet.ucJogInvalid = jogs[i].NoAction;
                ijog[i].stSet.ucProfile = jogs[i].Profile;

                ijog[i].ucId = jogs[i].ID;
                ijog[i].ucPlayTime = jogs[i].PlayTime_ms;
            }

            return set_packet(buffer, sid, CMD_I_JOG, (int)sizeof(DrsIJog) * count);
        }

        bool RAM_RegisterData_Read(unsigned int m_imotor_ID, unsigned int iAddr)
        {
            bool bResult = false;

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_ram_read_cmd(szSendBuffer, m_imotor_ID, iAddr);
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;

            return bResult;
        }

        bool EEP_RegisterData_Read(unsigned int m_imotor_ID, unsigned int iAddr)
        {
            bool bResult = false;

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_eep_read_cmd(szSendBuffer, m_imotor_ID, iAddr);
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;

            return bResult;
        }

        bool RAM_RegisterData_Read_ALL(unsigned int m_imotor_ID, unsigned int iAddr, unsigned char count)
        {
            bool bResult = false;

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_ram_map_read_cmd(szSendBuffer, m_imotor_ID, iAddr, count);
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;

            return bResult;
        }

        bool EEP_RegisterData_Read_ALL(unsigned int m_imotor_ID, unsigned int iAddr, unsigned char count)
        {
            bool bResult = false;

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_eep_map_read_cmd(szSendBuffer, m_imotor_ID, iAddr, count);
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;

            return bResult;
        }

        void RAM_RegisterData_Write(unsigned int m_imotor_ID, unsigned int iAddr, unsigned int cData)
        {
            memset(szSendBuffer, 0, sizeof(szSendBuffer));

            switch (iAddr)
            {
            case 0:
                RAM[m_imotor_ID - 1].ucID = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ID, &RAM[m_imotor_ID - 1].ucID);
                break;
            case 1:
                RAM[m_imotor_ID - 1].ucAckPolicy = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ACK_POLICY, &RAM[m_imotor_ID - 1].ucAckPolicy);
                break;
            case 2:
                RAM[m_imotor_ID - 1].ucAlarmLEDPolicy = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ALARM_LED_POLICY, &RAM[m_imotor_ID - 1].ucAlarmLEDPolicy);
                break;
            case 3:
                RAM[m_imotor_ID - 1].ucTorquePolicy = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TORQUE_POLICY, &RAM[m_imotor_ID - 1].ucTorquePolicy);
                break;
            case 5:
                RAM[m_imotor_ID - 1].ucMaxTemperature = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_TEMPERATURE, &RAM[m_imotor_ID - 1].ucMaxTemperature);
                break;
            case 6:
                RAM[m_imotor_ID - 1].ucMinVoltage = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_VOLTAGE, &RAM[m_imotor_ID - 1].ucMinVoltage);
                break;
            case 7:
                RAM[m_imotor_ID - 1].ucMaxVoltage = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_VOLTAGE, &RAM[m_imotor_ID - 1].ucMaxVoltage);
                break;
            case 8:
                RAM[m_imotor_ID - 1].ucAccelerationRatio = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ACCELERATION_RATIO, &RAM[m_imotor_ID - 1].ucAccelerationRatio);
                break;
            case 9:
                RAM[m_imotor_ID - 1].ucMaxAccelerationTime = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_ACCELERATION_TIME, &RAM[m_imotor_ID - 1].ucMaxAccelerationTime);
                break;
            case 10:
                RAM[m_imotor_ID - 1].ucDeadZone = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_DEAD_ZONE, &RAM[m_imotor_ID - 1].ucDeadZone);
                break;
            case 11:
                RAM[m_imotor_ID - 1].ucSaturatorOffset = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_SATURATOR_OFFSET, &RAM[m_imotor_ID - 1].ucSaturatorOffset);
                break;
            case 12:
                RAM[m_imotor_ID - 1].usSaturatorSlope = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_SATURATOR_SLOPE, &RAM[m_imotor_ID - 1].usSaturatorSlope);
                break;
            case 14:
                RAM[m_imotor_ID - 1].cPWMOffset = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_PWM_OFFSET, &RAM[m_imotor_ID - 1].cPWMOffset);
                break;
            case 15:
                RAM[m_imotor_ID - 1].ucMinPWM = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_PWM, &RAM[m_imotor_ID - 1].ucMinPWM);
                break;
            case 16:
                RAM[m_imotor_ID - 1].usMaxPWM = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_PWM, &RAM[m_imotor_ID - 1].usMaxPWM);
                break;
            case 18:
                RAM[m_imotor_ID - 1].usOverloadPWMThreshold = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_OVERLOAD_PWM_THRESHOLD, &RAM[m_imotor_ID - 1].usOverloadPWMThreshold);
                break;
            case 20:
                RAM[m_imotor_ID - 1].usMinPosition = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MIN_POSITION, &RAM[m_imotor_ID - 1].usMinPosition);
                break;
            case 22:
                RAM[m_imotor_ID - 1].usMaxPosition = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_MAX_POSITION, &RAM[m_imotor_ID - 1].usMaxPosition);
                break;
            case 24:
                RAM[m_imotor_ID - 1].usPositionKp = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KP, &RAM[m_imotor_ID - 1].usPositionKp);
                break;
            case 26:
                RAM[m_imotor_ID - 1].usPositionKd = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KD, &RAM[m_imotor_ID - 1].usPositionKd);
                break;
            case 28:
                RAM[m_imotor_ID - 1].usPositionKi = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_KI, &RAM[m_imotor_ID - 1].usPositionKi);
                break;
            case 30:
                RAM[m_imotor_ID - 1].usPositionFeedforward1stGain = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_POSITION_FEEDFORWARD_1ST_GAIN, &RAM[m_imotor_ID - 1].usPositionFeedforward1stGain);
                break;
            case 32:
                RAM[m_imotor_ID - 1].usPositionFeedforward2ndGain = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KP, &RAM[m_imotor_ID - 1].usPositionFeedforward2ndGain);
                break;
            case 34:
                RAM[m_imotor_ID - 1].usVelocityKd = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KD, &RAM[m_imotor_ID - 1].usVelocityKd);
                break;
            case 36:
                RAM[m_imotor_ID - 1].usVelocityKi = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_VELOCITY_KI, &RAM[m_imotor_ID - 1].usVelocityKi);
                break;
            case 38:
                RAM[m_imotor_ID - 1].ucLEDBlinkPeriod = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_LED_BLINK_PERIOD, &RAM[m_imotor_ID - 1].ucLEDBlinkPeriod);
                break;
            case 39:
                RAM[m_imotor_ID - 1].ucADCFaultCheckPeriod = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_ADC_FAULT_CHECK_PERIOD, &RAM[m_imotor_ID - 1].ucADCFaultCheckPeriod);
                break;
            case 40:
                RAM[m_imotor_ID - 1].ucPacketGarbageCheckPeriod = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_PACKET_GARBAGE_CHECK_PERIOD, &RAM[m_imotor_ID - 1].ucPacketGarbageCheckPeriod);
                break;
            case 41:
                RAM[m_imotor_ID - 1].ucStopDetectionPeriod = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_STOP_DETECTION_PERIOD, &RAM[m_imotor_ID - 1].ucStopDetectionPeriod);
                break;
            case 42:
                RAM[m_imotor_ID - 1].ucOverloadDetectionPeriod = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_OVELOAD_DETECTION_PERIOD, &RAM[m_imotor_ID - 1].ucOverloadDetectionPeriod);
                break;
            case 43:
                RAM[m_imotor_ID - 1].ucStopThreshold = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_STOP_THRESHOLD, &RAM[m_imotor_ID - 1].ucStopThreshold);
                break;
            case 44:
                RAM[m_imotor_ID - 1].ucInpositionMargin = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_INPOSITION_MARGIN, &RAM[m_imotor_ID - 1].ucInpositionMargin);
                break;
            case 45:
                RAM[m_imotor_ID - 1].ucTurn = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TURN, &RAM[m_imotor_ID - 1].ucTurn);
                break;
            case 46:
                RAM[m_imotor_ID - 1].cCalibrationDifference_L = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_CALIBRATION__DIFFERENCE_L, &RAM[m_imotor_ID - 1].cCalibrationDifference_L);
                break;
            case 47:
                RAM[m_imotor_ID - 1].cCalibrationDifference_H = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_CALIBRATION__DIFFERENCE_H, &RAM[m_imotor_ID - 1].cCalibrationDifference_H);
                break;
            case 52:
                RAM[m_imotor_ID - 1].ucTorqueControl = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_TORQUE_CONTROL, &RAM[m_imotor_ID - 1].ucTorqueControl);
                break;
            case 53:
                RAM[m_imotor_ID - 1].ucLEDControl = cData;
                nPacketLength = set_ram_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], RAM_LED_CONTROL, &RAM[m_imotor_ID - 1].ucLEDControl);
                break;
            default:
                printf("This is a read-only address !\n");
                break;

            }

            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);
        }


        void EEP_RegisterData_Write(unsigned int m_imotor_ID, unsigned int iAddr, unsigned int cData)
        {
            memset(szSendBuffer, 0, sizeof(szSendBuffer));

            switch (iAddr)
            {
            case 4:
                EEP[m_imotor_ID - 1].ucBaudRate = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_BAUD_RATE, &EEP[m_imotor_ID - 1].ucBaudRate);
                break;
            case 6:
                EEP[m_imotor_ID - 1].ucID = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ID, &EEP[m_imotor_ID - 1].ucID);
                break;
            case 7:
                EEP[m_imotor_ID - 1].ucAckPolicy = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ACK_POLICY, &EEP[m_imotor_ID - 1].ucAckPolicy);
                break;
            case 8:
                EEP[m_imotor_ID - 1].ucAlarmLEDPolicy = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ALARM_LED_POLICY, &EEP[m_imotor_ID - 1].ucAlarmLEDPolicy);
                break;
            case 9:
                EEP[m_imotor_ID - 1].ucTorquePolicy = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_TORQUE_POLICY, &EEP[m_imotor_ID - 1].ucTorquePolicy);
                break;
            case 11:
                EEP[m_imotor_ID - 1].ucMaxTemperature = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_TEMPERATURE, &EEP[m_imotor_ID - 1].ucMaxTemperature);
                break;
            case 12:
                EEP[m_imotor_ID - 1].ucMinVoltage = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_VOLTAGE, &EEP[m_imotor_ID - 1].ucMinVoltage);
                break;
            case 13:
                EEP[m_imotor_ID - 1].ucMaxVoltage = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_VOLTAGE, &EEP[m_imotor_ID - 1].ucMaxVoltage);
                break;
            case 14:
                EEP[m_imotor_ID - 1].ucAccelerationRatio = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ACCELERATION_RATIO, &EEP[m_imotor_ID - 1].ucAccelerationRatio);
                break;
            case 15:
                EEP[m_imotor_ID - 1].ucMaxAccelerationTime = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_ACCELERATION_TIME, &EEP[m_imotor_ID - 1].ucMaxAccelerationTime);
                break;
            case 16:
                EEP[m_imotor_ID - 1].ucDeadZone = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_DEAD_ZONE, &EEP[m_imotor_ID - 1].ucDeadZone);
                break;
            case 17:
                EEP[m_imotor_ID - 1].ucSaturatorOffset = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_SATURATOR_OFFSET, &EEP[m_imotor_ID - 1].ucSaturatorOffset);
                break;
            case 18:
                EEP[m_imotor_ID - 1].usSaturatorSlope = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_SATURATOR_SLOPE, &EEP[m_imotor_ID - 1].usSaturatorSlope);
                break;
            case 20:
                EEP[m_imotor_ID - 1].cPWMOffset = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_PWM_OFFSET, &EEP[m_imotor_ID - 1].cPWMOffset);
                break;
            case 21:
                EEP[m_imotor_ID - 1].ucMinPWM = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_PWM, &EEP[m_imotor_ID - 1].ucMinPWM);
                break;
            case 22:
                EEP[m_imotor_ID - 1].usMaxPWM = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_PWM, &EEP[m_imotor_ID - 1].usMaxPWM);
                break;
            case 24:
                EEP[m_imotor_ID - 1].usOverloadPWMThreshold = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_OVERLOAD_PWM_THRESHOLD, &EEP[m_imotor_ID - 1].usOverloadPWMThreshold);
                break;
            case 26:
                EEP[m_imotor_ID - 1].usMinPosition = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MIN_POSITION, &EEP[m_imotor_ID - 1].usMinPosition);
                break;
            case 28:
                EEP[m_imotor_ID - 1].usMaxPosition = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_MAX_POSITION, &EEP[m_imotor_ID - 1].usMaxPosition);
                break;
            case 30:
                EEP[m_imotor_ID - 1].usPositionKp = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KP, &EEP[m_imotor_ID - 1].usPositionKp);
                break;
            case 32:
                EEP[m_imotor_ID - 1].usPositionKd = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KD, &EEP[m_imotor_ID - 1].usPositionKd);
                break;
            case 34:
                EEP[m_imotor_ID - 1].usPositionKi = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_KI, &EEP[m_imotor_ID - 1].usPositionKi);
                break;
            case 36:
                EEP[m_imotor_ID - 1].usPositionFeedforward1stGain = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_POSITION_FEEDFORWARD_1ST_GAIN, &EEP[m_imotor_ID - 1].usPositionFeedforward1stGain);
                break;
            case 38:
                EEP[m_imotor_ID - 1].usPositionFeedforward2ndGain = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KP, &EEP[m_imotor_ID - 1].usPositionFeedforward2ndGain); //->usVelocityKp사용
                break;
            case 40:
                EEP[m_imotor_ID - 1].usVelocityKd = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KD, &EEP[m_imotor_ID - 1].usVelocityKd);
                break;
            case 42:
                EEP[m_imotor_ID - 1].usVelocityKi = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_VELOCITY_KI, &EEP[m_imotor_ID - 1].usVelocityKi);
                break;
            case 44:
                EEP[m_imotor_ID - 1].ucLEDBlinkPeriod = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_LED_BLINK_PERIOD, &EEP[m_imotor_ID - 1].ucLEDBlinkPeriod);
                break;
            case 45:
                EEP[m_imotor_ID - 1].ucADCFaultCheckPeriod = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_ADC_FAULT_CHECK_PERIOD, &EEP[m_imotor_ID - 1].ucADCFaultCheckPeriod);
                break;
            case 46:
                EEP[m_imotor_ID - 1].ucPacketGarbageCheckPeriod = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_PACKET_GARBAGE_CHECK_PERIOD, &EEP[m_imotor_ID - 1].ucPacketGarbageCheckPeriod);
                break;
            case 47:
                EEP[m_imotor_ID - 1].ucStopDetectionPeriod = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_STOP_DETECTION_PERIOD, &EEP[m_imotor_ID - 1].ucStopDetectionPeriod);
                break;
            case 48:
                EEP[m_imotor_ID - 1].ucOverloadDetectionPeriod = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_OVELOAD_DETECTION_PERIOD, &EEP[m_imotor_ID - 1].ucOverloadDetectionPeriod);
                break;
            case 49:
                EEP[m_imotor_ID - 1].ucStopThreshold = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_STOP_THRESHOLD, &EEP[m_imotor_ID - 1].ucStopThreshold);
                break;
            case 50:
                EEP[m_imotor_ID - 1].ucInpositionMargin = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_INPOSITION_MARGIN, &EEP[m_imotor_ID - 1].ucInpositionMargin);
                break;
            case 51:
                EEP[m_imotor_ID - 1].ucReserved5 = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_RESERVED_5, &EEP[m_imotor_ID - 1].ucReserved5);
                break;
            case 52:
                EEP[m_imotor_ID - 1].cCalibrationDifference_L = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_RESERVED_6, &EEP[m_imotor_ID - 1].cCalibrationDifference_L);
                break;
            case 53:
                EEP[m_imotor_ID - 1].cCalibrationDifference_H = cData;
                nPacketLength = set_eep_write_cmd(szSendBuffer, szIDs[m_imotor_ID - 1], EEP_CALIBRATION_DIFFERENCE, &EEP[m_imotor_ID - 1].cCalibrationDifference_H);
                break;

            default:
                printf("This is a read-only address !\n");
                break;

            }

            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);
        }


        int CALLING_CONVENTION set_stat_cmd(void* buffer, unsigned char sid)
        {
            DrsData* data = 0;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_STAT, 0);

            data = get_packet_data(buffer);
            data->stStatData.ucError = 0;
            data->stStatData.ucDetail = 0;
            return set_packet(buffer, sid, CMD_STAT, 0);
        }

        int CALLING_CONVENTION set_rollback_cmd(void* buffer, unsigned char sid, unsigned char id_skip, unsigned char baud_skip)
        {
            DrsData* data = 0;
            if (buffer == 0)
                return set_packet(buffer, sid, CMD_ROLLBACK, sizeof(DrsRollbackData));

            data = get_packet_data(buffer);
            data->stRollbackData.ucIdSkip = id_skip;
            data->stRollbackData.ucBaudSkip = baud_skip;
            return set_packet(buffer, sid, CMD_ROLLBACK, sizeof(DrsRollbackData));
        }

        int CALLING_CONVENTION set_reboot_cmd(void* buffer, unsigned char sid)
        {
            return set_packet(buffer, sid, CMD_REBOOT, 0);
        }

        int CALLING_CONVENTION set_calset_cmd(void* buffer, unsigned char sid)
        {
            return set_packet(buffer, sid, CMD_CALSET, 0);
        }

        /**********************************************************************************************************************************/

        void proc_eep_read_ack(DrsPacket* packet)
        {
            EEPRegisterMap _eepMap;

            if (_eep_read_ack_callback != 0)
                (*_eep_read_ack_callback)(
                    packet->ucChipID,
                    packet->unData.stRWData.ucAddress,
                    packet->unData.stRWData.ucLen,
                    packet->unData.stRWData.ucData,
                    get_ack_status(packet)->ucError,
                    get_ack_status(packet)->ucDetail
                    );

            if (_eep_map_read_ack_callback != 0)
            {
                memcpy((void*)((char*)& _eepMap + packet->unData.stRWData.ucAddress), packet->unData.stRWData.ucData, packet->unData.stRWData.ucLen);
                (*_eep_map_read_ack_callback)(
                    packet->ucChipID,
                    packet->unData.stRWData.ucAddress,
                    packet->unData.stRWData.ucLen,
                    _eepMap,
                    get_ack_status(packet)->ucError,
                    get_ack_status(packet)->ucDetail
                    );

            }
        }

        void proc_ram_read_ack(DrsPacket* packet)
        {
            RAMRegisterMap _ramMap;

            if (_ram_read_ack_callback != 0)
                (*_ram_read_ack_callback)(
                    packet->ucChipID,
                    packet->unData.stRWData.ucAddress,
                    packet->unData.stRWData.ucLen,
                    packet->unData.stRWData.ucData,
                    get_ack_status(packet)->ucError,
                    get_ack_status(packet)->ucDetail
                    );
            if (_ram_map_read_ack_callback != 0)
            {
                memcpy((void*)((char*)& _ramMap + packet->unData.stRWData.ucAddress), packet->unData.stRWData.ucData, packet->unData.stRWData.ucLen);
                (*_ram_map_read_ack_callback)(
                    packet->ucChipID,
                    packet->unData.stRWData.ucAddress,
                    packet->unData.stRWData.ucLen,
                    _ramMap,
                    get_ack_status(packet)->ucError,
                    get_ack_status(packet)->ucDetail
                    );
            }
        }


        int CALLING_CONVENTION parse(void* buffer, int length, int* pos)
        {
            DrsPacket* packet = get_ack_packet(buffer, length, pos);
            if (packet == 0)
                return 0;


            switch (packet->ucCmd)
            {
            case CMD_EEP_WRITE_ACK:
                if (_eep_write_ack_callback != 0)
                    (*_eep_write_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_EEP_READ_ACK:
                proc_eep_read_ack(packet);
                break;
            case CMD_RAM_WRITE_ACK:
                if (_ram_write_ack_callback != 0)
                    (*_ram_write_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_RAM_READ_ACK:
                proc_ram_read_ack(packet);
                break;
            case CMD_I_JOG_ACK:
                if (_i_jog_ack_callback != 0)
                    (*_i_jog_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_S_JOG_ACK:
                if (_s_jog_ack_callback != 0)
                    (*_s_jog_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_STAT_ACK:
                if (_stat_ack_callback != 0)
                    (*_stat_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_ROLLBACK_ACK:
                if (_rollback_ack_callback != 0)
                    (*_rollback_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            case CMD_REBOOT_ACK:
                if (_reboot_ack_callback != 0)
                    (*_reboot_ack_callback)(
                        packet->ucChipID,
                        get_ack_status(packet)->ucError,
                        get_ack_status(packet)->ucDetail
                        );
                break;
            default:
                return 0;
            }

            return 1;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Move Command//
        bool Herkulex_ErrorClear(unsigned char cID)
        {
            bool bResult = false;

            // Error Clear
            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_STATUS_ERROR, &ucValue);
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            // Error detail Clear
            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_STATUS_DETAIL, &ucValue);
            LibSerial::DataBuffer dataBuffer2(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer2);

            bResult = true;
            return bResult;
        }

        bool Herkulex_Servo_Enable(unsigned char cID, unsigned int uiMode)
        {
            bool bResult = false;

            memset(szSendBuffer, 0, sizeof(szSendBuffer));

            switch (uiMode)
            {
            case 0:
                ucValue = TORQUE_CONTROL_FREE;
                nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
                bResult = true;
                break;
            case 1:
                ucValue = TORQUE_CONTROL_TORQUEON;
                nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
                bResult = true;
                break;
            case 2:
                ucValue = TORQUE_CONTROL_BRAKEON;
                nPacketLength = set_ram_write_cmd(szSendBuffer, cID, RAM_TORQUE_CONTROL, &ucValue);
                bResult = true;
                break;
            }

            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            return bResult;
        }

        bool Position_Move(unsigned char cID, unsigned int iLED, unsigned int iPlayTime, int iTargetPos, int iJogMode/*int iprofile, bool bMulti, int turn*/)
        {
            bool bResult = false;

            switch (iJogMode)
            {
            case 0: //S_JOG
                // S_JOG 값 초기화
                sjogs[cID - 1].Value = 0;
                sjogs[cID - 1].Stop = 0;
                sjogs[cID - 1].LED = iLED;
                sjogs[cID - 1].NoAction = 0;
                sjogs[cID - 1].ID = szIDs[cID - 1];
                sjogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
                sjogs[cID - 1].Profile = 0; //iprofile; //default:0

                //Target Command -> Count
                sjogs[cID - 1].Value = iTargetPos;

                memset(szSendBuffer, 0, sizeof(szSendBuffer));
                nPacketLength = set_s_jog_cmd(szSendBuffer, cID, iPlayTime, &sjogs[cID - 1], 1);
                break;
            case 1: //I_JOG
                // I_JOG 값 초기화
                ijogs[cID - 1].Value = 0;
                ijogs[cID - 1].Stop = 0;
                ijogs[cID - 1].LED = iLED;
                ijogs[cID - 1].NoAction = 0;
                ijogs[cID - 1].ID = szIDs[cID - 1];
                ijogs[cID - 1].InfiniteTurn = 0x00;//Position Mode
                ijogs[cID - 1].Profile = 0; //iprofile; //default:0

                //Target Command -> Count
                ijogs[cID - 1].Value = iTargetPos;

                //Play time//
                ijogs[cID - 1].PlayTime_ms = iPlayTime;

                memset(szSendBuffer, 0, sizeof(szSendBuffer));
                nPacketLength = set_i_jog_cmd(szSendBuffer, cID, &ijogs[cID - 1], 1);
                break;
            }

            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;
            return bResult;
        }

        bool Velocity_Move(unsigned char cID, unsigned int iLED, int iTargetVel, int iJogMode/*,int iprofile, bool bMulti*/)
        {
            bool bResult = false;

            switch (iJogMode)
            {
            case 0: //S_JOG
                // S_JOG 값 초기화
                sjogs[cID - 1].Value = 0;
                sjogs[cID - 1].Stop = 0;
                sjogs[cID - 1].LED = iLED;
                sjogs[cID - 1].NoAction = 0;
                sjogs[cID - 1].ID = szIDs[cID - 1];
                sjogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
                sjogs[cID - 1].Profile = 0; //iprofile; //default:0

                //Target Command -> Count
                sjogs[cID - 1].Value = iTargetVel;

                memset(szSendBuffer, 0, sizeof(szSendBuffer));
                nPacketLength = set_s_jog_cmd(szSendBuffer, cID, 10, &sjogs[cID - 1], 1);
                break;
            case 1: //I_JOG
                // I_JOG 값 초기화
                ijogs[cID - 1].Value = 0;
                ijogs[cID - 1].Stop = 0;
                ijogs[cID - 1].LED = iLED;
                ijogs[cID - 1].NoAction = 0;
                ijogs[cID - 1].ID = szIDs[cID - 1];
                ijogs[cID - 1].InfiniteTurn = 0x01;//Velocity Mode
                ijogs[cID - 1].Profile = 0; //iprofile; //default:0

                //Target Command -> Count
                ijogs[cID - 1].Value = iTargetVel;

                memset(szSendBuffer, 0, sizeof(szSendBuffer));
                nPacketLength = set_i_jog_cmd(szSendBuffer, cID, &ijogs[cID - 1], 1);
                break;
            
            }
            
            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;
            return bResult;
        }

        bool S_JOG_MOVE(unsigned int iPlayTime, unsigned int iTotal_Axis, CMD_SJog * sjogArr)
        {
            bool bResult = false;

            for (int i = 0; i < iTotal_Axis; i++)
            {
                sjogs[i].Stop = sjogArr[i].Stop;
                sjogs[i].LED = sjogArr[i].LED;
                sjogs[i].NoAction = sjogArr[i].NoAction;
                sjogs[i].ID = sjogArr[i].ID; //szIDs[i];
                sjogs[i].InfiniteTurn = sjogArr[i].InfiniteTurn;
                sjogs[i].Profile = sjogArr[i].Profile;
                //Target Command -> Count
                sjogs[i].Value = sjogArr[i].Value;
            }

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_s_jog_cmd(szSendBuffer, 0xFE, iPlayTime, sjogs, iTotal_Axis);

            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;
            return bResult;
        }

        bool I_JOG_MOVE(unsigned int iTotal_Axis, CMD_IJog * ijogArr)
        {
            bool bResult = false;
            
            for (int i = 0; i < iTotal_Axis; i++)
            {
                ijogs[i].Stop = ijogArr[i].Stop;
                ijogs[i].LED = ijogArr[i].LED;
                ijogs[i].NoAction = ijogArr[i].NoAction;
                ijogs[i].ID = ijogArr[i].ID;
                ijogs[i].InfiniteTurn = ijogArr[i].InfiniteTurn;
                ijogs[i].Profile = ijogArr[i].Profile;
                //Target Command -> Count
                ijogs[i].Value = ijogArr[i].Value;
                //PlayTime_ms
                ijogs[i].PlayTime_ms = ijogArr[i].PlayTime_ms;
            }

            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_i_jog_cmd(szSendBuffer, 0xFE, ijogs, iTotal_Axis);
            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

            bResult = true;
            return bResult;
        }

        /*********************************************************************************************************************************/
        void FactoryReset(unsigned char cID, int iID_Skip, int iBaudrate_Skip)
        {
            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_rollback_cmd(szSendBuffer, cID, iID_Skip, iBaudrate_Skip);
            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

        }

        void Reboot(unsigned char cID)
        {
            memset(szSendBuffer, 0, sizeof(szSendBuffer));
            nPacketLength = set_reboot_cmd(szSendBuffer, cID);
            //Send//
            LibSerial::DataBuffer dataBuffer(szSendBuffer, szSendBuffer + nPacketLength);
            serial_port_->Write(dataBuffer);

        }

        void SerialPortClose()
        {
            serial_port_->Close();
        }

        //add function/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        int Set_DegreeToCount(int iModel_num, double dTarget_deg)
        {
            int m_iTarget_count = 0;
            switch (iModel_num)
			{
				case 0:
                    m_iTarget_count = (int)(dTarget_deg / 0.325) + 512; //0101
					break;
				case 1:
                    m_iTarget_count = (int)(dTarget_deg / 0.02778) + 16384; //0102
					break;
				case 2:
                    m_iTarget_count = (int)(dTarget_deg / 0.325) + 512; //0201
					break;
				case 3:
                    m_iTarget_count = (int)(dTarget_deg / 0.163) + 1024; //0401
					break;
				case 4:
                    m_iTarget_count = (int)(dTarget_deg / 0.02778) + 16384; //0402
					break;
				case 5:
                    m_iTarget_count = (int)(dTarget_deg / 0.163) + 1024; //0601
					break;
				case 6:
                    m_iTarget_count = (int)(dTarget_deg / 0.02778) + 16384; //0602
					break;
			}

            return m_iTarget_count;
        }

        int Set_RadianToCount(int iModel_num, double dTarget_rad)
        {
            int m_iTarget_count = 0;
            double m_dTarget_deg = (dTarget_rad * 180.0)/M_PI;
            switch (iModel_num)
			{
				case 0:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0101
					break;
				case 1:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0102
					break;
				case 2:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.325) + 512; //0201
					break;
				case 3:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0401
					break;
				case 4:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0402
					break;
				case 5:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.163) + 1024; //0601
					break;
				case 6:
                    m_iTarget_count = (int)(m_dTarget_deg / 0.02778) + 16384; //0602
					break;
			}

            return m_iTarget_count;
        }

        double Get_CountToDegree(int iModel_num, int iTarget_count)
        {
            double dTarget_deg = 0.0;
            //Count -> Degree
            switch (iModel_num)
			{
				case 0:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0101
					break;
				case 1:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0102
					break;
				case 2:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0201
					break;
				case 3:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0401
					break;
				case 4:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0402
					break;
				case 5:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0601
					break;
				case 6:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0602
					break;
			}

            return dTarget_deg;
        }
        

        double Get_CountToRadian(int iModel_num, int iTarget_count)
        {
            double dTarget_deg = 0.0;
            double dTarget_rad = 0.0;
            //Count -> Degree
            switch (iModel_num)
			{
				case 0:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0101
					break;
				case 1:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0102
					break;
				case 2:
                    dTarget_deg = (iTarget_count - 512.0) * 0.325; //0201
					break;
				case 3:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0401
					break;
				case 4:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0402
					break;
				case 5:
                    dTarget_deg = (iTarget_count - 1024.0) * 0.163; //0601
					break;
				case 6:
                    dTarget_deg = (iTarget_count - 16384.0) * 0.02778; //0602
					break;
			}

            //Degree -> Radian
            dTarget_rad = (dTarget_deg * M_PI)/180.0;

            return dTarget_rad;
        }


    
    private:
        void writeSerialCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) 
        {
            if (serial_port_->IsOpen()) 
            {
                std::string data_to_write(msg->data.begin(), msg->data.end());  // Convert to std::string
                try 
                {
                    serial_port_->Write(data_to_write);
                    //RCLCPP_INFO(this->get_logger(), "Data written to serial.");
                } 
                catch (const std::exception &e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Write failed: %s", e.what());
                }
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open.");
            }
        }

        void readSerialCallback() 
        {
            if (serial_port_->IsDataAvailable()) 
            {
                std_msgs::msg::UInt8MultiArray serial_data;
                m_idata_size = serial_port_->GetNumberOfBytesAvailable();
                //RCLCPP_INFO(this->get_logger(), "m_idata_size: %d", m_idata_size);

                serial_port_->Read(serial_data.data, m_idata_size, 1000);
                
                r_buffer.data.resize(m_idata_size);
                for(int i=0; i<m_idata_size; i++)
                {
                    r_buffer.data[i] = serial_data.data[i];
                    //RCLCPP_INFO(this->get_logger(), "Read[%d]: %02x", i, r_buffer.data[i]);
                }

                int pos = 0;
                unsigned char* buffer = new unsigned char[m_idata_size];
                std::copy(r_buffer.data.begin(), r_buffer.data.end(), buffer);
                parse(buffer, m_idata_size, &pos);
                delete[ ] buffer;

                read_pub_->publish(r_buffer);
                //RCLCPP_INFO(this->get_logger(), "Data read from serial and published.");

            }
        }

        //*serial///////////////////////////////////////////////////////////////////////*/
        std::shared_ptr<SerialPort> serial_port_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr write_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr read_pub_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr write_sub_;
        rclcpp::TimerBase::SharedPtr read_timer_;

        ////Service/////////////////////////////////////////////////////////////////////////////////
	    rclcpp::Service<herkulex::srv::HerkulexRegisterCommand>::SharedPtr Register_service;
        rclcpp::Service<herkulex::srv::HerkulexPositionMove>::SharedPtr PositionMove_service;
        rclcpp::Service<herkulex::srv::HerkulexVelocityMove>::SharedPtr VelocityMove_service;
        rclcpp::Service<herkulex::srv::HerkulexSjogMove>::SharedPtr SJOG_Move_service;
        rclcpp::Service<herkulex::srv::HerkulexIjogMove>::SharedPtr IJOG_Move_service;
        rclcpp::Service<herkulex::srv::HerkulexAngleMove>::SharedPtr AngleMove_service;


};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);

	std::signal(SIGINT, signal_handler);
	// Create a function for when messages are to be sent.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    auto node = std::make_shared<HerkuleXNode>();

    //init_memset//
	memset(RAM, 0, sizeof(RAM));
	memset(EEP, 0, sizeof(EEP));	
	memset(sjogs, 0, sizeof(sjogs));
	memset(ijogs, 0, sizeof(ijogs));
	memset(sjogArr, 0, sizeof(sjogArr));
	memset(ijogArr, 0, sizeof(ijogArr));

	////Callback init/////////
	regist_ack_callback_eep_write(EEPWriteAckCallback);
	regist_ack_callback_eep_read(EEPReadAckCallback);
	regist_ack_callback_ram_write(RAMWriteAckCallback);
	regist_ack_callback_ram_read(RAMReadAckCallback);
	regist_ack_callback_ram_map_read(RAMMapReadAckCallback);
	regist_ack_callback_eep_map_read(EEPMapReadAckCallback);
	regist_ack_callback_i_jog(IJogAckCallback);
	regist_ack_callback_s_jog(SJogAckCallback);
	regist_ack_callback_stat(StatAckCallback);
	regist_ack_callback_rollback(RollbackAckCallback);
	regist_ack_callback_reboot(RebootAckCallback);

    rclcpp::spin(node);

    node->SerialPortClose();
    
    rclcpp::shutdown();
    return 0;
}
