#ifndef CMD_SET_H
#define CMD_SET_H

//************************************************************************************
//*******************     Actuator's ID commands    **********************************
//************************************************************************************
#define C_SET_ID                0xAAU  // Load Actuator ID command
#define R_SET_ID                0x55U  // response code

#define C_ID_REPORT             0xDAU  // report Station ID command
#define R_ID_REPORT             0x6DU  // response code

//************************************************************************************
//*******************         Actuator's position commands        ********************
//************************************************************************************
#define C_SET_POS100            0xDDU  // servo position command for ±100° area
#define R_SET_POS100            0x44U  // response code

#define C_Set_Pos170            0x76U  // servo position command for ±170° area
#define R_Set_Pos170            0x56U  // response code

#define C_Set_Pos170_Silent     0x77U  // servo new position (±170°) command without response

#define C_Act_Pos100_report     0x92U  // report Actual Position (±100°)
#define R_Act_Pos100_report     0x62U  // response code

#define C_Act_Pos170_report     0x69U  // report of actual position (±170°)
#define R_Act_Pos170_report     0x49U  // response code

//#define C_Set_Scaled            0x78U  // servo position command of ±170° area
//#define R_Set_Scaled            0x58U  // response code

//#define C_Act_Pos_Scaled        0x79U  // report of actual position (±170°) command
//#define R_Act_Pos_Scaled        0x59U  // response code

#define C_Set_PosExt            0xDCU  // servo New Position Extended command
#define R_Set_PosExt            0x2CU  // response code

#define C_Act_PosExt_report     0x97U  // report Actual Position Extended command
#define R_Act_PosExt_report     0x67U  // response code

#define C_Failsafe_Pos          0xBBU  // set failsafe position command
#define R_Failsafe_Pos          0x5DU  // response code

#define C_Fs_Pos_report         0x90U  // report failsafe position command
#define R_Fs_Pos_report         0x60U  // response code

#define C_Failsafe_Time         0xCCU  // set failsafe timeout command
#define R_Failsafe_Time         0x66U  // response code

#define C_Fs_Time_report        0x91U  // report failsafe timeout command
#define R_Fs_Time_report        0x61U  // response code

#define C_Set_as_Failsafe       0xBCU  // set current position as new failsafe position command
#define R_Set_as_Failsafe       0x5CU  // response code

#define C_Set_Notch_Freq        0xBFU  // set notch filter frequency, Bit 15: freq1/freq2, 1/100 rad/sec
#define R_Set_Notch_Freq        0x5FU  // response code

//************************************************************************************
//*******************   Other actuator's commands  ***********************************
//************************************************************************************
#define C_SetDefault            0xB4U  // restore Factory Defaults command: cmd ID 41 53
#define R_SetDefault            0x5AU  // response code

#define C_ACE2_as_Master        0x33U  // Set ACE2 (Standby) as master command:  cmd ID AA 55
#define R_ACE2_as_Master        0x34U  // Set ACE2 (Standby) as master response: res ID 55 AA

#define C_ACE1_as_Master        0x35U  // Set ACE1 as a master command:  cmd ID AA 55
#define R_ACE1_as_Master        0x36U  // Set ACE1 as a master response: res ID 55 AA

#define C_Read_Role             0x37U  // Read ACE's role: cmd ID 00 00, read freshness error cmd ID 00 01, reset freshness error cmd ID 00 02
#define R_Read_Role             0x38U  // Read ACE's role response res ID 00 00 (0-slave, 1-master)

#define C_Set_as_Zero           0x99U  // Set current Position as zero command
#define R_Set_as_Zero           0x4CU  // response code

#define C_RS_MODEreport         0xD3U  // report Interface Mode command
#define R_RS_MODEreport         0x83U  // response code

#define C_RS_MODEreport_V       0x93U  // report Interface Mode command
#define R_RS_MODEreport_V       0x63U  // response code

#define C_Stroke_Len_report     0xD4U  // report Stroke Length command
#define R_Stroke_Len_report     0x84U  // response code

#define C_Stroke_Len_report_V   0x94U  // report Stroke Length command
#define R_Stroke_Len_report_V   0x64U  // response code

#define SIGN_SetDefault1        0x41U  // signature bits for SetDefault command
#define SIGN_SetDefault2        0x53U

#define C_Motor_Power           0x9AU  // Motor power command (power limit if 0x00FF, 0x8000-power in EEPROM, 0x8001-motor brake, 0x8002-motor report)
#define R_Motor_Power           0x4AU  // response code

#define C_SimulateACEfailure    0xF5U  // ACE failure simulation
#define R_SimulateACEfailure    0x15U  // response code

//************************************************************************************
//*******************   Actuator's control values commands  **************************
//************************************************************************************
#define C_Read_Current          0xB0U  // readout of current Consumption command
#define R_Read_Current          0x30U  // response code

#define C_Read_Voltage          0xB1U  // readout of Input Voltages command
#define R_Read_Voltage          0x31U  // response code

#define C_Read_Temperature      0xA0U  // readout of Temperatures command
#define R_Read_Temperature      0x20U  // response code

#define C_Read_Temperature_V    0xC0U  // Readout of Temperatures command
#define R_Read_Temperature_V    0x10U  // response code

#define C_Read_Humidity         0xA1U  // readout of Humidity command
#define R_Read_Humidity         0x21U  // response code

#define C_Read_Humidity_V       0xC1U  // readout of Humidity command
#define R_Read_Humidity_V       0x11U  // response code

#define C_Read_Magnet           0xE5U  // readout of magnet sensor data cmd ID 00 00 : raw, cmd ID 00 01 : raw - factory setting "0"
#define R_Read_Magnet           0x45U  // response code

#define C_Read_PWM              0xB2U  // readout of BLDC motor PWM
#define R_Read_PWM              0x32U  // response code

#define C_Read_Offset           0xD5U  // readout of position offset
#define R_Read_Offset           0x85U  // response code

#define C_Write_Offset          0xFEU  // set position offset
#define R_Write_Offset          0xFEU  // response code

#define C_Set_as_Zero           0x99U  // Set current Position as position offset
#define R_Set_as_Zero           0x4CU  // response code

//************************************************************************************
//*******************     EEPROM access commands   ***********************************
//************************************************************************************
#define C_Read_MP               0xE7U  // address range 0x20...0x27 e.g.: E7 1F 24 24 - read EEPROM cell 0x24 (Maximal Power)
#define R_Read_MP               0x47U  // response code

#define C_Write_MP              0xE8U  // address range 0x20...0x2F e.g.: E8 1F 24 FF - write 0xFF to EEPROM cell 0x24 (Maximal Power)
#define R_Write_MP              0x48U  // response code

#define C_RD_EEPROM_Lo          0xF8U  // read any low 0x00...0xFF EEPROM cell
#define R_RD_EEPROM_Lo          0xF8U  // response code

#define C_WR_EEPROM_Lo          0xF9U  // write any low 0x00...0xFF EEPROM cell
#define R_WR_EEPROM_Lo          0xF9U  // response code

#define C_RD_EEPROM_Hi          0xE9U  // "E9 1F 80 80 - read EEPROM cell 0x180" (High part of EEPROM 100..1FF)
#define R_RD_EEPROM_Hi          0x49U  // response code

#define C_WR_EEPROM_Hi          0xFAU  // write any high 0x0100...0x01FF EEPROM cell (command needs an access bit)
#define R_WR_EEPROM_Hi          0xFAU  // response code

#define C_Read_Serial_Num       0xF0U  // Readout of Electronic Serial Number
#define R_Read_Serial_Num       0x10U  // response code

#define C_Read_Product_Descr    0xF1U  // Readout of Product Description
#define R_Read_Product_Descr    0x11U  // response code

#define C_Read_Firmware_Num     0xF2U  // Readout of Software Revision Number
#define R_Read_Firmware_Num     0x12U  // response code

#define C_Read_Hardware_Num     0xF3U  // Readout of Hardware Revision Number
#define R_Read_Hardware_Num     0x13U  // response code

#define C_Read_Revision_Str     0xF4U  // Readout of Revision string (JSON)
#define R_Read_Revision_Str     0x14U  // response code

#define C_Read_Serial_Num_V     0xE0U  // Readout of Electronic Serial Number command
#define R_Read_Serial_Num_V     0x40U  // response code

#define C_Read_Product_Descr_V  0xE1U  // Readout of Product Description command
#define R_Read_Product_Descr_V  0x41U  // response code

#define C_Read_Firmware_Num_V   0xE2U  // Readout of Software Revision Number command
#define R_Read_Firmware_Num_V   0x42U  // response code

#define C_Read_Hardware_Num_V   0xE3U  // Readout of Hardware Revision Number command
#define R_Read_Hardware_Num_V   0x43U  // response code

#define C_Write_Serial_Num      0xD6U  // write Electronic Serial Number (data, place)
#define R_Write_Serial_Num      0x86U  // response code

#define C_Write_Prod_Descr      0xD7U  // write Product descriptor (data, place)
#define R_Write_Prod_Descr      0x87U  // response code

#define C_Write_FWrev_num       0xD8U  // write Firmware revision (data, place)
#define R_Write_FWrev_num       0x88U  // response code

#define C_Write_HWrev_num       0xD9U  // write Hardware revision (data, place)
#define R_Write_HWrev_num       0x89U  // response code

#define C_Set_AccessBit         0xFFU  // set EEPROM access bit for the next EEPROM operation

//************************************************************************************
//*******************   Actuator's timers commands  **********************************
//************************************************************************************
#define C_No_Load_Runtimer      0xA3U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_No_Load_Runtimer      0x23U  // response code

#define C_Load_Runtimer_25      0xA4U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_25      0x24U  // response code

#define C_Load_Runtimer_50      0xA5U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_50      0x25U  // response code

#define C_Load_Runtimer_75      0xA6U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_75      0x26U  // response code

#define C_Load_Runtimer_100     0xA7U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_100     0x27U  // response code

#define C_Stall_Counter         0xA8U  // command (00 00 - max.65536H stall events; AA 55 - reset)
#define R_Stall_Counter         0x28U  // response code

#define C_PowerUp_Counter       0xA9U  // Readout of number of Power Up Cycles (max.65536H)
#define R_PowerUp_Counter       0x29U  // response code

#define C_Runtimer              0xA2U  // lifetime command (cmd 01 00 00 - hours(max.65536H), A2 01 00 01 - min, sec)
#define R_Runtimer              0x22U  // response code

#define C_No_Load_Runtimer_V    0xC3U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_No_Load_Runtimer_V    0x13U  // response code

#define C_Load_Runtimer_25_V    0xC4U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_25_V    0x14U  // response code

#define C_Load_Runtimer_50_V    0xC5U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_50_V    0x15U  // response code

#define C_Load_Runtimer_75_V    0xC6U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_75_V    0x16U  // response code

#define C_Load_Runtimer_100_V   0xC7U  // command (00 00 - hours(max.65536H); 00 01 - min, sec; AA 55 - reset)
#define R_Load_Runtimer_100_V   0x17U  // response code

#define C_Stall_Counter_V       0xC8U  // command (00 00 - max.65536H stall events; AA 55 - reset)
#define R_Stall_Counter_V       0x18U  // response code

#define C_PowerUp_Counter_V     0xC9U  // Readout of number of Power Up Cycles (max.65536H)
#define R_PowerUp_Counter_V     0x19U  // response code

#define C_Runtimer_V            0xC2U  // lifetime command (cmd 01 00 00 - hours(max.65536H), A2 01 00 01 - min, sec)
#define R_Runtimer_V            0x12U  // response code

//************************************************************************************
//*******************   Other commands  **********************************************
//************************************************************************************
#define C_Motor_Power           0x9AU  // Motor power command (power limit if 0x00FF, 0x8000-power in EEPROM, 0x8001-motor brake, 0x8002-motor report)
#define R_Motor_Power           0x4AU  // response code

#define C_Slave_Free            0x9EU  // Set slave as free-running
#define R_Slave_Free            0x4EU  // response code

#define C_Misc                  0xDBU  // readout miscellaneous parameters (e.g. CRCs) and tests
#define R_Misc                  0x8BU  // response code

#define C_StatusRW              0xB5U  // Cmd ID 00 xx read status, Cmd ID 01 nn - write status nn (internal g_my_status & g_partner_status)

//#define C_Misc_AppCRC           0x11U
//#define C_Misc_EepromCRC        0x12U

#define C_Test                  0x40U  // readout of CRCok bits, RAM (memory integrity), BLDC tests, sensors test. cmd ID AA type
#define R_Test                  0x41U  // response code

#define C_StartBootLoader       0x4EU  // start bootloader, ACE1 sends requests to all internal PCBs
#define R_StartBootLoader       0x4FU  // response code

#define C_Test_Preamble         0xAAU
#define C_Test_GetStatus        0x02U

//************************************************************************************
//*******************   Long commands for CAN adapter ********************************
//*******************  every request is 6 bytes long  ********************************
//*******************   every reply is 12 bytes long  ********************************
//************************************************************************************
#define C_SetMainPos            0x01U  // set position and get position and all basic info
#define C_GetMainInfo           0x02U  // set position and get position and all basic info
#define C_GetLoadCounters       0x03U  // get load counters in % with fixed point
#define C_GetBigStatus          0x04U  // get extended status
#define C_GetVersions           0x05U  // get HW and FW versions
#define C_GetTimeCounter        0x06U  // get working time counter
#define C_GetIBIT               0x07U  // get IBIT status
#define C_ResetLoadCounters     0x08U  // reset load counters
#define C_WriteEEPROM           0x09U  // write EEPOM value
#define C_ReadEEPROM            0x0AU  // read EEPROM value
#define C_SetTimeout            0x0BU  // set LoC timeout
#define C_ReadTimeout           0x0CU  // read LoC timeout

#define ADDR_MP_LOW             0x20U  // low address for motor parameters
#define ADDR_MP_HIGH            0x29U  // high address for motor parameters

#define ALLSTS_ERROR_LOSSCOMM   (1UL << 4U)

#endif
