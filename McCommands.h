#ifndef T39_M3A_MC_COMMAND_SET
#define T39_M3A_MC_COMMAND_SET

#define MC_COMMAND_SET                        33    //33 = 0x21 = "!"
#define MC_COMMAND_REQ                        34    //34 = 0x22 = """

#define MC_COMMAND_ACT_SAVE_FLASH_PARAMS      35    //35 = 0x23 = "#"    Param: 0-1-2-3  - page to save
#define MC_COMMAND_ACT_RELOAD_FLASH_PARAMS    36    //36 = 0x24 = "$"    Param: 0-1-2-3  - page to reload

#define MC_COMMAND_ACT_T_CALIBRATION          37    //37 = 0x25 = "%"
#define MC_COMMAND_ACT_RESET_T_CALIB          38    //38 = 0x26 = "&"

#define MC_COMMAND_ACT_LASER_OFF              39    //39 = 0x27 = "'"
#define MC_COMMAND_ACT_INTEGR_OFF             40    //40 = 0x28 = "("
#define MC_COMMAND_ACT_INTEGR_ON              41    //41 = 0x29 = ")"
#define MC_COMMAND_ACT_INTEGR_RESET           42    //42 = 0x2A = "*"

#define MC_COMMAND_ACT_SWC_DW_DNDU_OUTPUT     43    //43 = 0x2B = "+"

#define MC_COMMAND_ACT_LOCK_DEVICE            44    //44 = 0x2C = ","
#define MC_COMMAND_ACT_UNLOCK_DEVICE          45    //45 = 0x2D = "-"

/*
//********************* GROUP1
//MAIN PARAMETERS. CHANGES OFTEN
#define MC_COMMAND_SET_AMPLITUDE         33    //33 = 0x21 = "!"
#define MC_COMMAND_REQ_AMPLITUDE         34    //34 = 0x22 = """

#define MC_COMMAND_SET_CODETACT          35    //35 = 0x23 = "#"
#define MC_COMMAND_REQ_CODETACT          36    //36 = 0x24 = "$"

#define MC_COMMAND_SET_M_COEFF           37    //37 = 0x25 = "%"
#define MC_COMMAND_REQ_M_COEFF           38    //38 = 0x26 = "&"

#define MC_COMMAND_SET_START_MODE        39    //39 = 0x27 = "'"
#define MC_COMMAND_REQ_START_MODE        40    //40 = 0x28 = "("

#define MC_COMMAND_SET_DEC_COEFF         41    //41 = 0x29 = ")"
#define MC_COMMAND_REQ_DEC_COEFF         42    //42 = 0x2A = "*"


//********************* GROUP2
//SELF-CONTROL PARAMETERS. CHANGES RARE
#define MC_COMMAND_SET_CONTROL_I1        43    //43 = 0x2B = "+"
#define MC_COMMAND_REQ_CONTROL_I1        44    //44 = 0x2C = ","

#define MC_COMMAND_SET_CONTROL_I2        45    //45 = 0x2D = "-"
#define MC_COMMAND_REQ_CONTROL_I2        46    //46 = 0x2E = "."

#define MC_COMMAND_SET_CONTROL_AA        47    //47 = 0x2F = "/"
#define MC_COMMAND_REQ_CONTROL_AA        48    //48 = 0x30 = "0"


#define MC_COMMAND_SET_HV_APPLY_MAX      49    //49 = 0x31 = "1"
#define MC_COMMAND_REQ_HV_APPLY_MAX      50    //50 = 0x32 = "2"
#define MC_COMMAND_REQ_HV_APPLY_CUR      51    //51 = 0x33 = "3"

//HV_APPLY_DURATION
//#define MC_COMMAND_SET_HV_APPLY_MAX      49    //49 = 0x31 = "1"
//#define MC_COMMAND_REQ_HV_APPLY_MAX      50    //50 = 0x32 = "2"

//********************* GROUP3
//REST PARAMETERS THAT CHANGES VERY RARE
//Sign Coeff
#define MC_COMMAND_SET_SIGN_COEFF        86    //86 = 0x56 = "V"
#define MC_COMMAND_REQ_SIGN_COEFF        52    //52 = 0x34 = "4"

//Device ID
#define MC_COMMAND_SET_DEVICE_ID         53    //53 = 0x35 = "5"
#define MC_COMMAND_REQ_DEVICE_ID         54    //54 = 0x36 = "6"


//Device.date
#define MC_COMMAND_REQ_DEV_DATE          55    //55 = 0x37 = "7"
#define MC_COMMAND_SET_DEV_DATE_Y        56    //56 = 0x38 = "8"
#define MC_COMMAND_SET_DEV_DATE_M        57    //57 = 0x39 = "9"
#define MC_COMMAND_SET_DEV_DATE_D        58    //58 = 0x3A = ":"

//ORGANIZATION
#define MC_COMMAND_REQ_ORGANIZATION      59    //59 = 0x3B = ";"
#define MC_COMMAND_SET_ORG_B1            60    //60 = 0x3C = "<"
#define MC_COMMAND_SET_ORG_B2            61    //61 = 0x3D = "="
#define MC_COMMAND_SET_ORG_B3            62    //62 = 0x3E = ">"
#define MC_COMMAND_SET_ORG_B4            63    //63 = 0x3F = "?"
#define MC_COMMAND_SET_ORG_B5            64    //64 = 0x40 = "@"
#define MC_COMMAND_SET_ORG_B6            65    //65 = 0x41 = "A"
#define MC_COMMAND_SET_ORG_B7            66    //66 = 0x42 = "B"
#define MC_COMMAND_SET_ORG_B8            67    //67 = 0x43 = "C"
#define MC_COMMAND_SET_ORG_B9            68    //68 = 0x44 = "D"
#define MC_COMMAND_SET_ORG_B10           69    //69 = 0x45 = "E"
#define MC_COMMAND_SET_ORG_B11           70    //70 = 0x46 = "F"
#define MC_COMMAND_SET_ORG_B12           71    //71 = 0x47 = "G"
#define MC_COMMAND_SET_ORG_B13           72    //72 = 0x48 = "H"
#define MC_COMMAND_SET_ORG_B14           73    //73 = 0x49 = "I"
#define MC_COMMAND_SET_ORG_B15           74    //74 = 0x4A = "J"
#define MC_COMMAND_SET_ORG_B16           75    //75 = 0x4B = "K"

//********************* GROUP4
//THERMO-CALIBRATION
#define MC_COMMAND_GET_T_CALIB           76    //76 = 0x4C = "L"
//SETTING IMPLEMENTS THROUGH  MC_COMMAND_ACT_T_CALIBRATION and MC_COMMAND_ACT_RESET_T_CALIB


//*********************
//ACTIONS, SWITCHES
#define MC_COMMAND_ACT_SAVE_FLASH_PARAMS     77    //77 = 0x4D = "M"    Param: 0-1-2-3  - page to save
#define MC_COMMAND_ACT_RELOAD_FLASH_PARAMS   78    //78 = 0x4E = "N"    Param: 0-1-2-3  - page to reload

#define MC_COMMAND_ACT_T_CALIBRATION     79    //79 = 0x4F = "O"
#define MC_COMMAND_ACT_RESET_T_CALIB     80    //80 = 0x50 = "P"

#define MC_COMMAND_ACT_LASER_OFF         81    //81 = 0x51 = "Q"
#define MC_COMMAND_ACT_INTEGR_OFF        82    //82 = 0x52 = "R"
#define MC_COMMAND_ACT_INTEGR_ON         83    //83 = 0x53 = "S"
#define MC_COMMAND_ACT_INTEGR_RESET      84    //84 = 0x54 = "T"

#define MC_COMMAND_SWC_DW_DNDU_OUTPUT    85    //85 = 0x55 = "U"
*/
#endif