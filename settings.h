#ifndef SETTINGS_H
#define SETTINGS_H

#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000

void load_params( void);                        //загрузить параметры из флэш-памяти в переменные
void load_params_p1( void);
void load_params_p2( void);
void load_params_p3( void);
void load_params_p4( void);

void save_params( void);                        //сохранить параметры из переменных во флэш-память
void save_params_p1( void);
void save_params_p2( void);
void save_params_p3( void);
void save_params_p4( void);


void check_params_p1( void);
void check_params_p2( void);
void check_params_p3( void);
void check_params_p4( void);

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 1
#define ADDR_PAGE1            0xF000
#define ADDR_AMPLITUDE        0xF000
#define ADDR_TACT_CODE        0xF002
#define ADDR_M_COEFF          0xF004
#define ADDR_START_MODE       0xF006
#define ADDR_DEC_COEFF        0xF008
#define ADDR_LOCK_DEV         0xF00A

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 2
#define ADDR_PAGE2            0xF200
#define ADDR_CONTROL_I1       0xF200
#define ADDR_CONTROL_I2       0xF202
#define ADDR_CONTROL_AA       0xF204
#define ADDR_HV_APPLY_C       0xF206
#define ADDR_HV_APPLY_D       0xF208
#define ADDR_HV_APPLY_P       0xF20A

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 3
#define ADDR_PAGE3            0xF400
#define ADDR_SIGN_COEFF       0xF400
#define ADDR_DEVICE_ID        0xF402
#define ADDR_DATE_Y           0xF404
#define ADDR_DATE_M           0xF406
#define ADDR_DATE_D           0xF408
#define ADDR_ORG              0xF40A    //16 bytes length

// ADDRESSES FOR FLASH-STORED PARAMS. PAGE 4
#define ADDR_PAGE4            0xF600
#define ADDR_TCALIB_T1        0xF600
#define ADDR_TCALIB_T1_TD1    0xF602
#define ADDR_TCALIB_T1_TD2    0xF604
#define ADDR_TCALIB_T1_TD3    0xF606
#define ADDR_TCALIB_T2        0xF608
#define ADDR_TCALIB_T2_TD1    0xF60A
#define ADDR_TCALIB_T2_TD2    0xF60C
#define ADDR_TCALIB_T2_TD3    0xF60E
#endif