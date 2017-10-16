#include <stdio.h>
#include "settings.h"
#include "errors.h"
#include "flashEE.h"

#include "debug.h"

//declared in Main.c
extern char gl_c_EmergencyCode;

//flash-stored params declared in Main.c
//page 1
extern unsigned short gl_ush_flashParamAmplitudeCode;    //амплитуда колебания виброподвеса
extern unsigned short gl_ush_flashParamTactCode;         //код такта
extern unsigned short gl_ush_flashParamMCoeff;           //коэффициент ошумления
extern unsigned short gl_ush_flashParamStartMode;        //начальная мода
extern unsigned short gl_ush_flashParamDecCoeff;         //коэффициент вычета
extern unsigned short gl_ush_flashLockDev;               //флаг блокировки устройства

//page 2
extern unsigned short gl_ush_flashParamI1min;            //контрольное значение токв поджига I1
extern unsigned short gl_ush_flashParamI2min;            //контрольное значение тока поджига I2
extern unsigned short gl_ush_flashParamAmplAngMin1;      //контрольное значение раскачки с ДУСа
extern unsigned short gl_ush_flashParamHvApplyCount;     //Количество попыток применения 3kV при поджиге в пачке
extern unsigned short gl_ush_flashParamHvApplyDurat;     //Длительность попыток применения 3kV при поджиге [мсек]
extern unsigned short gl_ush_flashParamHvApplyPacks;     //Количество пачек попыток поджига

//page 3
extern unsigned short gl_ush_flashParamSignCoeff;        //знаковый коэффициент
extern unsigned short gl_ush_flashParamDeviceId;         //ID устройства
extern unsigned short gl_ush_flashParamDateYear;         //дата прибора.год
extern unsigned short gl_ush_flashParamDateMonth;        //дата прибора.месяц
extern unsigned short gl_ush_flashParamDateDay;          //дата прибора.день
extern char gl_ac_flashParamOrg[];                       //название организации


//калибровка термодатчиков
extern signed short     gl_ush_flashParam_calibT1;
extern unsigned short   gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD3_val;
extern signed short     gl_ush_flashParam_calibT2;
extern unsigned short   gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD3_val;

void load_params_p1( void) {
  //код амплитуды
  if( flashEE_load_short( ADDR_AMPLITUDE,   &gl_ush_flashParamAmplitudeCode)) gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //код такта подставки
  if( flashEE_load_short( ADDR_TACT_CODE,   &gl_ush_flashParamTactCode))      gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //коэффициент М
  if( flashEE_load_short( ADDR_M_COEFF,     &gl_ush_flashParamMCoeff))        gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //Начальная мода
  if( flashEE_load_short( ADDR_START_MODE,  &gl_ush_flashParamStartMode))     gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //коэффициент вычета
  if( flashEE_load_short( ADDR_DEC_COEFF,   &gl_ush_flashParamDecCoeff))      gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  //флаг блокировки устройства
  if( flashEE_load_short( ADDR_LOCK_DEV,    &gl_ush_flashLockDev))            gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  
#ifdef DEBUG
  printf("DBG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", gl_ush_flashParamDecCoeff,       gl_ush_flashParamDecCoeff);       //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //флаг блокировки устройства
#endif
}

void load_params_p2( void) {
  //минимальный ток I1
  if( flashEE_load_short( ADDR_CONTROL_I1, &gl_ush_flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I2
  if( flashEE_load_short( ADDR_CONTROL_I2, &gl_ush_flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный AmplAng
  if( flashEE_load_short( ADDR_CONTROL_AA, &gl_ush_flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Количество попыток применения 3kV при поджиге в пачке
  if( flashEE_load_short( ADDR_HV_APPLY_C, &gl_ush_flashParamHvApplyCount)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Длительность попыток применения 3kV при поджиге
  if( flashEE_load_short( ADDR_HV_APPLY_D, &gl_ush_flashParamHvApplyDurat)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Количество пачек попыток поджига
  if( flashEE_load_short( ADDR_HV_APPLY_P, &gl_ush_flashParamHvApplyPacks)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DBG:load_params_p2()\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min,        gl_ush_flashParamI1min);        //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min,        gl_ush_flashParamI2min);        //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1,  gl_ush_flashParamAmplAngMin1);  //минимальный AmplAng
  printf("DBG:   HV_count:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyCount, gl_ush_flashParamHvApplyCount); //HV_applies tries amount in pack
  printf("DBG:   HV_duration:    0x%04x (%04d)\n", gl_ush_flashParamHvApplyDurat, gl_ush_flashParamHvApplyDurat); //HV_applies tries duration
  printf("DBG:   HV_packs:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyPacks, gl_ush_flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void load_params_p3( void) {
  //знаковый коэффициент
  if( flashEE_load_short( ADDR_SIGN_COEFF, &gl_ush_flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //серийный номер
  if( flashEE_load_short( ADDR_DEVICE_ID, &gl_ush_flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.год
  if( flashEE_load_short( ADDR_DATE_Y, &gl_ush_flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.месяц
  if( flashEE_load_short( ADDR_DATE_M, &gl_ush_flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //дата.день
  if( flashEE_load_short( ADDR_DATE_D, &gl_ush_flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //организация
  if( flashEE_load_text( ADDR_ORG, gl_ac_flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

#ifdef DEBUG
  printf("DBG:load_params_p3()\n");
  printf("DBG:   Sign coeff:      0x%04x (%04d)\n", gl_ush_flashParamSignCoeff, gl_ush_flashParamSignCoeff);  //знаковый коэффициент
  printf("DBG:   Serial number:   0x%04x (%04d)\n", gl_ush_flashParamDeviceId,  gl_ush_flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:    '%s'\n", gl_ac_flashParamOrg);                                              //организация
  printf("DBG:   Year:            0x%04x (%04d)\n", gl_ush_flashParamDateYear,  gl_ush_flashParamDateYear);   //год
  printf("DBG:   Month:           0x%04x (%04d)\n", gl_ush_flashParamDateMonth, gl_ush_flashParamDateMonth);  //месяц
  printf("DBG:   Day:             0x%04x (%04d)\n", gl_ush_flashParamDateDay,   gl_ush_flashParamDateDay);    //день
#endif
}

void load_params_p4( void) {
  //Температура минимальной точки калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1, ( unsigned short *) &gl_ush_flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD1, &gl_ush_flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD2, &gl_ush_flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты третьего термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T1_TD3, &gl_ush_flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //Температура максимальной точки калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2, ( unsigned short *) &gl_ush_flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD1, &gl_ush_flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD2, &gl_ush_flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( ADDR_TCALIB_T2_TD3, &gl_ush_flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
#ifdef DEBUG
  printf("DBG:load_params_p4()\n");
  printf("DBG:   T-Calibration T1=0x%04x (%04d)\n", gl_ush_flashParam_calibT1,    gl_ush_flashParam_calibT1);    //Температурная калибровка: температура нижней точки
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD1_val,  gl_ush_flashParamT1_TD1_val);  //Температурная калибровка: показания термодатчика TD1 в нижней температурной точке
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD2_val,  gl_ush_flashParamT1_TD2_val);  //Температурная калибровка: показания термодатчика TD2 в нижней температурной точке
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", gl_ush_flashParamT1_TD3_val,  gl_ush_flashParamT1_TD3_val);  //Температурная калибровка: показания термодатчика TD3 в нижней температурной точке

  printf("DBG:   T-Calibration T2=0x%04x (%04d)\n", gl_ush_flashParam_calibT2,    gl_ush_flashParam_calibT2);    //Температурная калибровка: температура верхней точки
  printf("DBG:   T1_TD1:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD1_val,  gl_ush_flashParamT2_TD1_val);  //Температурная калибровка: показания термодатчика TD1 в верхней температурной точке
  printf("DBG:   T1_TD2:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD2_val,  gl_ush_flashParamT2_TD2_val);  //Температурная калибровка: показания термодатчика TD2 в верхней температурной точке
  printf("DBG:   T1_TD3:          0x%04x (%04d)\n", gl_ush_flashParamT2_TD3_val,  gl_ush_flashParamT2_TD3_val);  //Температурная калибровка: показания термодатчика TD3 в верхней температурной точке
#endif
}

void check_params_p1( void) {
  //Код амплитуды [0-255]. дефолтное значение 90
  //90 для большого 35 для маленького
  if( gl_ush_flashParamAmplitudeCode > 255)
    gl_ush_flashParamAmplitudeCode = 35;

  //Код такта подставки [0-3]. дефолтное значение 0
  if( gl_ush_flashParamTactCode > 3)
    gl_ush_flashParamTactCode = 0;

  //Коэффициент М[0-1] = значения параметра [0-250]
  //дефолтное значение 200 (что означает M=0.8 и DAC1 = 0.8 * DAC0)
  if( gl_ush_flashParamMCoeff > 250)
    gl_ush_flashParamMCoeff = 200;

  //Начальная мода [0-250]. дефолтное значение 125 (что означает 1,25В на DAC2)
  if( gl_ush_flashParamStartMode > 250)
    gl_ush_flashParamStartMode = 125;

  //коэффициент вычета
  //default значение 0,04 УТОЧНИТЬ!
  if( gl_ush_flashParamDecCoeff == 0xffff)
    gl_ush_flashParamDecCoeff = ( int) ( 0.04 * 65535.);

  //флаг блокировки устройства
  //default значение 0 - режим разработчиков
  if( gl_ush_flashLockDev != 1)
    gl_ush_flashLockDev = 0;

#ifdef DEBUG
  printf("DBG: check_params_p1(): params checked for the range. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", gl_ush_flashParamDecCoeff,       gl_ush_flashParamDecCoeff);       //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //флаг блокировки устройства
#endif
}

void check_params_p2( void) {
  //контрольный ток поджига I1 [0-0.750 mA] = значения параметра [ 0 - 65534]
  //default значение 0.4 mA
  if( gl_ush_flashParamI1min == 0xffff)
    gl_ush_flashParamI1min = ( short) ( 65535. * 0.4 / 0.75);

  //контрольный ток поджига I2 [0-0.750 mA] = значения параметра [ 0 - 65534]
  //default значение 0.4 mA
  if( gl_ush_flashParamI2min == 0xffff)
    gl_ush_flashParamI2min = ( short) ( 65535. * 0.4 / 0.75);

  //контрольное значение раскачки - амплитуда ДУПа (0-3В) = значения параметра [ 0 - 65534]
  //default значение 1.0В
  if( gl_ush_flashParamAmplAngMin1 == 0xffff)
    gl_ush_flashParamAmplAngMin1 = ( int) ( 1.0 / 3. * 65535.);

  //количество попыток применения 3kV при поджиге ( 1 - 20)
  //default value = 10
  if( gl_ush_flashParamHvApplyCount < 1 || gl_ush_flashParamHvApplyCount > 20) {
    gl_ush_flashParamHvApplyCount = 10;
  }

  //HV_applies tries duration [0 - 5 sec] = values [0 - 5000]
  //default value = 1000
  if( gl_ush_flashParamHvApplyDurat < 1 || gl_ush_flashParamHvApplyDurat > 5000) {
    gl_ush_flashParamHvApplyDurat = 1000;
  }

  //HV_applies tries packs [1 - 10] = values [1 - 10]
  //default value = 5
  if( gl_ush_flashParamHvApplyPacks < 1 || gl_ush_flashParamHvApplyPacks > 10) {
    gl_ush_flashParamHvApplyPacks = 5;
  }

#ifdef DEBUG
  printf("DBG: check_params_p2(): params checked for the range. Here they are:\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min,        gl_ush_flashParamI1min);        //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min,        gl_ush_flashParamI2min);        //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1,  gl_ush_flashParamAmplAngMin1);  //минимальный AmplAng
  printf("DBG:   HV_count:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyCount, gl_ush_flashParamHvApplyCount); //HV_applies tries amount
  printf("DBG:   HV_duration:    0x%04x (%04d)\n", gl_ush_flashParamHvApplyDurat, gl_ush_flashParamHvApplyDurat);   //HV_applies tries duration
  printf("DBG:   HV_packs:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyPacks, gl_ush_flashParamHvApplyPacks);   //HV_applies tries packs
#endif
}

void check_params_p3( void) {
  int i;
  //знаковый коэффициент. [-1; +1]
  //default value = 1
  if( gl_ush_flashParamSignCoeff > 2)
    gl_ush_flashParamSignCoeff = 2;

  //ID устройства
  if( gl_ush_flashParamDeviceId == 65535)
    gl_ush_flashParamDeviceId = 0;

  //Дата ?? устройства
  //default value = 2016.01.01
  if( gl_ush_flashParamDateYear < 2000 || gl_ush_flashParamDateYear > 2200)
    gl_ush_flashParamDateYear = 2016;

  if( gl_ush_flashParamDateMonth > 12)
    gl_ush_flashParamDateMonth = 1;

  if( gl_ush_flashParamDateDay > 31)
    gl_ush_flashParamDateDay = 1;

  //название организации
  //default - все минусы
  for( i=0; i<17; i++) {
    if( gl_ac_flashParamOrg[i] < 33 || gl_ac_flashParamOrg[i] > 126)
      gl_ac_flashParamOrg[i] = ' ';
  }



#ifdef DEBUG
  printf("DBG: check_params_p3(): params checked for the range. Here they are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", gl_ush_flashParamSignCoeff,  gl_ush_flashParamSignCoeff);  //знаковый коэффициент
  printf("DBG:   Serial number:  0x%04x (%04d)\n", gl_ush_flashParamDeviceId,   gl_ush_flashParamDeviceId);   //серийный номер
  printf("DBG:   Year:           0x%04x (%04d)\n", gl_ush_flashParamDateYear,   gl_ush_flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", gl_ush_flashParamDateMonth,  gl_ush_flashParamDateMonth);  //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", gl_ush_flashParamDateDay,    gl_ush_flashParamDateDay);    //день
  printf("DBG:   Organization:   '%s'\n", gl_ac_flashParamOrg);                                               //организация
#endif
}

void check_params_p4( void) {
  if( gl_ush_flashParam_calibT1 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  ||
      gl_ush_flashParam_calibT1 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {

    gl_ush_flashParam_calibT1 = 0;
    gl_ush_flashParamT1_TD1_val = 0;
    gl_ush_flashParamT1_TD2_val = 1;
    gl_ush_flashParamT1_TD3_val = 2;
  }

  if( gl_ush_flashParam_calibT2 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) ||
      gl_ush_flashParam_calibT2 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {

    gl_ush_flashParam_calibT2 = 0;
    gl_ush_flashParamT2_TD1_val = 0;
    gl_ush_flashParamT2_TD2_val = 1;
    gl_ush_flashParamT2_TD3_val = 2;
  }
#ifdef DEBUG
  printf("DBG: check_params_p4(): params checked for the range. Here they are:\n");
#endif
}

void load_params( void) {
  load_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_LOAD_PARAMS_FAIL) load_params_p4();

  check_params_p1();
  check_params_p2();
  check_params_p3();
  check_params_p4();
}

void save_params_p1( void) {
#ifdef DEBUG
  printf("DBG: save_params_p1(): params to be saved are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", gl_ush_flashParamAmplitudeCode,  gl_ush_flashParamAmplitudeCode);  //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", gl_ush_flashParamTactCode,       gl_ush_flashParamTactCode);       //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", gl_ush_flashParamMCoeff,         gl_ush_flashParamMCoeff);         //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", gl_ush_flashParamStartMode,      gl_ush_flashParamStartMode);      //Начальная мода
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", gl_ush_flashParamDecCoeff,       gl_ush_flashParamDecCoeff);       //коэффициент вычета
  printf("DBG:   Dev Lock:       0x%04x (%04d)\n", gl_ush_flashLockDev,             gl_ush_flashLockDev);             //флаг блокировки устройства
#endif

  if( flashEE_erase_page( ADDR_PAGE1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_AMPLITUDE, gl_ush_flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TACT_CODE, gl_ush_flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_M_COEFF, gl_ush_flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_START_MODE, gl_ush_flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEC_COEFF, gl_ush_flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
#ifdef DEBUG
  if( gl_ush_flashLockDev == 1) {
    printf( "DBG: device will be locked\n");
  }
#endif
  if( flashEE_save_short( ADDR_LOCK_DEV, gl_ush_flashLockDev)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p2( void) {
#ifdef DEBUG
  printf("DBG: save_params_p2(): params to be saved are:\n");
  printf("DBG:   Control I1:     0x%04x (%04d)\n", gl_ush_flashParamI1min,        gl_ush_flashParamI1min);        //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", gl_ush_flashParamI2min,        gl_ush_flashParamI2min);        //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", gl_ush_flashParamAmplAngMin1,  gl_ush_flashParamAmplAngMin1);  //минимальный AmplAng
  printf("DBG:   HV_count:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyCount, gl_ush_flashParamHvApplyCount); //HV_applies tries amount
  printf("DBG:   HV_duration:    0x%04x (%04d)\n", gl_ush_flashParamHvApplyDurat, gl_ush_flashParamHvApplyDurat); //HV_applies tries duration
  printf("DBG:   HV_packs:       0x%04x (%04d)\n", gl_ush_flashParamHvApplyPacks, gl_ush_flashParamHvApplyPacks); //HV_applies tries packs
#endif

  if( flashEE_erase_page( ADDR_PAGE2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_CONTROL_I1, gl_ush_flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_I2, gl_ush_flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_CONTROL_AA, gl_ush_flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_HV_APPLY_C, gl_ush_flashParamHvApplyCount)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_HV_APPLY_D, gl_ush_flashParamHvApplyDurat)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_HV_APPLY_P, gl_ush_flashParamHvApplyPacks)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p3( void) {
#ifdef DEBUG
  printf("DBG: save_params_p3(): params to be saved are:\n");
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", gl_ush_flashParamSignCoeff,  gl_ush_flashParamSignCoeff);  //Знаковый коэффициент
  printf("DBG:   Serial number:  0x%04x (%04d)\n", gl_ush_flashParamDeviceId,   gl_ush_flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:   '%s'\n", gl_ac_flashParamOrg);                                               //организация
  printf("DBG:   Year:           0x%04x (%04d)\n", gl_ush_flashParamDateYear,   gl_ush_flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", gl_ush_flashParamDateMonth,  gl_ush_flashParamDateMonth);  //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", gl_ush_flashParamDateDay,    gl_ush_flashParamDateDay);    //день
#endif

  if( flashEE_erase_page( ADDR_PAGE3)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }

  if( flashEE_save_short( ADDR_SIGN_COEFF, gl_ush_flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DEVICE_ID, gl_ush_flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_Y, gl_ush_flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_M, gl_ush_flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_DATE_D, gl_ush_flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_text( ADDR_ORG, gl_ac_flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params_p4( void) {
#ifdef DEBUG
  printf("DBG: save_params_p4(): params to be saved are:\n");
#endif

  if( flashEE_save_short( ADDR_TCALIB_T1, gl_ush_flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD1, gl_ush_flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD2, gl_ush_flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T1_TD3, gl_ush_flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2, gl_ush_flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD1, gl_ush_flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD2, gl_ush_flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( ADDR_TCALIB_T2_TD3, gl_ush_flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params( void) {
  save_params_p1();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p2();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p3();
  if( gl_c_EmergencyCode != ERROR_FLASH_SAVE_PARAMS_FAIL) save_params_p4();
}