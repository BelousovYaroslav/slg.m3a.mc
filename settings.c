#include "settings.h"
#include "errors.h"
#include "flashEE.h"

//declared in Main.c
extern char gl_c_EmergencyCode;

//flash-stored params declared in Main.c
extern unsigned short flashParamAmplitudeCode;    //амплитуда колебания виброподвеса
extern unsigned short flashParamTactCode;         //код такта
extern unsigned short flashParamMCoeff;           //коэффициент ошумления
extern unsigned short flashParamStartMode;        //начальная мода
extern unsigned int flashParamDeviceId;           //ID устройства

extern char flashParamOrg[];                      //название организации

extern unsigned short flashParamDateYear;         //дата прибора.год
extern unsigned short flashParamDateMonth;        //дата прибора.месяц
extern unsigned short flashParamDateDay;          //дата прибора.день

extern unsigned short flashParamI1min;            //контрольное значение токв поджига I1
extern unsigned short flashParamI2min;            //контрольное значение тока поджига I2
extern unsigned short flashParamAmplAngMin1;      //контрольное значение раскачки с ДУСа
extern unsigned short flashParamDecCoeff;         //коэффициент вычета
extern unsigned short flashParamSignCoeff;        //знаковый коэффициент
extern unsigned short flashParamPhaseShift;       //[OBSOLETE] фазовый сдвиг

//калибровка термодатчиков
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;



void load_params( void) {
  //код амплитуды
  if( flashEE_load_short( 0xf000, &flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //код такта подставки
  if( flashEE_load_short( 0xf002, &flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //коэффициент М
  if( flashEE_load_short( 0xf004, &flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Начальная мода
  if( flashEE_load_short( 0xf006, &flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //серийный номер
  if( flashEE_load_int( 0xf008, &flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //организация
  if( flashEE_load_text( 0xf00C, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  } 
  //год
  if( flashEE_load_short( 0xf02C, &flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //месяц
  if( flashEE_load_short( 0xf02E, &flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //день
  if( flashEE_load_short( 0xf030, &flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I1
  if( flashEE_load_short( 0xf032, &flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный ток I2
  if( flashEE_load_short( 0xf034, &flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //минимальный AmplAng
  if( flashEE_load_short( 0xf036, &flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //коэффициент вычета
  if( flashEE_load_short( 0xf038, &flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //SA время
  if( flashEE_load_short( 0xf03A, &flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //**************************************************************************************
  // КАЛИБРОВКА ТЕРМОДАТЧИКОВ
  //**************************************************************************************
  //Температура минимальной точки калибровки
  if( flashEE_load_short( 0xf03C, ( unsigned short *) &flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( 0xf03E, &flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( 0xf040, &flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты третьего термодатчика при минимальной температуре калибровки
  if( flashEE_load_short( 0xf042, &flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  //Температура максимальной точки калибровки
  if( flashEE_load_short( 0xf044, ( unsigned short *) &flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты первого термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( 0xf046, &flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( 0xf048, &flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  //Отсчёты второго термодатчика при максимальной температуре калибровки
  if( flashEE_load_short( 0xf04A, &flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }

  
  //********************************************************************
  // параметр сдвига
  if( flashEE_load_short( 0xf04C, &flashParamPhaseShift)) {
    gl_c_EmergencyCode = ERROR_FLASH_LOAD_PARAMS_FAIL;
  }
  
#ifdef DEBUG
  printf("DBG: load_params(): params loaded from flash memory. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DBG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
#endif

  //PARAMS CHECKING
  if( flashParamAmplitudeCode > 255)     //Код амплитуды [0-255]. дефолтное значение 90
    flashParamAmplitudeCode = 35;       //90 для большого 35 для маленького

  if( flashParamTactCode > 3)       //Код такта амплитуды [0-3]. дефолтное значение 0
    flashParamTactCode = 0;

  if( flashParamMCoeff > 250)     //Коэффициент М[0-1] = значения параметра [0-250].
    flashParamMCoeff = 125;       //дефолтное значение 125 (что означает M=0.5 и DAC1 = 0.5 * DAC0)  

  if( flashParamStartMode > 250)     //Начальная мода [0-250]. дефолтное значение 125 (что означает 1,25В на DAC2)
    flashParamStartMode = 125;

  //device_id = 0;
  //organization[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};

  if( flashParamDateYear < 2000 && flashParamDateYear > 2200)
    flashParamDateYear = 2009;
  
  if( flashParamDateMonth > 12)
    flashParamDateMonth = 9;

  if( flashParamDateDay > 31)
    flashParamDateDay = 3;

/*
  if( flashParamI1min > 255)
    flashParamI1min = 0;

  if( flashParamI2min > 255)
    flashParamI2min = 0;

  if( flashParamAmplAngMin1 > 255)
    flashParamAmplAngMin1 = 0;
*/

  if( flashParamDecCoeff == 0xffff)
    flashParamDecCoeff = 0;

  if( flashParamSignCoeff > 2)
    flashParamSignCoeff = 2;

  if( flashParam_calibT1 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  ||
      flashParam_calibT1 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT1 = 0;
    flashParamT1_TD1_val = 0;
    flashParamT1_TD2_val = 1;
    flashParamT1_TD3_val = 2;
  }

  if( flashParam_calibT2 < ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) ||
      flashParam_calibT2 > ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
    flashParam_calibT2 = 0;
    flashParamT2_TD1_val = 0;
    flashParamT2_TD2_val = 1;
    flashParamT2_TD3_val = 2;
  }

  if( flashParamPhaseShift > 63) {
    flashParamPhaseShift = 0;
  }

#ifdef DEBUG
  printf("DBG: load_params(): params checked for the range. Here they are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //знаковый коэффициент
  printf("DBG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
#endif
}

void SaveThermoCalibPoint( void) {
  if( flashEE_save_short( 0xf03C, flashParam_calibT1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf03E, flashParamT1_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf040, flashParamT1_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf042, flashParamT1_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf044, flashParam_calibT2)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf046, flashParamT2_TD1_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf048, flashParamT2_TD2_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf04A, flashParamT2_TD3_val)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}

void save_params( void) {
#ifdef DEBUG
  printf("DBG: save_params(): params to be saved are:\n");
  printf("DBG:   Amplitude Code: 0x%04x (%04d)\n", flashParamAmplitudeCode, flashParamAmplitudeCode); //код амплитуды
  printf("DBG:   Base Tact Code: 0x%04x (%04d)\n", flashParamTactCode, flashParamTactCode);   //код такта подставки
  printf("DBG:   M Coefficient:  0x%04x (%04d)\n", flashParamMCoeff, flashParamMCoeff);       //коэффициент М
  printf("DBG:   Start Mode:     0x%04x (%04d)\n", flashParamStartMode, flashParamStartMode); //Начальная мода
  printf("DBG:   Serial number:  0x%04x (%04d)\n", flashParamDeviceId, flashParamDeviceId);   //серийный номер
  printf("DBG:   Organization:   '%s'\n", flashParamOrg);                                     //организация
  printf("DBG:   Year:           0x%04x (%04d)\n", flashParamDateYear, flashParamDateYear);   //год
  printf("DBG:   Month:          0x%04x (%04d)\n", flashParamDateMonth, flashParamDateMonth); //месяц
  printf("DBG:   Day:            0x%04x (%04d)\n", flashParamDateDay, flashParamDateDay);     //день
  printf("DBG:   Control I1:     0x%04x (%04d)\n", flashParamI1min, flashParamI1min);         //минимальный ток I1
  printf("DBG:   Control I2:     0x%04x (%04d)\n", flashParamI2min, flashParamI2min);         //минимальный ток I2
  printf("DBG:   Control AA:     0x%04x (%04d)\n", flashParamAmplAngMin1, flashParamAmplAngMin1); //минимальный AmplAng
  printf("DBG:   Dec. Coeff:     0x%04x (%04d)\n", flashParamDecCoeff, flashParamDecCoeff);   //коэффициент вычета
  printf("DBG:   Sign coeff:     0x%04x (%04d)\n", flashParamSignCoeff, flashParamSignCoeff); //Знаковый коэффициент
  printf("DBG:   Phase shift:    0x%04x (%04d)\n", flashParamPhaseShift, flashParamPhaseShift); //фазовый сдвиг
#endif

  if( flashEE_erase_page( 0xf000)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf000, flashParamAmplitudeCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf002, flashParamTactCode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf004, flashParamMCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf006, flashParamStartMode)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_int( 0xf008, flashParamDeviceId)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_text( 0xf00C, flashParamOrg, 16)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  } 
  if( flashEE_save_short( 0xf02C, flashParamDateYear)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf02E, flashParamDateMonth)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf030, flashParamDateDay)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf032, flashParamI1min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf034, flashParamI2min)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf036, flashParamAmplAngMin1)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf038, flashParamDecCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  if( flashEE_save_short( 0xf03A, flashParamSignCoeff)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
  //0xf03C - 0xf04A включительно - калибровочные данные термодатчиков
  SaveThermoCalibPoint();

  if( flashEE_save_short( 0xf04C, flashParamPhaseShift)) {
    gl_c_EmergencyCode = ERROR_FLASH_SAVE_PARAMS_FAIL;
    return;
  }
}