#include <ADuC7026.h>
#include <stdio.h>

#include "settings.h"
#include "version.h"
#include "errors.h"
#include "AnalogueParamsConstList.h"

#include "debug.h"

//declared in Main.c
extern char gl_c_EmergencyCode;
extern int gl_n_prT1VAL;
extern signed short gl_ssh_SA_time;
extern signed short gl_ssh_ampl_angle;
extern signed short gl_ssh_angle_inc;
extern signed short gl_ssh_angle_inc_prev;
extern short gl_nSentPackIndex;

extern char gl_cPos_in_in_buf;
extern char gl_acInput_buffer[];

//flash-stored params declared in Main.c
extern unsigned short gl_ush_flashParamAmplitudeCode;    //амплитуда колебания виброподвеса
extern unsigned short gl_ush_flashParamTactCode;         //код такта
extern unsigned short gl_ush_flashParamMCoeff;           //коэффициент ошумления
extern unsigned short gl_ush_flashParamStartMode;        //начальная мода
extern unsigned short gl_ush_flashParamI1min;            //контрольное значение токв поджига I1
extern unsigned short gl_ush_flashParamI2min;            //контрольное значение тока поджига I2
extern unsigned short gl_ush_flashParamAmplAngMin1;      //контрольное значение раскачки с ДУСа
extern unsigned short gl_ush_flashParamDecCoeff;         //коэффициент вычета
extern unsigned short gl_ush_flashParamSignCoeff;        //знаковый коэффициент
extern unsigned short gl_ush_flashParamHvApplyCount;     //HV_applies tries amount
extern unsigned short gl_ush_flashParamHvApplyDurat;     //HV_applies tries duration [msec]
extern unsigned short gl_ush_flashParamHvApplyPacks;     //HV_applies packs
extern unsigned short gl_ush_flashParamDeviceId;         //ID устройства
extern unsigned short gl_ush_flashParamDateYear;         //дата прибора.год
extern unsigned short gl_ush_flashParamDateMonth;        //дата прибора.месяц
extern unsigned short gl_ush_flashParamDateDay;          //дата прибора.день
extern char gl_ac_flashParamOrg[];                       //название организации

extern unsigned short gl_ushFiringTry;                 //число включений HV при поджиге

//implemented in Main.c
extern void send_pack( short analog_param);
extern void pause( int n);
extern void processIncomingCommand( void);

void deadloop_no_firing( int nError) {
  //ОБРАБОТКА ОТКАЗА ПОДЖИГА
#ifdef DEBUG
  printf("DBG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //выдача данных
    gl_nSentPackIndex = AMPLITUDE;        send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;        send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;          send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;        send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = CONTROL_I1;       send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;       send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;       send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = DECCOEFF_CURRENT; send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = SIGNCOEFF;        send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = VERSION;          send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача настроечных параметров
    gl_nSentPackIndex = AMPLITUDE;          send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;          send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;            send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;          send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF_CURRENT;   send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;         send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;         send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;         send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = HV_APPLY_COUNT_SET; send_pack( gl_ush_flashParamHvApplyCount);
    gl_nSentPackIndex = HV_APPLY_DURAT_SET; send_pack( gl_ush_flashParamHvApplyDurat);
    gl_nSentPackIndex = HV_APPLY_PACKS;     send_pack( gl_ush_flashParamHvApplyPacks);
    gl_nSentPackIndex = SIGNCOEFF;          send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;             send_pack( gl_ush_flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;             send_pack( gl_ush_flashParamDateYear);
    gl_nSentPackIndex = DATE_M;             send_pack( gl_ush_flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;             send_pack( gl_ush_flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;             send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;             send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;             send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;             send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;             send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;             send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;             send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;             send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;             send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;            send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;            send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;            send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;            send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;            send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;            send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;            send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;            send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"мертвый" while
}

void deadloop_current_unstable( int nError) {
  //ОБРАБОТКА ОТКАЗА ПОДЖИГА
#ifdef DEBUG
  printf("DBG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //выдача данных
    gl_nSentPackIndex = AMPLITUDE;          send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;          send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;            send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;          send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = CONTROL_I1;         send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;         send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;         send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = DECCOEFF_CURRENT;   send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = SIGNCOEFF;          send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = VERSION;            send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача настроечных параметров
    gl_nSentPackIndex = AMPLITUDE;          send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;          send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;            send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;          send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF_CURRENT;   send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;         send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;         send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;         send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = HV_APPLY_COUNT_SET; send_pack( gl_ush_flashParamHvApplyCount);
    gl_nSentPackIndex = HV_APPLY_DURAT_SET; send_pack( gl_ush_flashParamHvApplyDurat);
    gl_nSentPackIndex = HV_APPLY_PACKS;     send_pack( gl_ush_flashParamHvApplyPacks);
    gl_nSentPackIndex = SIGNCOEFF;          send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;             send_pack( gl_ush_flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;             send_pack( gl_ush_flashParamDateYear);
    gl_nSentPackIndex = DATE_M;             send_pack( gl_ush_flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;             send_pack( gl_ush_flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;             send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;             send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;             send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;             send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;             send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;             send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;             send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;             send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;             send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;            send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;            send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;            send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;            send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;            send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;            send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;            send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;            send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"мертвый" while
}

void deadloop_no_hangerup( void) {
  //ОБРАБОТКА ОТКАЗА РАСКАЧКИ ВИБРОПОДВЕСА
  double dStartAmplAngCheck = ( double) gl_ush_flashParamAmplAngMin1 / 65535. * 3.;

#ifdef DEBUG
  printf("DBG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x08;     //мы будем посылать ТОЛЬКО AmplAng
  pause(10);
  ADCCON |= 0x80;   //запуск преобразования

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //измерение AmplAng (и ТОЛЬКО ЕГО)
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //запуск преобразования

#ifdef DEBUG
    printf( "DBG: AA: %d = %.2fV    Control value: %.2f\n",
            gl_ssh_ampl_angle,
            3. * ( double) gl_ssh_ampl_angle / 4095.,
            dStartAmplAngCheck);
#endif
    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача данных
    gl_nSentPackIndex = AMPLANG_DUS;        send_pack( gl_ssh_ampl_angle);
    gl_nSentPackIndex = AMPLITUDE;          send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;          send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;            send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;          send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF_CURRENT;   send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;         send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;         send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;         send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = HV_APPLY_COUNT_SET; send_pack( gl_ush_flashParamHvApplyCount);
    gl_nSentPackIndex = HV_APPLY_DURAT_SET; send_pack( gl_ush_flashParamHvApplyDurat);
    gl_nSentPackIndex = HV_APPLY_PACKS;     send_pack( gl_ush_flashParamHvApplyPacks);
    gl_nSentPackIndex = SIGNCOEFF;          send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;             send_pack( gl_ush_flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;             send_pack( gl_ush_flashParamDateYear);
    gl_nSentPackIndex = DATE_M;             send_pack( gl_ush_flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;             send_pack( gl_ush_flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;             send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;             send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;             send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;             send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;             send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;             send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;             send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;             send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;             send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;            send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;            send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;            send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;            send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;            send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;            send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;            send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;            send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  } //"мертвый" захват отказа раскачки виброподвеса
}

void deadloop_no_tact( int nError) {
  //ОБРАБОТКА ОТСУТСТВИЯ ТАКТИРОВАНИЯ
#ifdef DEBUG
  printf("DBG: NO TACT SIGNAL! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //показания приращения угла = 0
  gl_ssh_angle_inc = gl_ssh_angle_inc_prev = 0;

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    processIncomingCommand();

    //выдача настроечных параметров
    gl_nSentPackIndex = AMPLITUDE;          send_pack( gl_ush_flashParamAmplitudeCode);
    gl_nSentPackIndex = TACT_CODE;          send_pack( gl_ush_flashParamTactCode);
    gl_nSentPackIndex = M_COEFF;            send_pack( gl_ush_flashParamMCoeff);
    gl_nSentPackIndex = STARTMODE;          send_pack( gl_ush_flashParamStartMode);
    gl_nSentPackIndex = DECCOEFF_CURRENT;   send_pack( gl_ush_flashParamDecCoeff);
    gl_nSentPackIndex = CONTROL_I1;         send_pack( gl_ush_flashParamI1min);
    gl_nSentPackIndex = CONTROL_I2;         send_pack( gl_ush_flashParamI2min);
    gl_nSentPackIndex = CONTROL_AA;         send_pack( gl_ush_flashParamAmplAngMin1);
    gl_nSentPackIndex = HV_APPLY_COUNT_SET; send_pack( gl_ush_flashParamHvApplyCount);
    gl_nSentPackIndex = HV_APPLY_DURAT_SET; send_pack( gl_ush_flashParamHvApplyDurat);
    gl_nSentPackIndex = HV_APPLY_PACKS;     send_pack( gl_ush_flashParamHvApplyPacks);
    gl_nSentPackIndex = HV_APPLY_COUNT_TR;  send_pack( gl_ushFiringTry);
    gl_nSentPackIndex = SIGNCOEFF;          send_pack( gl_ush_flashParamSignCoeff);
    gl_nSentPackIndex = DEVNUM;             send_pack( gl_ush_flashParamDeviceId);
    gl_nSentPackIndex = DATE_Y;             send_pack( gl_ush_flashParamDateYear);
    gl_nSentPackIndex = DATE_M;             send_pack( gl_ush_flashParamDateMonth);
    gl_nSentPackIndex = DATE_D;             send_pack( gl_ush_flashParamDateDay);
    gl_nSentPackIndex = ORG_B1;             send_pack( gl_ac_flashParamOrg[ 0]);
    gl_nSentPackIndex = ORG_B2;             send_pack( gl_ac_flashParamOrg[ 1]);
    gl_nSentPackIndex = ORG_B3;             send_pack( gl_ac_flashParamOrg[ 2]);
    gl_nSentPackIndex = ORG_B4;             send_pack( gl_ac_flashParamOrg[ 3]);
    gl_nSentPackIndex = ORG_B5;             send_pack( gl_ac_flashParamOrg[ 4]);
    gl_nSentPackIndex = ORG_B6;             send_pack( gl_ac_flashParamOrg[ 5]);
    gl_nSentPackIndex = ORG_B7;             send_pack( gl_ac_flashParamOrg[ 6]);
    gl_nSentPackIndex = ORG_B8;             send_pack( gl_ac_flashParamOrg[ 7]);
    gl_nSentPackIndex = ORG_B9;             send_pack( gl_ac_flashParamOrg[ 8]);
    gl_nSentPackIndex = ORG_B10;            send_pack( gl_ac_flashParamOrg[ 9]);
    gl_nSentPackIndex = ORG_B11;            send_pack( gl_ac_flashParamOrg[ 10]);
    gl_nSentPackIndex = ORG_B12;            send_pack( gl_ac_flashParamOrg[ 11]);
    gl_nSentPackIndex = ORG_B13;            send_pack( gl_ac_flashParamOrg[ 12]);
    gl_nSentPackIndex = ORG_B14;            send_pack( gl_ac_flashParamOrg[ 13]);
    gl_nSentPackIndex = ORG_B15;            send_pack( gl_ac_flashParamOrg[ 14]);
    gl_nSentPackIndex = ORG_B16;            send_pack( gl_ac_flashParamOrg[ 15]);
    gl_nSentPackIndex = VERSION;            send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  }  //"мертвый" while отсутствия тактирования
}