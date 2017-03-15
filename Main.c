#include <ADuC7026.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"
#include "flashEE.h"
#include "deadloops.h"
#include "settings.h"
#include "errors.h"
#include "version.h"

#define SHORT_OUTPUT_PACK_LEN 7       //в норме мы выдаем циклически по 7 пачек с аналог. параметрами               [0,6]
#define LONG_OUTPUT_PACK_LEN  26      //когда запрашивают мы выдаем 25 пачек (со всеми доп. параметрами по очереди) [7,25]
#define DEVICE_NUM_PACK_LEN   28      //когда запрашивают серийный номер прибора мы отдаём два байта с маркерами    [26,27]
#define ORG_NAME_PACK_LEN     44      //когда запрашивают название организации мы выдаём 16 байт с маркерами        [28,43]
#define DATE_PACK_LEN         47      //когда запрашивают дату мы выдаём 3 байта с маркерами                        [44,45,46]
#define HV_APPLY_TIME_LEN     48      //когда запрашивают длительность применения HV мы выдаём 1 байт с маркером    [47]

//#define DEBUG
//#define SKIP_START_CHECKS


//********************
// Decrement coefficient calculation
#define DEC_COEFF_FIRST_CALCULATION_N 100
#define DEC_COEFF_CONTINUOUS_CALCULATION_N 1000
unsigned int gl_un_DecCoeffStatPoints = 0;
double gl_dbl_Nsumm = 0.;
double gl_dbl_Usumm = 0.;
double gl_dbl_Omega;

//********************
//ПАРАМЕТРЫ ХРАНИМЫЕ ВО ФЛЭШ-ПАМЯТИ
//********************
unsigned short flashParamAmplitudeCode = 90;    //амплитуда колебания виброподвеса
unsigned short flashParamTactCode = 0;          //код такта
unsigned short flashParamMCoeff = 4;            //коэффициент ошумления
unsigned short flashParamStartMode = 5;         //начальная мода
unsigned int flashParamDeviceId = 0;            //ID устройства

char flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //название организации

unsigned short flashParamDateYear = 0;          //дата прибора.год
unsigned short flashParamDateMonth = 0;         //дата прибора.месяц
unsigned short flashParamDateDay = 0;           //дата прибора.день

unsigned short flashParamI1min = 0;             //контрольное значение токв поджига I1
unsigned short flashParamI2min = 0;             //контрольное значение тока поджига I2
unsigned short flashParamAmplAngMin1 = 0;       //контрольное значение раскачки с ДУСа
unsigned short flashParamDecCoeff = 0;          //коэффициент вычета
unsigned short flashParamSignCoeff = 2;         //знаковый коэффициент
unsigned short flashParamPhaseShift = 0;        //[OBSOLETE] фазовый сдвиг

//калибровка термодатчиков
signed short flashParam_calibT1;
unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
signed short flashParam_calibT2;
unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;



//********************
//ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ВО ВСЕХ МОДУЛЯХ
//********************
char gl_c_EmergencyCode = 0;    //код ошибки прибора

#define IN_COMMAND_BUF_LEN 3                //длина буфера входящих команд
char input_buffer[6] = { 0, 0, 0, 0, 0, 0}; //буфер входящих команд
char pos_in_in_buf = 0;                     //позиция записи в буфере входящих команд


signed short gl_ssh_SA_time = 0;        //вычисление периода такта.период SA/TA
int gl_n_prT1VAL = 0x1000;   //вычисление периода такта.значение T1 в момент предыдущего такта SA/TA


signed short gl_ssh_angle_inc = 0;      //приращение угла
signed short gl_ssh_angle_inc_prev = 0; //приращение угла
signed short gl_ssh_current_1 = 0;      //разрядный ток 1
signed short gl_ssh_current_2 = 0;      //разрядный ток 2
signed short gl_ssh_Perim_Voltage = 5;  //напряжение контроля пьезокорректоров
signed short gl_ssh_ampl_angle = 0;     //напряжение пропорциональное амплитуде выходного сигнала усилителя датчика угл. скорости
signed short gl_ssh_Utd1 = 0;           //напряжение корпусного термодатчика
signed short gl_ssh_Utd2 = 0;           //напряжение лазерного термодатчика
signed short gl_ssh_Utd3 = 0;           //напряжение лазерного термодатчика
signed short gl_ssh_Utd1_cal = 0;       //напряжение корпусного термодатчика (калибр.)
signed short gl_ssh_Utd2_cal = 0;       //напряжение лазерного термодатчика (калибр.)
signed short gl_ssh_Utd3_cal = 0;       //напряжение лазерного термодатчика (калибр.)


signed short gl_ssh_angle_hanger = 0;       //текущий угол отклонения виброподвеса
signed short gl_ssh_angle_hanger_prev = 0;  //предыдущее занчение угла отклонения виброподвеса

unsigned short gl_ush_MeanImpulses = 1;
#define RULA_MAX 4090
#define RULA_MIN 25

//unsigned char cRulaH = RULA_MAX, cRulaL = RULA_MIN;
//unsigned char cRULAControl = 40; //( RULA_MAX - RULA_MIN) / 2;      127 = 1.25V (Dac0)

//unsigned int gl_un_RULAControl = 0;       //0    = 0.000 V
//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V
//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V
unsigned int gl_un_RULAControl = 3276;      //3276 = 2.000 V
//unsigned int gl_un_RULAControl = 4095;    //4095 = 2.500 V



//unsigned char delta = ( RULA_MAX - RULA_MIN) / 2;
unsigned int nDelta = ( RULA_MAX - RULA_MIN) / 2;

#define VPRC_SLIDING_ROUND 10              //скользящее среднее напряжения системы регулировки периметра
short gl_shVprc[VPRC_SLIDING_ROUND];
int gl_nVrpcCounter = 0;

#define MEANING_IMP_PERIOD_100 100
#define MEANING_IMP_PERIOD_200 200
#define MEANING_IMP_PERIOD_300 300
#define MEANING_IMP_PERIOD_400 400
#define MEANING_IMP_PERIOD_500 500
#define MEANING_IMP_PERIOD_STABLE 1000

int gl_sn_MeaningCounter = 0;
int gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
double dMeaningSumm = 0.;
double dMeanImps = 0.;
int nT2RepeatBang;

char gl_b_PerimeterReset = 0;         //флаг сигнала сброса периметра (0 = данные достоверны, 1 = данные НЕдостоверны, прошло выключение интегратора, 2 = данные НЕдостоверны, прошло включение интегратора)

char gl_b_SA_Processed = 0;           //флаг окончания обработки сигнала SA
char gl_b_SyncMode = 0;               //флаг режима работы гироскопа:   0=синхр. 1=асинхр.
char bAsyncDu = 0;                    //флаг передачи времени SA или приращ. угла в асинхр. режиме: 0-передается SA 1-передается dU

short nSentPacksCounter = 0;          //счетчик посылок
int nSentPacksRound = SHORT_OUTPUT_PACK_LEN;    //круг счетчика посылок
char gl_c_OutPackCounter = 0;         //выдаваемый наружу счётчик посылок

int ADCChannel = 1;     //читаемый канал АЦП
                    //0 = ADC1 = 78 нога = UTD1
                    //1 = ADC2 = 79 нога = UTD2
                    //2 = ADC3 = 80 нога = I1
                    //3 = ADC4 =  1 нога = I2
                    //4 = ADC5 =  2 нога = CntrPC
                    //5 = ADC6 =  3 нога = AmplAng

#define BIT_0 1
#define BIT_1 2
#define BIT_2 4
#define BIT_3 8
#define BIT_4 16
#define BIT_5 32
#define BIT_6 64
#define BIT_7 128


double dStartAmplAngCheck = 0.5;

unsigned short nFiringTry = 0;


char bCalibProcessState;

char bCalibrated;
double TD1_K, TD1_B;
double TD2_K, TD2_B;
double TD3_K, TD3_B;

//2014-08-27 - enabling external oscillator
static int new_clock_locked = 0;

char gl_bOutData = 0;       //флаг разрешения выдачи данных наружу

//Система контроля просадок токов
char gl_bManualLaserOff = 0;          //флаг что мы сами выключили ток лазера (командой). Флаг нужен чтобы исключить это событие в отслеживателе просадок тока.
int gl_nLaserCurrentUnstableT2 = 0;   //последний момент времени когда была просадка (отслеживается, и если за последние 5 сек просадок не было - сбрасывается)
int gl_nLaserCurrentUnstableCnt = 0;  //счётчик просадок в течении последних 5 сек

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//обработчик прерываний
void FIQ_Handler (void) __fiq
{
  if( ( FIQSTA & UART_BIT) != 0)	{
    if( pos_in_in_buf < IN_COMMAND_BUF_LEN)
      input_buffer[ pos_in_in_buf++] = COMRX;
    //GP0DAT = ( 1 << 16);
  }
}

void IRQ_Handler (void) __irq
{
  //int i;

/*
#ifdef DEBUG
  printf("IRQ i\n");
#endif
*/
  /*
  //В тестовых целях делаем сигнал на линии P3.7
  GP3DAT |= 1 << ( 16 + 7); //тестовая линия p3.7 set
  for( i=0; i<100; i++);
  GP3DAT &= ~( 1 << ( 16 + 7)); //тестовая линия p3.7 clear
  */

  if (IRQSIG & 0x00000010) {
    /* disable timer, clear interrupt flag */
    T2CON   = 0x00;
    T2CLRI  = 0x01;
    new_clock_locked = 1;
  }
/*#ifdef DEBUG
  printf("IRQ o\n");
#endif
*/
}

double CalcSlidingAverage( void) {
  int i;
  double summ = 0.;
  for( i=0; i<VPRC_SLIDING_ROUND; summ += gl_shVprc[i++]);
  return (summ / VPRC_SLIDING_ROUND);
}

void pause_T0( int n) {
  unsigned int prval, chk;

/*
#ifdef DEBUG
  printf("pause() in\n");
#endif
*/

  prval = T0VAL;
  chk = (( 0xFFFF + prval - T0VAL) % 0xFFFF);
  while( chk < n)
    chk = (( 0xFFFF + prval - T0VAL) % 0xFFFF);
/*
#ifdef DEBUG
  printf("pause() out\n");
#endif
*/
}

void pause( int n) {
  unsigned int prval, chk;

/*
#ifdef DEBUG
  printf("pause() in\n");
#endif
*/

  prval = T2VAL;
  chk = (( T2LD + prval - T2VAL) % T2LD);
  while( chk < n)
    chk = (( T2LD + prval - T2VAL) % T2LD);
/*
#ifdef DEBUG
  printf("pause() out\n");
#endif
*/
}

double round( double val) {
  double lstd = val - floor( val);
  if( lstd < .5) return floor( val);
  else return ceil( val);
}

/*void PrintString( char*ptr, int n) {
}*/

void send_pack( signed short angle_inc1, short param_indicator, short analog_param) {
  char cCheckSumm = 0;
  char *pntr;
  char b1, b2, b3, b4;

#ifndef DEBUG
  //float angle_inc_corr;
  signed short angle_inc_corr;
  float f_dN;
  double dbl_dN;

  float Coeff = (( float) flashParamDecCoeff) / 65535.;

  signed short ssh_dN, ssh_dU;
  signed int n_dN, n_dU;
  double result;

/*
	char b1 = 0, b2 = 0;

	b1 = angle_inc1 & 0xff;
	b2 = ( angle_inc1 & 0xff00) >> 8;

	PrintHexShortNumber( angle_inc1);
	putchar('=');
	PrintHexCharNumber( b2);
	putchar(' ');
	PrintHexCharNumber( b1);
	putchar(' ');
	putchar(' ');
	PrintHexShortNumber( param_indicator);
	putchar(' ');
	PrintHexShortNumber( analog_param);
	putchar('\n');*/


/*
  //**********************************************************************
	// Наебка (сделано когда у нас ибанулся лазер)
	//**********************************************************************
  angle_inc1 = ( short) ( ( double) rand() / 32767. * 4.) - 2;
  switch( param_indicator) {
    case 0: analog_param = ( short) ( ( double) rand() / 32767. * 2.); break;             //0-1
    case 1: analog_param = ( short) ( ( double) rand() / 32767. * 2.); break;             //0-1
    case 2: analog_param = 2700 + ( short) ( ( double) rand() / 32767. * 100.); break;    //2700-2800
    case 3: analog_param = 2700 + ( short) ( ( double) rand() / 32767. * 2.); break;      //2700-2800
    case 4: analog_param = ( short) ( ( double) rand() / 32767. * 1.); break;             //0-1
    case 5: analog_param = 480 + ( short) ( ( double) rand() / 32767. * 20.); break;      //480-500
  }                                                             
  gl_ssh_SA_time = 78 + ( short) ( ( double) rand() / 32767. * 4.);
*/

  //***************************************************************************
	//START_MARKER
  //***************************************************************************
	putchar_nocheck( 0x55);
	putchar_nocheck( 0xAA);
	
  //***************************************************************************
  //    dN
  //***************************************************************************
  if( gl_b_SyncMode) {
    if( bAsyncDu) {
      //АСИНХР: выдача dN-dU
      signed int siAngleInc1 = ( signed short) angle_inc1;
      //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
      angle_inc_corr = angle_inc1;
      f_dN = ( float) ( ( signed int) angle_inc1);
      dbl_dN = ( double) ( ( signed int) angle_inc1);
    }
    else {
      //АСИНХР: нормальный режим
      /*double db_dN1 = ( double) gl_ssh_angle_inc_prev;
      double db_dN2 = ( double) gl_ssh_angle_inc;
      double dbU1 = ( double) gl_ssh_angle_hanger_prev;
      double dbU2 = ( double) gl_ssh_angle_hanger;*/

      ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
      ssh_dU = gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev;

      n_dN = ( signed int) ssh_dN;
      n_dU = ( signed int) ssh_dU;

      result =  ( double) n_dN - ( ( double) n_dU) * Coeff * (( signed short) flashParamSignCoeff - 1);
      //printf("\n%.3f %.3f %.3f %.3f %.3f\n", db_dN1, db_dN2, dbU1, dbU2, result);
      angle_inc_corr = ( signed short) ( result * 100.);

      f_dN = ( float) ( ( signed short) result);
      dbl_dN = ( double) ( ( signed short) result);
    }
  }
  else {
    //СИНХРОННЫЙ РЕЖИМ
    double dAngleInc1 = ( double) angle_inc1;
    //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
    angle_inc_corr = ( signed short) ( angle_inc1 * 100.);

    f_dN = ( float) ( ( signed int) angle_inc1);
	  dbl_dN = ( double) ( ( signed int) angle_inc1);
  }

  /*  
  putchar_nocheck( angle_inc_corr & 0xff);
  putchar_nocheck( ( angle_inc_corr & 0xff00) >> 8);*/

  //размазываем f_dN на диапазон [-99 310; + 99 310]
  dbl_dN = ( dbl_dN / 99310.) * 2147483647.;
  n_dN = ( int) dbl_dN;

  pntr = ( char *) &n_dN;
  b1 = pntr[0];
  b2 = pntr[1];
  b3 = pntr[2];
  b4 = pntr[3];

  putchar_nocheck( b1);   cCheckSumm += b1;
  putchar_nocheck( b2);   cCheckSumm += b2;
  putchar_nocheck( b3);   cCheckSumm += b3;
  putchar_nocheck( b4);   cCheckSumm += b4;


  //***************************************************************************
  //ANALOG PARAMETER INDICATOR
  //***************************************************************************
  putchar_nocheck( ( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff);
  cCheckSumm += (( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  /*
  if( param_indicator == 16) {
    putchar_nocheck( VERSION_MAJOR * 16 + VERSION_MIDDLE);
    cCheckSumm += ( VERSION_MAJOR * 16 + VERSION_MIDDLE);

  
    putchar_nocheck( VERSION_MINOR * 16);// + 1 + HIRO_COEFF);
    cCheckSumm += ( VERSION_MINOR * 16);
  }
  else {
  */
    putchar_nocheck( analog_param & 0xff);
    cCheckSumm += (analog_param & 0xff);

    putchar_nocheck( ( analog_param & 0xff00) >> 8);
    cCheckSumm += ( ( analog_param & 0xff00) >> 8);
  //}

  //***************************************************************************
  //синхр. режим: SA TIME
  //асинхр. режим: SA TIME или приращение угла поворота
  if( gl_b_SyncMode) {
    //асинхр. режим
    if( bAsyncDu) {
      //передаем dU
      putchar_nocheck( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);
      cCheckSumm += ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);

      putchar_nocheck( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
      cCheckSumm += ( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
    }
    else {
      //передаем SA
      putchar_nocheck( gl_ssh_SA_time & 0xff);
      cCheckSumm += ( gl_ssh_SA_time & 0xff);

      putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
      cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
    }
  }
  else {
    //синхронный режим
    putchar_nocheck( gl_ssh_SA_time & 0xff);
    cCheckSumm += ( gl_ssh_SA_time & 0xff);

    putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
    cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
  }

  //***************************************************************************
  //PACK COUNTER
  //***************************************************************************
  putchar_nocheck( gl_c_OutPackCounter);
  cCheckSumm += gl_c_OutPackCounter;

  gl_c_OutPackCounter++;

  //***************************************************************************
  //EMERGENCY CODE
  //***************************************************************************
  putchar_nocheck( gl_c_EmergencyCode);
  cCheckSumm += gl_c_EmergencyCode;

  //***************************************************************************
  // CHECKSUM
  //***************************************************************************
  putchar_nocheck( cCheckSumm);

#else
  //if( param_indicator > 10)

  angle_inc1 = ( short) ( ( double) rand() / 32767. * 10.) - 5;
  
  pntr = ( char *) &angle_inc1;
  b1 = pntr[0];
  b2 = pntr[1];
  b3 = pntr[2];
  b4 = pntr[3];
  
  putchar_nocheck('.');
  /*
  printf("(0x55 0xAA) (0x%02x 0x%02x 0x%02x 0x%02x) (0x%02x) (0x?? 0x??) (0x?? 0x??) (0x%02x) (0x%02x) (0x??)\n",
          b1, b2, b3, b4,
          ( gl_b_PerimeterReset ? 0x80 : 0x00) + param_indicator & 0xff,
          gl_c_OutPackCounter++,
          gl_c_EmergencyCode
          );
   */
  
#endif

}

void configure_hanger( void) {

  //1. Код такта подставки
  //Выставка TactNoise0 (младший бит параметра "код такта подставки")
  if( ( flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //Выставка TactNoise1 (старший бит параметра "код такта подставки")
  if( ( flashParamTactCode & 0x02))  //Set TactNoise1 to TactCode parameter bit1
    GP3DAT |= ( 1 << (16 + 1));
  else
    GP3DAT &= ~( 1 << (16 + 1));
}

void DACConfiguration( void) {
/*#ifdef DEBUG
  printf("DBG: DACConfiguration(): enter\n");
#endif
*/
  //**********************************************************************
  // выставка ЦАПов
  //**********************************************************************
  // ЦАП 0
  DAC0DAT = (( int) ( 4095.0 * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 ) / 3.0)) << 16; //выставка на выходе ЦАП0 1,0 В
  // ЦАП 1 (мода)
  DAC1DAT = (( int) ( 4095.0 * ( ( double) flashParamMCoeff / 250. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 )) / 3.0)) << 16;  //(1.0) - это RULA в вольтах который на DAC0
  //DAC1DAT = (( int) ( 4095.0 * ( ( double) flashParamParam3 / 250. * 0.25) / 3.0)) << 16;  //(1.0) - это RULA в вольтах который на DAC0
  // ЦАП 2 (начальная мода)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) flashParamStartMode / 250. * 2.5) / 3.0)) << 16;
}

void FirstDecrementCoeffCalculation( void) {
  char lb, hb;
  int i, k, val;

  //****
  //ПЕРВЫЙ ПРОХОД (нам ведь дельты нужны)
  //****

  //выжидаем нарастающий фронт 
  while( !(GP0DAT & 0x10));

  //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
  //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
  // запуск преобразования угла поворота подвеса cnvst->0
  GP3CLR |= 1 << (16 + 7);

  //Импульс WrCnt
  GP4DAT ^= 1 << (16 + 2);
  //pause( 1);
  GP4DAT ^= 1 << (16 + 2);

  //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
  //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
  // тут же cnvst ->1 (чтобы внешний АЦП не "выпадал" в shutdown по окончанию преобразования)
  GP3SET |= 1 << (16 + 7);


  // Получение приращения угла:
  //получение старшего байта приращения угла
  GP1SET |= 1 << (16 + 3);  //rdhbc set
  //pause( 1);

  hb = (( GP1DAT & BIT_5) >> 5) +
       ((( GP2DAT & BIT_1) >> 1) << 1) +
       ((( GP0DAT & BIT_1) >> 1) << 2) +
       ((( GP2DAT & BIT_3) >> 3) << 3) +
       ((( GP4DAT & BIT_6) >> 6) << 4) +
       ((( GP4DAT & BIT_7) >> 7) << 5) +
       ((( GP0DAT & BIT_6) >> 6) << 6) +
       ((( GP0DAT & BIT_2) >> 2) << 7);

  GP1CLR |= 1 << (16 + 3);  //rdhbc clear

  //получение младшего байта приращения угла
  GP1SET |= 1 << (16 + 4);  //rdlbc set
  //pause( 1);

  lb = (( GP1DAT & BIT_5) >> 5) +
       ((( GP2DAT & BIT_1) >> 1) << 1) +
       ((( GP0DAT & BIT_1) >> 1) << 2) +
       ((( GP2DAT & BIT_3) >> 3) << 3) +
       ((( GP4DAT & BIT_6) >> 6) << 4) +
       ((( GP4DAT & BIT_7) >> 7) << 5) +
       ((( GP0DAT & BIT_6) >> 6) << 6) +
       ((( GP0DAT & BIT_2) >> 2) << 7);

  GP1CLR |= 1 << (16 + 4);	//rdlbc clear

  gl_ssh_angle_inc_prev = lb + (hb << 8);

  // Ожидание результата измерение угла отклонения подвеса (асинхр. режим)
  gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
  while( ( GP2DAT & 0x80)) {}                                 //ждем пока busy не упадет в 0

  //читаем
  GP1CLR |= 1 << (16 + 7);                                    //cs->0
  for( k=0; k<100; k++);                                      //выждать t4
  GP2SET |= 1 << (16 + 2);                                    //sclk->1
  val = 0;

  //читаем 14 bit
  for( i=0; i<14; i++) {
    GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0
    for( k=0; k<50; k++);                                    //выждать t5
    val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //читаем бит
    GP2DAT |= 1 << (16 + 2);                                  //sclk->1
    for( k=0; k<50; k++);                                    //выждать t8
  }

  GP1SET |= 1 << (16 + 7);                                    //cs->1
  gl_ssh_angle_hanger_prev = val << 2;
  gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger_prev / 4;

  //выжидаем спадающий фронт
  while( (GP0DAT & 0x10));

  //****
  //И ПОСЛЕДУЮЩИЕ
  //****
  do {
    //выжидаем нарастающий фронт
    while( !(GP0DAT & 0x10));

    //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
    //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
    // запуск преобразования угла поворота подвеса cnvst->0
    GP3CLR |= 1 << (16 + 7);


    //Импульс WrCnt
    GP4DAT ^= 1 << (16 + 2);
    //pause( 1);
    GP4DAT ^= 1 << (16 + 2);

    //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
    //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
    // тут же cnvst ->1 (чтобы внешний АЦП не "выпадал" в shutdown по окончанию преобразования)
    GP3SET |= 1 << (16 + 7);


    // Получение приращения угла

    //получение старшего байта приращения угла
    GP1SET |= 1 << (16 + 3);  //rdhbc set
    //pause( 1);

    hb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP2DAT & BIT_1) >> 1) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);


    GP1CLR |= 1 << (16 + 3);  //rdhbc clear

    //получение младшего байта приращения угла
    GP1SET |= 1 << (16 + 4);  //rdlbc set
    //pause( 1);

    lb = (( GP1DAT & BIT_5) >> 5) +
         ((( GP2DAT & BIT_1) >> 1) << 1) +
         ((( GP0DAT & BIT_1) >> 1) << 2) +
         ((( GP2DAT & BIT_3) >> 3) << 3) +
         ((( GP4DAT & BIT_6) >> 6) << 4) +
         ((( GP4DAT & BIT_7) >> 7) << 5) +
         ((( GP0DAT & BIT_6) >> 6) << 6) +
         ((( GP0DAT & BIT_2) >> 2) << 7);

    GP1CLR |= 1 << (16 + 4);  //rdlbc clear

    gl_ssh_angle_inc = lb + (hb << 8);

    // Ожидание результата измерение угла отклонения подвеса (асинхр. режим)
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
    while( ( GP2DAT & 0x80)) {}                                 //ждем пока busy не упадет в 0

    //читаем
    GP1CLR |= 1 << (16 + 7);                                    //cs->0
    for( k=0; k<100; k++);                                      //выждать t4
    GP2SET |= 1 << (16 + 2);                                    //sclk->1
    val = 0;

    //читаем 14 bit
    for( i=0; i<14; i++) {
      GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0
      for( k=0; k<50; k++);                                    //выждать t5
      val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //читаем бит
      GP2DAT |= 1 << (16 + 2);                                  //sclk->1
      for( k=0; k<50; k++);                                    //выждать t8
    }

    GP1SET |= 1 << (16 + 7);                                    //cs->1
    gl_ssh_angle_hanger = val << 2;
    gl_ssh_angle_hanger = gl_ssh_angle_hanger / 4;
      
    //выжидаем спадающий фронт
    while( (GP0DAT & 0x10));

    gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
    gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);

    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

    gl_un_DecCoeffStatPoints++;

  } while( gl_un_DecCoeffStatPoints < DEC_COEFF_FIRST_CALCULATION_N);

  //считаем собсно значение коэф. вычета
  flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

void SendPhaseShift( void) {
  int i, k, base = 1;

  //импульс сброса
  GP2DAT |= 1 << (16 + 6);                                  //CLR (p2.6) -> 1
  for( k=0; k<100; k++);
  GP2DAT &= ~( 1 << (16 + 6));                              //CLR (p2.6) -> 0
  for( k=0; k<100; k++);

  for( i=0; i<6; i++) {
    //выставка БИТИКА на линию DATA
    if( flashParamPhaseShift & base) {
      GP2DAT |= 1 << (16 + 5);                              //DATA (p2.5) -> 1
    }
    else {
      GP2DAT &= ~( 1 << (16 + 5));                          //DATA (p2.5) -> 0
    }

    //тактирующий импульс SCK
    GP0DAT |= 1 << (16 + 3);                                //SCK (p0.3) -> 1
    for( k=0; k<100; k++);
    GP0DAT &= ~( 1 << (16 + 3));                            //SCK (p0.3) -> 0
    for( k=0; k<100; k++);
    
    base *= 2;
  }
}

void ThermoCalibrationCalculation( void)
{
  double x1, y1, x2, y2;
  //рассчёт калибровки термодатчиков
  if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
      flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION) &&

      flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) &&
      flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {


    //TD1
    x1 = flashParam_calibT1 - THERMO_CALIB_PARAMS_BASE;
    x2 = flashParam_calibT2 - THERMO_CALIB_PARAMS_BASE;

    y1 = flashParamT1_TD1_val;
    y2 = flashParamT2_TD1_val;

    TD1_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD1_K = ( y2 - y1) / ( x2 - x1);


    //TD2
    y1 = flashParamT1_TD2_val;
    y2 = flashParamT2_TD2_val;

    TD2_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD2_K = ( y2 - y1) / ( x2 - x1);


    //TD3
    y1 = flashParamT1_TD3_val;
    y2 = flashParamT2_TD3_val;

    TD3_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD3_K = ( y2 - y1) / ( x2 - x1);

    bCalibrated = 1;
  }
  else
    bCalibrated = 0;
}


void main() {
  //unsigned short ush_SA_check_time;

  short in_param_temp;
  int nRppTimer = 0;
  //unsigned char jchar = 0x30; 
  int time = 20000;
  int i, k, val;
  char lb, hb;
  short sSrc = -10;
  unsigned short unSrc = 10;

  int nSrc = 10;
  int nDst = 0;
  float fSrc = 10.2345;
  float fDst = 0.;
  /*char *pntr;
  char b1, b2, b3, b4;*/

  /*short q;
  unsigned short q1;
  unsigned int i1;*/

  char bSAFake = 0;
  int prt2val;

  double db_dN1, db_dN2, dbU1, dbU2;
  float Coeff;

  double dbl_V_piezo = 0.;
  double temp_t;

  //РЕЖИМ dN_dU_Cnt
  char bSimpleDnDu = 0;         //режим выдачи dN,dU,counter: флаг      0-обычный режим (выключен), 1-режим dN_dU_C (включен)
  char cSimpleDnDuCounter = 0;  //режим выдачи dN,dU,counter: счётчик пачек
  signed short ssh_dN, ssh_dU;
  char cCheckSumm = 0;

  //int n;
  int nT2VALold;      //2014-08-27 - enabling external oscillator



  bCalibProcessState = 0;    //0 - no calibration
                             //1 - processing min_t_point 1st thermo
                             //2 - processing min_t_point 2nd thermo
                             //3 - processing max_t_point 1st thermo
                             //4 - processing max_t_point 2nd thermo

  // Setup tx & rx pins on P1.0 and P1.1
  GP0CON = 0x00;
  GP1CON = 0x011;       //*** 0x011 = 0001 0001 = 00 01 00 01
                        //*** 01 - Выбор функции вывода порта P1.0
                        //*** 00 - Reserved
                        //*** 01 - Выбор функции вывода порта P1.1
                        //*** 00 - Reserved
                        //*** Функция:
                        //***         00    01    10    11
                        //***   P1.0  GPIO  SIN   SCL0  PLAI[0]
                        //***   P1.1  GPIO  SOUT  SDA0  PLAI[1]

  // Start setting up UART
  COMCON0 = 0x080;      // Setting DLAB

  // Setting DIV0 and DIV1 to DL calculated


  //Рассчёт:
  //DL = 41.78 * 10^6 / (2 ^ (CD=0) * 16 * 2 * Baudrate)
  //RESULT = (M+N/2048) = 41.78 * 10^6 / ( 2 ^ (CD=0) * 16 * 2 * DL * 2)
  // M = Floor ( RESULT)
  // N = (RESULT - M) * 2048


/*
  //WORK PARAMETERS FOR PRECISE 115200
  COMDIV0 = 0x0B;
  COMDIV1 = 0x00;
  COMDIV2 = 0x0029;
*/

/*
  //WORK PARAMETERS FOR PRECISE 256000
  //DL = 41.78e6 / ( [2^CD]=1 * 16 * 2 * 256000) = 5.10009765625 = 5
  //M + N/2048 = 41.78 * 10^6 / ( 256000 * 2 ^ (CD=0) * 16 * DL * 2) = 41.78e6 / (256000 * 32 * 5) = 1.02001953125

  //M = 1  = 0x01
  //N = 41 = 0x029 = @000 0010 1001

  //COMDIV0 = DL = 0x05
  //COMDIV2 = 
  //    8            8
  //1 0 0 [0    1] [0 0 0
  //0 0 1  0    1   0 0 1]
  //    2            9
  //= 0x8829

  COMDIV0 = 0x05;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8829;
*/


  /*
  //WORK PARAMETERS FOR PRECISE 460800
  //DL = 41.78e6 / ([2^CD]=1 * 16 * 2 * 460800) = 2.8333875868055555555555555555556 = 2
  //M + N/2048 = 41.78 * 10^6 / ( 460800 * 2 ^ (CD=0) * 16 * DL * 2) = 41.78e6 / (460800 * 32 * 2) = 1.4166937934027777777777777777778

  //M = 1
  //N = 853 = 0x355

  //COMDIV0 = DL = 2 = 0x02
  //COMDIV2 = 
  //    8            B
  //1 0 0 [0    1] [0 1 1
  //0 1 0  1    0   1 0 1]
  //    5            5
  //  = 0x8B55

  COMDIV0 = 0x02;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8B55;
  */

  
  //WORK PARAMETERS FOR PRECISE 512000
  //DL = 41.78e6 / ([2^CD]=1 * 16 * 2 * 512000) = 2.550048828125 = 2
  //M + N/2048 = 41.78 * 10^6 / ( 512000 * 2 ^ (CD=0) * 16 * DL * 2) = 41.78e6 / (512000 * 32 * 2) = 1.2750244140625

  //M = 1
  //N = 563 = 0x233

  //COMDIV0 = DL = 2 = 0x02
  //COMDIV2 = 
  //    8            A
  //1 0 0 [0    1] [0 1 0
  //0 0 1  1    0   0 1 1]
  //    3            3
  //  = 0x8A33
  COMDIV0 = 0x02;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8A33;


  COMCON0 = 0x007;      // Clearing DLAB

#ifdef DEBUG
  printf("DBG MODE\n");
  printf("T39-SLG. (C) SLC Alcor Laboratories, 2016.\n");
  printf("Software version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MIDDLE, VERSION_MINOR);

  /*
  gl_sn_MeaningCounterRound = LONG_OUTPUT_PACK_LEN;
  printf("LONG_PACK: %d\n", gl_sn_MeaningCounterRound);
  gl_sn_MeaningCounterRound = SHORT_OUTPUT_PACK_LEN;
  printf("SHORT_PACK: %d\n", gl_sn_MeaningCounterRound);
  */
#endif

  //**********************************************************************
  // Конфигурация лампочки
  //**********************************************************************	
  GP0DAT = 0x01000000;
  GP0DAT ^= (1 << 16+0);
  GP0CLR = (1 << 16+0);

#ifdef DEBUG
  printf("DBG: GPIO lines direction configuration\n");
#endif

  //**********************************************************************
  // Конфигурация GPIO (двунаправленных входов/выходов общего назначения)
  //**********************************************************************
  GP0DAT |= 1 << (24 + 0);	//Конфигурация тестовой линии (лампочка) (p0.0) в качестве выхода
  GP0DAT |= 1 << (24 + 3);	//Конфигурация линии тактирования сдвига (p0.3) в качестве выхода
  GP0DAT |= 1 << (24 + 5);	//Конфигурация линии RP_P                (p0.5) в качестве выхода

  GP1DAT |= 1 << (24 + 2);	//Конфигурация линии RdCodeDac           (p1.2) в качестве выхода
  GP1DAT |= 1 << (24 + 3);	//Конфигурация линии RdHbc               (p1.3) в качестве выхода
  GP1DAT |= 1 << (24 + 4);	//Конфигурация линии RdLbc               (p1.4) в качестве выхода
  GP1DAT |= 1 << (24 + 7);	//Конфигурация линии CS                  (p1.7) в качестве выхода

  /*
  2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
  GP2DAT |= 1 << (24 + 1);	//Конфигурация линии CnvSt               (p2.1) в качестве выхода
  */

  GP2DAT |= 1 << (24 + 2);	//Конфигурация линии SClck               (p2.2) в качестве выхода	
  GP2DAT |= 1 << (24 + 4);	//Конфигурация линии RdN10N8             (p2.4) в качестве выхода
  GP2DAT |= 1 << (24 + 5);	//Конфигурация линии выдачи сдвига Data  (p2.5) в качестве выхода
  GP2DAT |= 1 << (24 + 6);	//Конфигурация линии сброса сдвига CLR   (p2.6) в качестве выхода

  GP3DAT |= 1 << (24 + 0);	//Конфигурация линии TactNoise0          (p3.0) в качестве выхода
  GP3DAT |= 1 << (24 + 1);	//Конфигурация линии TactNoise1          (p3.1) в качестве выхода 
  GP3DAT |= 1 << (24 + 3);	//Конфигурация линии RdN7N0              (p3.3) в качестве выхода
  //GP3DAT |= 1 << (24 + 4);	//Конфигурация линии Start               (p3.4) в качестве выхода
  GP3DAT |= 1 << (24 + 5);	//Конфигурация линии OutLnfType          (p3.5) в качестве выхода 
  GP3DAT |= 1 << (24 + 7);	//Конфигурация тестовой линии            (p3.7) в качестве выхода

  GP4DAT |= 1 << (24 + 0);	//Конфигурация линии ONHV                (p4.0) в качестве выхода
  GP4DAT |= 1 << (24 + 1);	//Конфигурация линии OFF3KV              (p4.1) в качестве выхода
  GP4DAT |= 1 << (24 + 2);	//Конфигурация линии WrCnt               (p4.2) в качестве выхода
  GP4DAT |= 1 << (24 + 3);	//Конфигурация линии Reset               (p4.3) в качестве выхода 

#ifdef DEBUG
  printf("DBG: GPIO lines values configuration...\n");
#endif

  //начальное значение интегратора - выключен
  GP0DAT |= 1 << (16 + 5);      //RP_P       (p0.5) = 1
  GP0SET = 1 << (16 + 5);       //дублёр

  GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
  GP4SET = 1 << (16 + 0);      //дублёр

  GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
  GP4SET = 1 << (16 + 1);      //дублёр

  GP4DAT &= ~( 1 << (16 + 2));  //WrCnt      (p4.2) = 0
  GP4CLR = ( 1 << (16 + 2));   //дублёр

  GP2DAT &= ~( 1 << (16 + 4));  //RdN10N8    (p2.4) = 0
  GP2CLR = 1 << (16 + 4);      //дублёр

  GP3DAT &= ~( 1 << (16 + 3));  //RdN7N0     (p3.3) = 0
  GP3CLR = 1 << (16 + 3);      //дублёр

#ifdef DEBUG
  printf("DBG: Pulsing Reset signal...\n");
#endif

  //**********************************************************************
  // Посылка сигнала Reset
  //**********************************************************************
  GP4DAT |= 1 << (16 + 3);      //Reset set
  for( i=0; i<100; i++);
  GP4DAT &= ~( 1 << (16 + 3));  //Reset clear

#ifdef DEBUG
  printf("DBG: Enabling UART0 FIQ...\n");
#endif

  //**********************************************************************
  // Включение прерывания по UART0
  //**********************************************************************	
  FIQEN |= UART_BIT;
  COMIEN0 |= 1;

#ifdef DEBUG
  printf("DBG: Internal ADC configuration...\n");
#endif

  //**********************************************************************
  // Конфигурация АЦП
  //**********************************************************************	
  ADCCON = 0x20;            // включаем АЦП
  while (time >=0)	        // ждем указанное в datasheet время (5мксек) для полного включения АЦП
    time--;

  ADCCON = 0x624;           // Конфигурируем АЦП:
                            // непрерывное преобразование с управлением от программы
                            // однополярный вход
                            // (включенное питание АЦП)
                            // запрещенный ADCBusy
                            // без старта преобразования
                            // преобразование 8 тактов
                            // тактирование [fADC / 4]
  ADCCP = 0x01;             // ставим 1ый канал АЦП
  REFCON = 0x00;            // отключаем внутренний ИОН от пина Vref


#ifdef DEBUG
  printf("DBG: External ADC AD7367 configuration...\n");
#endif

  //**********************************************************************
  // Конфигурация АЦП AD7367
  //**********************************************************************	
  //конфигурим ноги DOUT, BUSY на вход
  //они и так на вход по дефолту
  //конфигурим ноги SCLK, CNVST, CS на выход
  //...(это было тут)

  GP1DAT |= 1 << (16 + 7);                                      //cs->1
  GP2DAT &= ~( 1 << (16 + 2));                                  //sclk->0



  //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
  //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
  //включаемся (на всякий случай выжидаем t power-up (70 usec))
  GP3DAT |= 1 << (16 + 7);                                      //cnvst -> 1
  for( k=0; k<1000; k++);                                       //70 usec



#ifdef DEBUG
  printf("DBG: Timers configuration...\n");
#endif
  //**********************************************************************
  // Конфигурация Timer0
  //**********************************************************************
  T0CON = 0x80;

  //**********************************************************************
  // Конфигурация Timer1
  //**********************************************************************
  T1CON = 0x0C4;
  //0x0C4 = 0000 1100 0100
  // 000 0   1 1 00   0100
  // |   |   | | |    |------ SourceClock/16
  // |   |   | | |----------- Binary
  // |   |   | -------------- Periodic
  // |   |   ---------------- Enable
  // |   -------------------- Count down
  // ------------------------ CoreClock (41.78 Mhz)
  T1LD = 0xFFFFFFFF;

  //**********************************************************************
  // Конфигурация внешнего осциллятора
  //**********************************************************************
/*#ifdef DEBUG
  printf("PLLCON=%x. Before change.\n", PLLCON);
#endif*/


  /*
  while(1) {
    for( i=0; i<50; i++) {
      GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
      GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear
    }
    GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
    pause( 1);
    GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear
    pause( 2);
    GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
    pause( 3);
    GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear
    pause_T0( 100);
    GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
    pause_T0( 500);
    GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear
    pause_T0( 2500);
  }*/


  /* configuring Timer2 for Wake-up */
  nT2VALold = T2VAL;
  T2LD = 0x050;   // 5; 5 clocks @32.768kHz = 152ms
  T2CON = 0x480;  // Enable Timer2;

  //while ((T2VAL == nT2VAL_old) || (T2VAL > 3)); // ensures timer is started...

  /* enabling Timer2 interrupt */
  IRQEN = 0x10;   // enable TIMER2 Wakeup IRQ

  /* setting power into NAP mode */
  PLLKEY1 = 0xAA;
  PLLCON  = 0x01;  // External Osc. Select.
  PLLKEY2 = 0x55;

  POWKEY1 = 0x01;
  POWCON  = 0x20;  // NAP Mode, 326kHZ Clock Divider
  POWKEY2 = 0xF4;



  /* waiting for switch to new clock */
  while (!new_clock_locked);

  /* disabling Timer2 interrupt */
  //IRQEN &= ~(0x10);   // WRONG-WAY disable TIMER2 Wakeup IRQ
  IRQCLR |= 0x10;     // 2017.02.02 RIGHT-WAY disable TIMER2 Wakeup IRQ

/*
#ifdef DEBUG
  printf("PLLCON=%x. After change.\n", PLLCON);
#endif
*/

  //**********************************************************************
  // Конфигурация Timer2
  //**********************************************************************
  T2CON = 0x2C0;
  //0x2C0 = 0010 1100 0000
  // 0 01 0   1 1 00   0000
  //   |  |   | | |    |------ SourceClock/1
  //   |  |   | | |----------- Binary
  //   |  |   | -------------- Periodic
  //   |  |   ---------------- Enable
  //   |  -------------------- Count down
  //   ----------------------- External Crystal
  T2LD = 0x0FFFFFFF;

#ifdef DEBUG
  printf("DBG: FlashEE configuration\n");
#endif

  /*
  //********************************************************************
  // Тест жизни процессора
  //********************************************************************
  n = 0;
  while(1) {
    printf("%d ", n);
    if( ( ++n) % 2)
      GP3DAT &= ~( 1 << (16 + 7));  //(p3.7) = 0
    else
      GP3DAT |= ( 1 << (16 + 7));   //(p3.7) = 1

    //pause 1 sec
    pause( 32768);
  }
  */

  //**********************************************************************
  // Конфигурация флэш-памяти FlashEE
  //**********************************************************************
  flashEE_configure();

#ifdef DEBUG
  printf("DBG: loading flash params.\n");
#endif

  //**********************************************************************
  // Загрузка параметров из флэш-памяти
  //**********************************************************************
  load_params();
  ThermoCalibrationCalculation();

#ifdef DEBUG
  printf("DBG: DAC Configuration\n");
#endif

  //**********************************************************************
  // Конфигурация и выставка ЦАП
  //**********************************************************************
  // ЦАП 0
  DAC0CON = 0x11;       // конфигурация ЦАП 0:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП0 обновляется по заднему фронту такта ядра

  // ЦАП 1
  DAC1CON = 0x11;       // конфигурация ЦАП 1:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП1 обновляется по заднему фронту такта ядра

  // ЦАП 2
  DAC2CON = 0x11;       // конфигурация ЦАП 2:
                        // диапазон 0-DAC(REF)
                        // значение выдаваемое ЦАП2 обновляется по заднему фронту такта ядра
  DACConfiguration();

#ifdef DEBUG
  //printf("PLLCON=%x\n", PLLCON);
  printf("DBG: Hangerup configure\n");
#endif

  //**********************************************************************
  // Конфигурация подвеса ?????
  //**********************************************************************
  configure_hanger();

  // *********************************************************************
  // Конфигурация фазового сдвига
  // *********************************************************************
  SendPhaseShift();

  // *********************************************************************
  // Пауза 1 сек
  // *********************************************************************
  pause( 32768*1);


  //**********************************************************************
  // Ожидание раскачки виброподвеса
  //**********************************************************************

#ifdef DEBUG
  printf("DBG: Hangerup vibration control\n");
#endif

#ifdef SKIP_START_CHECKS
  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("SKIPPED\n");
  #endif
#else

  dStartAmplAngCheck = ( double) flashParamAmplAngMin1 / 65535. * 6.;
  prt2val = T2VAL;
  ADCCP = 0x08;   //AmplAng channel = ADC8
  pause(10);
  while( 1) {
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    if( ( ( double) gl_ssh_ampl_angle / 4095. * 3. / 0.5) > dStartAmplAngCheck) {

      //SUCCESS

      #ifdef DEBUG
        printf("successfully passed\n");
      #endif

      break;
    }

    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 5.0) {
      #ifdef DEBUG
        printf("FAILED\n");
      #endif
     deadloop_no_hangerup();

    }
  }
#endif



  /*
  while(1) {
    printf("\n\n\n");

    printf("4.0 -> 0\n");
    printf("DAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);
    GP4DAT &= ~( 1 << (16 + 0));
    GP4CLR = ( 1 << ( 16 + 0));
    printf("\nDAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);

    pause( 65535);

    printf("4.1 -> 0\n");
    printf("DAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);
    GP4DAT &= ~( 1 << (16 + 1));
    GP4CLR = ( 1 << ( 16 + 1));
    printf("\nDAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);

    pause( 65535);

    printf("4.0 -> 1\n");
    printf("DAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);
    GP4DAT |= ( 1 << (16 + 0));
    GP4SET = ( 1 << (16 + 0));
    printf("\nDAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);

    pause( 65535);

    printf("4.1 -> 1\n");
    printf("DAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);
    GP4DAT |= ( 1 << (16 + 1));
    GP4SET = ( 1 << (16 + 1));
    printf("\nDAT:%x\n", GP4DAT);
    printf("SET:%x\n", GP4SET);
    printf("CLR:%x\n", GP4CLR);

    pause( 65535);
  }*/




  //**********************************************************************
  // ПОДЖИГ ЛАЗЕРА
  //**********************************************************************
#ifdef DEBUG
  printf("DBG: Laser fireup...\n");
#endif

#ifdef SKIP_START_CHECKS
  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("DBG: Laser fireup...SKIPPED\n");
  #endif
#else

  nFiringTry = 0;

  #ifdef DEBUG
    printf("DBG: Turning on HV\n");
  #endif




  //поджигаем лазер
  GP4DAT &= ~( 1 << (16 + 0));      //ONHV   (p4.0) = 0
  GP4CLR = ( 1 << ( 16 + 0));      //дублёр

  //пауза 0.5 сек (32768 * 0.5 = 16384)
  pause( 16384);

  while( 1) {

    //измеряем ток I1
    ADCCP = 0x02;       //ADC2 --> I1
    //pause(10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП
    gl_ssh_current_1 = (ADCDAT >> 16);

    //измеряем ток I2
    ADCCP = 0x01;       //ADC1 --> I2
    //pause(10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП
    gl_ssh_current_2 = (ADCDAT >> 16);


    /*if( ( ( double) gl_ssh_current_1 / 4095. * 3. / 3.973 < ( double) flashParamI1min / 65535. * 0.75)  ||
        ( ( double) gl_ssh_current_2 / 4095. * 3. / 3.973 < ( double) flashParamI2min / 65535. * 0.75)) {*/

    #ifdef DEBUG
      printf("DBG: try %d\n", nFiringTry);
      printf("DBG: I1: Measured: %.02f  Goal: %.02f\n", ( 2.5 - ( double) gl_ssh_current_1 / 4095. * 3.) / 2.5, ( double) flashParamI1min / 65535. * 0.75);
      printf("DBG: I2: Measured: %.02f  Goal: %.02f\n", ( 2.5 - ( double) gl_ssh_current_2 / 4095. * 3.) / 2.5, ( double) flashParamI2min / 65535. * 0.75);
    #endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4095. * 3.) / 2.5 < ( double) flashParamI1min / 65535. * 0.75)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4095. * 3.) / 2.5 < ( double) flashParamI2min / 65535. * 0.75)) {
      //не зажглось

      if( nFiringTry < 25) {

        if( nFiringTry > 0 && ( nFiringTry % 5) == 0) { //if( nFiringTry % 5) == 0)
#ifdef DEBUG
  printf( "DBG: Relax pause 1 sec\n");
#endif
          //выключаем лазер
          GP4DAT |= 1 << (16 + 0);        //ONHV       (p4.0) = 1
          GP4SET = 1 << (16 + 0);         //дублёр

          //пауза 1 сек
          pause( 32786);

          //поджигаем лазер
          GP4DAT &= ~( 1 << (16 + 0));    //ONHV   (p4.0) = 0
          GP4CLR = ( 1 << ( 16 + 0));     //дублёр
        }

#ifdef DEBUG
  printf( "DBG: Applying 3kV for 1 sec\n");
#endif

        //включаем усиленный поджиг
        GP4DAT &= ~( 1 << (16 + 1));      //OFF3KV (p4.1) = 0
        GP4CLR = ( 1 << ( 16 + 1));       //дублёр

        //пауза 1 сек
        pause( 32786);

        //выключаем усиленный поджиг
        GP4DAT |= 1 << (16 + 1);          //OFF3KV     (p4.1) = 1
        GP4SET = 1 << (16 + 1);           //дублёр

        //увеличиваем число попыток
        nFiringTry++;

        //пауза 0.5 сек
        pause( 16384);
      }
      else {

#ifdef DEBUG
  printf( "DBG: fireup FAILS\n");
#endif

        //выключаем горение
        GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
        GP4SET = 1 << (16 + 0);      //дублёр

        deadloop_no_firing();         //FAIL.FINISH
      }
    }
    else {
      //SUCCESS! зажглось

      #ifdef DEBUG
        printf("DBG: Laser fireup... successfully passed\n");
      #endif

      break;
    }
  }
#endif


  //**********************************************************************
  // ПРОВЕРКА ТАКТОВОГО СИГНАЛА
  //**********************************************************************

#ifdef DEBUG
  printf("DBG: Tacting signal check...\n");
#endif

#ifdef SKIP_START_CHECKS
  gl_b_SyncMode = 1;

  #ifdef DEBUG
    pause( 16384*1);      //0.5 sec pause
    printf("SKIPPED\n");
    printf("DBG: Working in asynchronous mode\n");
  #endif

#else

  /*
  //сброс интеграторов в системе регулировки периметра
  GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
  pause( 327);                  //pause 10msec
  GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
  */

  //**********************************************************************
  //ТАКТИРОВАНИЕ
  //**********************************************************************

  GP3DAT |= ( 1 << (16 + 5));	//OutLnfType (p3.5) = 1  изначально предполагаем асинхронное тактирование

  //**********************************************************************
  // Ожидание первого такта SA. Если его не будет в течении 0.5 сек - включаемся в синхронный режим
  //**********************************************************************
  prt2val = T2VAL;
  while(1) {
    if( GP0DAT & 0x10) {
      gl_b_SyncMode = 1;
      #ifdef DEBUG
        printf("DBG: Got 1 on P0.4 so working in asynchronous mode\n");
      #endif
      break;
    }

    if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {
      GP3DAT &= ~(1 << (16 + 5));   //OutLnfType (p3.5) = 0
      break;
    }
  }

  if( !gl_b_SyncMode) {
    #ifdef DEBUG
      printf("DBG: Passed 0.5 sec with P0.4 in zero-state. So working in synchronous mode\n");
      printf("DBG: Waiting for SA signal\n");
    #endif

    //работаем в синхронном режиме - проверка наличия SA
    prt2val = T2VAL;
    while( 1) {
      if( GP0DAT & 0x10) {
        //SA пришел - все ОК
        #ifdef DEBUG
          printf("DBG: Got SA signal! SA TEST PASSED\n");
        #endif
        break;
      }

      if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {
        //SA не пришел в течении 0.5 сек - отвал в deadloop

        #ifdef DEBUG
          printf("FAILED\n");
        #endif

        deadloop_no_tact( ERROR_NO_TACT_SIGNAL);
        break;
      }
    }
  }

  /*
  //принудительный синхронный режим
  gl_b_SyncMode = 0;
  GP3DAT &= ~(1 << (16 + 5)); //OutLnfType (p3.5) = 0
  */

  #ifdef DEBUG
    printf("passed\n");
    printf("DBG: Internal ADC start...");
  #endif

#endif

  //**********************************************************************
  // Запуск преобразования АЦП
  //**********************************************************************
  ADCCP = 0x01;
  //pause(10);
  ADCCON |= 0x80;

#ifdef DEBUG
  printf("passed\n");
  printf("DBG: Skipping first SA Tact...");
#endif

#ifndef SKIP_START_CHECKS
  //**********************************************************************  
  //пропуск (такта ? N тактов) сигнала SA
  //**********************************************************************
  /*
  for( i=0; i<500; i++) {
    while( (GP0DAT & 0x10));
    while( !(GP0DAT & 0x10));
    while( (GP0DAT & 0x10));
  }
  */

  while( (GP0DAT & 0x10));
  while( !(GP0DAT & 0x10));
  while( (GP0DAT & 0x10));

#endif

  //включение интегратора системы регулировки периметра
  GP0DAT &= ~( 1 << (16 + 5));  //RP_P       (p0.5) = 0
  GP0CLR = ( 1 << (16 + 5));   //дублёр

//поначалу блокируем выдачу данных программой (время готовности)
#ifdef DEBUG
  //НО НЕ В DEBUG'e!!!!!
  gl_bOutData = 1;
#else
  gl_bOutData = 0;
#endif

  //**********************************************************************
  // Основной цикл работы программы
  //**********************************************************************
#ifdef DEBUG
  printf("done\n");

/*
  if( gl_b_SyncMode)
    printf("DBG: Calculation of first decrement coefficient\n");  
#endif
  
  if( gl_b_SyncMode)
    FirstDecrementCoeffCalculation();

#ifdef DEBUG
  printf("VALUE=%.2f  ", flashParamDecCoeff / 65535.);
#endif

#ifdef DEBUG
  if( gl_b_SyncMode)
    printf("passed\n");
*/

  printf("DBG: Configuration passed. Main loop starts!\n");
#endif

  //FAKE ('VERACITY DATA' flag on)
  gl_b_PerimeterReset = 1;

  //Software version
  send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + ( VERSION_MAJOR * 16 + VERSION_MIDDLE));

  //Device.serial.Num
  send_pack( 0, 26,   flashParamDeviceId & 0xFF);         //Device_Num.Byte1
  send_pack( 0, 27, ( flashParamDeviceId & 0xFF00) >> 8); //Device_Num.Byte2

  //FAKE ('VERACITY DATA' flag off)
  gl_b_PerimeterReset = 0;

  nT2RepeatBang = T2VAL;

  while(1) {

    //************************************************************************
    //РЕЖИМ SIMPLE dN_dU_Counter
    if( bSimpleDnDu == 1) {

      while( !(GP0DAT & 0x10)) {}

      if( gl_b_SA_Processed == 0) {

        //Руководство к действиям:
        //сразу же по пришествию сигнала SA:
        //1. в случае асинхронного режима запускаем преобразование угла поворота
        //2. в любом режиме работы без пауз делаем импульс WrCnt


        //**********************************************************************
        // Запуск АЦП сигнала угла отклонения подвеса (асинхр. режим)
        //**********************************************************************
        if( gl_b_SyncMode) {
          //асинхронный режим


          //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
          //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
          //**********************************************************************
          // запуск преобразования угла поворота подвеса cnvst->0
          //**********************************************************************

          //GP3CLR |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 0
          GP3DAT ^= 1 << (16 + 7);

          //**********************************************************************
          //Импульс WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause_T0( 20);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);  //WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);  //WrCnt clear
          */


          //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
          //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
          //**********************************************************************
          // тут же cnvst ->1 (чтобы внешний АЦП не "выпадал" в shutdown по окончанию преобразования)
          //**********************************************************************
          //GP3SET |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 1
          GP3DAT ^= 1 << (16 + 7);

        }
        else {
          //синхронный режим

          //**********************************************************************
          //Импульс WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause_T0( 20);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);	//WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);	//WrCnt clear
          */

        }

        //**********************************************************************
        // Получение приращения угла
        //**********************************************************************

        //получение старшего байта приращения угла
        GP1SET |= 1 << (16 + 3);    //rdhbc set
        //pause_T0( 1);

        hb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP2DAT & BIT_1) >> 1) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP1CLR |= 1 << (16 + 3);    //rdhbc clear

        //получение младшего байта приращения угла
        GP1SET |= 1 << (16 + 4);    //rdlbc set
        //pause_T0( 20);

        lb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP2DAT & BIT_1) >> 1) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP1CLR |= 1 << (16 + 4);    //rdlbc clear

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
        gl_ssh_angle_inc = lb + (hb << 8);


        //**********************************************************************
        // Ожидание результата измерение угла отклонения подвеса (асинхр. режим)
        //**********************************************************************
        if( gl_b_SyncMode) {
          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

          while( ( GP2DAT & 0x80)) {}                                 //ждем пока busy не упадет в 0

          //читаем
          GP1CLR |= 1 << (16 + 7);                                    //cs->0

          //pause_T0( 20);
          //for( k=0; k<10; k++);                                      //выждать t4
          //pause( 1);


          GP2SET |= 1 << (16 + 2);                                    //sclk->1
          val = 0;

          //читаем 14 bit
          for( i=0; i<14; i++) {
            //GP2CLR |= 1 << (16 + 2);                                  //sclk->0
            GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0

            //pause_T0( 20);
            //for( k=0; k<5; k++);                                    //выждать t5
            //pause( 1);

            val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //читаем бит

            //GP2SET |= 1 << (16 + 2);                                  //sclk->1
            GP2DAT |= 1 << (16 + 2);                                  //sclk->1

            //pause_T0( 20);
            //for( k=0; k<5; k++);                                    //выждать t8
            //pause( 1);
          }

          GP1SET |= 1 << (16 + 7);                                    //cs->1

          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
          gl_ssh_angle_hanger = val << 2;
          gl_ssh_angle_hanger = gl_ssh_angle_hanger / 4;
          //printf("\n** %d (codes) = %f V\n\n", si, ( double) si / 8192. * 5.);
        }

        /*
        //dN
        sh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
        //dU
        sh_dU = gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev;
        //dT
        gl_ssh_SA_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;
        gl_ssh_prT1VAL = T1VAL;
        */

        //счётчик
        cSimpleDnDuCounter++;

        ssh_dN = ( gl_ssh_angle_inc - gl_ssh_angle_inc_prev);
        ssh_dU = ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev);

        cCheckSumm = 0;
        //вывод.маркер
        putchar_nocheck( 0xCC);
        //вывод.dN
        putchar_nocheck( ssh_dN & 0xFF);            cCheckSumm += ssh_dN & 0xFF;
        putchar_nocheck( ( ssh_dN & 0xFF00) >> 8);  cCheckSumm += ( ssh_dN & 0xFF00) >> 8;
        //вывод.dU
        putchar_nocheck( ssh_dU & 0xFF);            cCheckSumm += ssh_dU & 0xFF;
        putchar_nocheck( ( ssh_dU & 0xFF00) >> 8);  cCheckSumm += ( ssh_dU & 0xFF00) >> 8;
        //вывод.счётчик
        putchar_nocheck( cSimpleDnDuCounter);       cCheckSumm += cSimpleDnDuCounter;
        //вывод.CS
        putchar_nocheck( cCheckSumm);

        //поднимаем флаг о том, что текущий высокий уровень SA (такт) мы обработали
        gl_b_SA_Processed = 1;
      }
      else {
        while( ( GP0DAT & 0x10)) {}
        gl_b_SA_Processed = 0;
      }
    }



    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          /*
          cRULAControl = ( RULA_MAX - RULA_MIN) / 2;
          delta = ( RULA_MAX - RULA_MIN) / 4;
          */
          gl_un_RULAControl = 64;
          nDelta = ( RULA_MAX - RULA_MIN) / 2;

          gl_sn_MeaningCounter = 0;
          gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
          dMeaningSumm = 0.;
          dMeanImps = 0.;

        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          configure_hanger(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();

          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1

          nRppTimer = T2VAL;
          gl_b_PerimeterReset = 1;

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
/*#ifdef DEBUG
          printf("DBG: Input command (0x04 - SetControlI1) accepted. Param: 0x%04x\n", flashParamI1min);
#endif*/
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 8: //установить знаковый коэффициент dU
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
          bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
          bAsyncDu = 0;
        break;

        case 11: //калибровка термодатчиков (параметр - реальная температура)
          bCalibrated = 0;
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
              flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //у нас есть нормальная минимальная точка калибровки

              //затычка на то, чтобы не давали мин точку равной макс
              if( in_param_temp == flashParam_calibT1) {
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
                break;
              }

              if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
                  flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
                //у нас есть нормальные минимальная и максимальная точка калибровки
                //определим какую надо заменить
                if( in_param_temp < flashParam_calibT1) {
                  //надо заменить минимальную
                  bCalibProcessState = 1;
                  flashParam_calibT1 = in_param_temp;
                }
                else {
                  //надо заменить максимальную
                  bCalibProcessState = 4;
                  flashParam_calibT2 = in_param_temp;
                }
              }
              else {
                //у нас есть нормальная минимальная точка калибровки, но нет нормальной максимальной
                bCalibProcessState = 3;
                flashParam_calibT2 = in_param_temp;
              }

          }
          else {
            //у нас нет даже минимальной точки!! значит это будет минимальная :)
            flashParam_calibT1 = in_param_temp;
            bCalibProcessState = 1;
          }
        break;

        case 12:    //команда сброса данных калибровки термодатчиков
          bCalibrated = 0;
          flashParam_calibT1 = 0;
          flashParamT1_TD1_val = 0;
          flashParamT1_TD2_val = 0;
          flashParamT1_TD3_val = 0;

          flashParam_calibT2 = 0;
          flashParamT2_TD1_val = 1;
          flashParamT2_TD2_val = 1;
          flashParamT2_TD3_val = 0;

          SaveThermoCalibPoint();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 14:    //команда нового фазового сдвига
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          flashParamPhaseShift = in_param_temp;
          SendPhaseShift();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 15:    //команда выключения лазера
          GP4DAT |= 1 << (16 + 0);	    //ONHV       (p4.0) = 1
          GP4DAT |= 1 << (16 + 1);	    //OFF3KV     (p4.1) = 1
          gl_bManualLaserOff = 1;
        break;

        case 16:    //команда выключение интегратора
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (выключение)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_b_PerimeterReset = 1;      //устанавливаем флаг недостоверности данных (1=прошло выключение, данные НЕдостоверны)
        //nRppTimer = ;               //<-- ВРЕМЯ НЕ ЗАСЕКАЕМ! ФЛАГ НЕ ОПУСТИТСЯ НИКОГДА (только по команде включить интегратор)
        break;

        case 17:    //команда включения интегратора
          GP0DAT &= ~( 1 << (16 + 5));   //RP_P   (p0.5) = 0 (включение)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_b_PerimeterReset = 2;      //устанавливаем флаг недостоверности данных (2=прошло включение, данные НЕдостоверны)
          nRppTimer = T2VAL;            //<-- ЗАСЕКАЕМ ВРЕМЯ! И ЧЕРЕЗ [определённое время] ФЛАГ БУДЕТ СНЯТ
        break;

        case 18:    //команда сброса интегратора
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (выключение)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          nRppTimer = T2VAL;            //<-- ЗАСЕКАЕМ ВРЕМЯ! И ЧЕРЕЗ [определённое время] ИНТЕГРАТОР ВКЛЮЧИТСЯ, ФЛАГ перейдёт в состояние 2, а ещё через [определённое время] флаг перейдёт в состояние 0 (данные достоверны)
          gl_b_PerimeterReset = 1;      //устанавливаем флаг недостоверности данных (1=прошло выключение, данные НЕдостоверны)
        break;

        case 49: //запрос длинной пачки параметров                          49 = 0x31 = "1"
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 50: //сохранить параметры во флэш память                       50 = 0x32 = "2"
          save_params();
        break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их     51 = 0x33 = "3"
          load_params(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        //DEVICE_id
        case 'S': //запрос DeviceID                                         83 = 0x53 = "S"
          #ifdef DEBUG
            printf("DBG: 'S' cmd in\n");
          #endif
          nSentPacksCounter = 26; nSentPacksRound = DEVICE_NUM_PACK_LEN;
        break;

        case 'T': //установить DeviceID                                     84 = 0x54 = "T"
          flashParamDeviceId = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
        break;

        //DATE
        case 'U': //запрос Date                                             85 = 0x55 = "U"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'V': //установка Date.YEAR                                     86 = 0x56 = "V"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'W': //установка Date.MONTH                                    87 = 0x57 = "W"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'X': //установка Date.DAY                                      88 = 0x58 = "X"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        //HV_apply_duration
        case 'Y': //запрос Hv.apply.duration                                89 = 0x59 = "Y"
          nSentPacksCounter = 47; nSentPacksRound = HV_APPLY_TIME_LEN;
        break;

        //ORGANIZATION
        case 'a': //запрос Organization                                     97 = 0x61 = "a"
          nSentPacksCounter = 28; nSentPacksRound = ORG_NAME_PACK_LEN;
        break;

        case 'b': //установить Organization.byte1                           98 = 0x62 = "b"
          flashParamOrg[0] = input_buffer[1];        break;
        case 'c': //установить Organization.byte2                           99 = 0x63 = "c"
          flashParamOrg[1] = input_buffer[1];        break;
        case 'd': //установить Organization.byte3                           100= 0x64 = "d"
          flashParamOrg[2] = input_buffer[1];        break;
        case 'e': //установить Organization.byte4                           101= 0x65 = "e"
          flashParamOrg[3] = input_buffer[1];        break;
        case 'f': //установить Organization.byte5                           102= 0x66 = "f"
          flashParamOrg[4] = input_buffer[1];        break;
        case 'g': //установить Organization.byte6                           103= 0x67 = "g"
          flashParamOrg[5] = input_buffer[1];        break;
        case 'h': //установить Organization.byte7                           104= 0x68 = "h"
          flashParamOrg[6] = input_buffer[1];        break;
        case 'i': //установить Organization.byte8                           105= 0x69 = "i"
          flashParamOrg[7] = input_buffer[1];        break;
        case 'j': //установить Organization.byte9                           106= 0x6A = "j"
          flashParamOrg[8] = input_buffer[1];        break;
        case 'k': //установить Organization.byte10                          107= 0x6B = "k"
          flashParamOrg[9] = input_buffer[1];        break;
        case 'l': //установить Organization.byte11                          108= 0x6C = "l"
          flashParamOrg[10] = input_buffer[1];        break;
        case 'm': //установить Organization.byte12                          109= 0x6D = "m"
          flashParamOrg[11] = input_buffer[1];        break;
        case 'n': //установить Organization.byte13                          110= 0x6E = "n"
          flashParamOrg[12] = input_buffer[1];        break;
        case 'o': //установить Organization.byte14                          111= 0x6F = "o"
          flashParamOrg[13] = input_buffer[1];        break;
        case 'p': //установить Organization.byte15                          112= 0x70 = "p"
          flashParamOrg[14] = input_buffer[1];        break;
        case 'q': //установить Organization.byte16                          113= 0x71 = "q"
          flashParamOrg[15] = input_buffer[1];        break;

        case 'r': //FAKE! установить код ошибки (типа ошибка прибора)       114= 0x72 = "r"
          gl_c_EmergencyCode = input_buffer[1];
        break;

        case 's': //включение-выключение режима dN_dU_counter               115= 0x73 = "s"
          bSimpleDnDu ^= 1;
        break;

      }


      pos_in_in_buf = 0;
    }


    if( bSimpleDnDu == 1) {
      continue;
    }

    /*
    //наёбочная часть - эмуляция такта 10(5?) раз в секунду
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;
    */

    //if( bSAFake) {	// на ноге P0.4 есть FAKE сигнал
    if( GP0DAT & 0x10) {  //на ноге P0.4 есть сигнал

      if( gl_b_SA_Processed == 0) { //если в этом SA цикле мы его еще не обрабатывали

        gl_ssh_SA_time = ( unsigned short) ( ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD);
        gl_n_prT1VAL = T1VAL;

        //В тестовых целях делаем сигнал на линии P0.0
        GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
        /*for( i=0; i<100; i++);
        GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear*/

        //Руководство к действиям:
        //сразу же по пришествию сигнала SA:
        //1. в случае асинхронного режима запускаем преобразование угла поворота
        //2. в любом режиме работы без пауз делаем импульс WrCnt


        //**********************************************************************
        // Запуск АЦП сигнала угла отклонения подвеса (асинхр. режим)
        //**********************************************************************
        if( gl_b_SyncMode) {
          //асинхронный режим


          //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
          //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
          //**********************************************************************
          // запуск преобразования угла поворота подвеса cnvst->0
          //**********************************************************************

          //GP3CLR |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 0
          GP3DAT ^= 1 << (16 + 7);

          //**********************************************************************
          //Импульс WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause( 1);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);  //WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);  //WrCnt clear
          */


          //2014-09-17 Выяснилось что функция CnvSt отдана Альтере и я дёргал ногой впустую... комментим
          //2015-02-11 оказывается что альтера плохо дергает ногой CNVST. Управление возвращается мне
          //**********************************************************************
          // тут же cnvst ->1 (чтобы внешний АЦП не "выпадал" в shutdown по окончанию преобразования)
          //**********************************************************************
          //GP3SET |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 1
          GP3DAT ^= 1 << (16 + 7);

        }
        else {
          //синхронный режим

          //**********************************************************************
          //Импульс WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause( 1);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);	//WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);	//WrCnt clear
          */

        }

        //**********************************************************************
        // Получение приращения угла
        //**********************************************************************

        //получение старшего байта приращения угла
        GP1SET |= 1 << (16 + 3);    //rdhbc set
        //pause( 1);

        hb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP2DAT & BIT_1) >> 1) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP1CLR |= 1 << (16 + 3);    //rdhbc clear

        //получение младшего байта приращения угла
        GP1SET |= 1 << (16 + 4);    //rdlbc set
        //pause( 1);

        lb = (( GP1DAT & BIT_5) >> 5) +
             ((( GP2DAT & BIT_1) >> 1) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);

        GP1CLR |= 1 << (16 + 4);    //rdlbc clear

        gl_ssh_angle_inc = lb + (hb << 8);


        //**********************************************************************
        // Ожидание результата измерение угла отклонения подвеса (асинхр. режим)
        //**********************************************************************
        if( gl_b_SyncMode) {
          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

          while( ( GP2DAT & 0x80)) {}                                 //ждем пока busy не упадет в 0

          //читаем
          GP1CLR |= 1 << (16 + 7);                                    //cs->0

          //for( k=0; k<10; k++);                                      //выждать t4
          //pause( 1);

          GP2SET |= 1 << (16 + 2);                                    //sclk->1
          val = 0;

          //читаем 14 bit
          for( i=0; i<14; i++) {
            //GP2CLR |= 1 << (16 + 2);                                  //sclk->0
            GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0

            //for( k=0; k<5; k++);                                    //выждать t5
            //pause( 1);

            val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //читаем бит

            //GP2SET |= 1 << (16 + 2);                                  //sclk->1
            GP2DAT |= 1 << (16 + 2);                                  //sclk->1

            for( k=0; k<5; k++);                                    //выждать t8
            //pause( 1);
          }

          GP1SET |= 1 << (16 + 7);                                    //cs->1
          gl_ssh_angle_hanger = val << 2;
          gl_ssh_angle_hanger = gl_ssh_angle_hanger / 4;
          //printf("\n** %d (codes) = %f V\n\n", si, ( double) si / 8192. * 5.);
        }


        //**********************************************************************
        // Получение аналогового параметра
        //**********************************************************************
        while (!( ADCSTA & 0x01)){}     // ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)
        
        switch( ADCChannel) { //анализируем что мы оцифровывали и сохраняем в соответствующую переменную

          case 0:
            gl_ssh_current_2 = (ADCDAT >> 16);
          break;                       //ADC1 = I2

          case 1:
            gl_ssh_current_1 = (ADCDAT >> 16);
          break;                       //ADC2 = I1

          case 2:                                                                 //ADC3 = UINT (U_td3)
          break;

          case 3:                                                                 //ADC4 = CntrPc
            gl_ssh_Perim_Voltage = (ADCDAT >> 16);
            //gl_ssh_Perim_Voltage = ( short) ( 4095 - ( int) gl_ssh_Perim_Voltage);
          break;

          case 4:                                                                 //ADC5 = UTD1
            gl_ssh_Utd1 = (ADCDAT >> 16);
            gl_ssh_Utd1_cal = gl_ssh_Utd1;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd1 * TD1_K + TD1_B;
            else
              temp_t = ( (( double) gl_ssh_Utd1 / 4095. * 3000. - 744. ) / 11.9);
            gl_ssh_Utd1 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD1

          case 5:                                                                 //ADC6 = UTD2
            gl_ssh_Utd2 = (ADCDAT >> 16);
            gl_ssh_Utd2_cal = gl_ssh_Utd2;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd2 * TD1_K + TD1_B;
            else
              temp_t = ( (( double) gl_ssh_Utd2 / 4095. * 3000. - 744. ) / 11.9);
            gl_ssh_Utd2 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD2

          case 6:                                                                 //ADC7 = UTD3
            gl_ssh_Utd3 = (ADCDAT >> 16);
            gl_ssh_Utd3_cal = gl_ssh_Utd3;
            if( bCalibrated)
              temp_t = ( double) gl_ssh_Utd3 * TD3_K + TD3_B;
            else
              temp_t = ( (( double) gl_ssh_Utd3 / 4095. * 3000. - 744. ) / 11.9);
            gl_ssh_Utd3 = ( short) ( ( temp_t + 100.) / 200. * 65535.);
          break;  //UTD3

          case 7: gl_ssh_ampl_angle = (ADCDAT >> 16); break;                      //ADC8 = AmplAng
        }

        if( bCalibProcessState) {
          switch( bCalibProcessState) {
            case 1:
              //калибруем первый датчик на минимальной температуре
              if( ADCChannel == 0) {
                flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 2;
              }
            break;

            case 2:
              //калибруем второй датчик на минимальной температуре
              if( ADCChannel == 1) {
                flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 3;
              }
            break;

            case 3:
              //калибруем третий датчик на минимальной температуре
              if( ADCChannel == 2) {
                flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;

            case 4:
              //калибруем первый датчик на максимальной температуре
              if( ADCChannel == 0) {
                flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 5;
              }
            break;

            case 5:
              //калибруем второй датчик на максимальной температуре
              if( ADCChannel == 1) {
                flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 6;
              }
            break;

            case 6:
              //калибруем третий датчик на максимальной температуре
              if( ADCChannel == 2) {
                flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;
          }

          if( !bCalibProcessState)
            //если это закончилась калбировка какой либо точки - перерасчитаем калибровочные параметры
            ThermoCalibrationCalculation();
        }

        ADCChannel = ( ++ADCChannel) % 8;
        /*switch( ADCChannel) {
          case 0: ADCChannel = 1; break;  //just measured I2, next is I1
          case 1: ADCChannel = 2; break;  //just measured I1, next is TD3
          case 2: ADCChannel = 3; break;  //just measured TD3, next is CntrPC
          case 3: ADCChannel = 4; break;  //just measured CntrPc, next is TD1
          case 4: ADCChannel = 5; break;  //just measured TD1, next is TD2
          case 5: ADCChannel = 6; break;  //just measured TD2, next is AmplAng
          case 6: ADCChannel = 0; break;  //just measured AmplAng, returning to I2
        }*/


        ADCCP = 0x01 + ADCChannel;              //выставляем новый канал АЦП
        //pause(10);
        ADCCON |= 0x80;                         //запуск нового преобразования (съем будет в следующем такте SA)


        //**********************************************************************
        // Получение числа импульсов от альтеры
        //**********************************************************************
        GP3DAT |= 1 << (16 + 3);      //set RdN7N0 -> 1
        //pause( 1);
        gl_ush_MeanImpulses = (( GP1DAT & BIT_5) >> 5) +
             ((( GP2DAT & BIT_1) >> 1) << 1) +
             ((( GP0DAT & BIT_1) >> 1) << 2) +
             ((( GP2DAT & BIT_3) >> 3) << 3) +
             ((( GP4DAT & BIT_6) >> 6) << 4) +
             ((( GP4DAT & BIT_7) >> 7) << 5) +
             ((( GP0DAT & BIT_6) >> 6) << 6) +
             ((( GP0DAT & BIT_2) >> 2) << 7);
        GP3DAT &= ~( 1 << (16 + 3));  //set RdN7N0 -> 0


        GP2DAT |= 1 << (16 + 4);      //set RdN10N8 -> 1
        //pause( 1);
        gl_ush_MeanImpulses += 
             ((( GP1DAT & BIT_5) >> 5) << 8) +
             ((( GP2DAT & BIT_1) >> 1) << 9) +
             ((( GP0DAT & BIT_1) >> 1) << 10);
        GP2DAT &= ~( 1 << (16 + 4));  //set RdN10N8 -> 0


        //**********************************************************************
        // Выдача данных согласно протоколу
        //**********************************************************************
        if( gl_bOutData == 1) {
          switch( nSentPacksCounter) {
            case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_ssh_Utd1);      break;      //UTD1
            //case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_ssh_ampl_angle);      break; //AmplAng с ДУСа

            case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_ssh_Utd2);      break;      //UTD2
            //case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_un_RULAControl); break;     //RULA
            //case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, dMeanImps); break;             //dMeanImps (сигнал который мы пытаемся приблизить к заданной амплитуде)

            case 2: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 2, gl_ssh_Utd3);      break;      //UTD3

            case 3: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 3, gl_ssh_current_1);      break; //I1
            case 4: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 4, gl_ssh_current_2);      break; //I2
            case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_Perim_Voltage);  break; //CntrPc

            case 6: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 6, gl_ush_MeanImpulses); break; //AmplAng от альтеры
            //case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_ampl_angle); break; //AmplAng с ДУСа

            case 7: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 7, flashParamAmplitudeCode);    break; //код амплитуды
            case 8: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 8, flashParamTactCode);    break;      //код такта подставки
            case 9: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 9, flashParamMCoeff);    break;        //Коэффициент М
            case 10: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 10, flashParamStartMode);    break;     //Начальная мода
            case 11: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 11, flashParamI1min);    break;       //flashParamI1min
            case 12: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 12, flashParamI2min);    break;       //flashParamI2min
            case 13: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 13, flashParamAmplAngMin1);    break; //flashParamAmplAngMin1
            case 14: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 14, flashParamDecCoeff);    break;    //flashParamAmplAngMin2
            case 15: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 15, flashParamSignCoeff);    break;      //flashParamSignCoeff
            case 16: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));    break;               //SOFTWARE VERSION
            case 17: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 17, flashParam_calibT1);    break;	    //min thermo-calib point T
            case 18: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 18, flashParamT1_TD1_val);    break;	  //min thermo-calib point thermo1 data
            case 19: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 19, flashParamT1_TD2_val);    break;	  //min thermo-calib point thermo2 data
            case 20: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 20, flashParamT1_TD3_val);    break;	  //min thermo-calib point thermo3 data
            case 21: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 21, flashParam_calibT2);    break;	    //max thermo-calib point T
            case 22: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 22, flashParamT2_TD1_val);    break;	  //max thermo-calib point thermo1 data
            case 23: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 23, flashParamT2_TD2_val);    break;	  //max thermo-calib point thermo2 data
            case 24: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 24, flashParamT2_TD3_val);    break;	  //max thermo-calib point thermo3 data
            case 25: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 25, flashParamPhaseShift);    break;	  //phase shift

            case 26: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 26,   flashParamDeviceId & 0xFF);            break;    //Device_Num.Byte1
            case 27: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 27, ( flashParamDeviceId & 0xFF00) >> 8);    break;    //Device_Num.Byte2

            case 28: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 28, flashParamOrg[ 0]);        break;    //Organization.Byte1
            case 29: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 29, flashParamOrg[ 1]);        break;    //Organization.Byte2
            case 30: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 30, flashParamOrg[ 2]);        break;    //Organization.Byte3
            case 31: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 31, flashParamOrg[ 3]);        break;    //Organization.Byte4
            case 32: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 32, flashParamOrg[ 4]);        break;    //Organization.Byte5
            case 33: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 33, flashParamOrg[ 5]);        break;    //Organization.Byte6
            case 34: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 34, flashParamOrg[ 6]);        break;    //Organization.Byte7
            case 35: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 35, flashParamOrg[ 7]);        break;    //Organization.Byte8
            case 36: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 36, flashParamOrg[ 8]);        break;    //Organization.Byte9
            case 37: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 37, flashParamOrg[ 9]);        break;    //Organization.Byte10
            case 38: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 38, flashParamOrg[10]);        break;    //Organization.Byte11
            case 39: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 39, flashParamOrg[11]);        break;    //Organization.Byte12
            case 40: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 40, flashParamOrg[12]);        break;    //Organization.Byte13
            case 41: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 41, flashParamOrg[13]);        break;    //Organization.Byte14
            case 42: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 42, flashParamOrg[14]);        break;    //Organization.Byte15
            case 43: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 43, flashParamOrg[15]);        break;    //Organization.Byte16    БЕЗ завершающего 0 на конце!!!!!

            case 44: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 44, flashParamDateYear);     break;    //Date.Year
            case 45: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 45, flashParamDateMonth);    break;    //Date.Month
            case 46: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 46, flashParamDateDay);      break;    //Date.Day

            case 47: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 47, /*(( n3kvApplyStart + T2LD - n3kvApplyEnd) % T2LD)*/ 0 );    break;    //HV apply duration
          }
        }

        //РАБОЧЕЕ ПЕРЕВЫЧИСЛЕНИЕ КОЭФФИЦИЕНТА ВЫЧЕТА
        if( gl_b_SyncMode) {
          db_dN1 = ( double) gl_ssh_angle_inc_prev;
          db_dN2 = ( double) gl_ssh_angle_inc;
          dbU1 = ( double) gl_ssh_angle_hanger_prev;
          dbU2 = ( double) gl_ssh_angle_hanger;
          Coeff = (( float) flashParamDecCoeff) / 65535.;
          gl_dbl_Omega =  ( db_dN2 - db_dN1) - ( dbU2 - dbU1) * Coeff * ( ( signed short) flashParamSignCoeff - 1);
          if( fabs( gl_dbl_Omega) < 5) {
            gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
            gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);
            gl_un_DecCoeffStatPoints++;
            if( !( gl_un_DecCoeffStatPoints % DEC_COEFF_CONTINUOUS_CALCULATION_N)) {
              flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);
              gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
              gl_un_DecCoeffStatPoints = 0;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            }
          }
        }

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;

        // ************************************************************************************
        // 2016-12-14
        // Слежение за разрядными токами
        // ************************************************************************************
        if( gl_bManualLaserOff == 0 && nSentPacksCounter == 2) {
          //если мы не выключили слежение

          //отслеживаем 5 сек с последнего момента просадки токов
          if( gl_nLaserCurrentUnstableT2 != 0) {
            if( ( gl_nLaserCurrentUnstableT2 + T2LD - T2VAL) >= 163840) { //5sec (*32kHz = 163840)
              //5сек - просадок не было. сбрасываем флаг.
              gl_nLaserCurrentUnstableT2 = 0;
              gl_nLaserCurrentUnstableCnt = 0;
            }
          }

          //чтобы токи лежали в диапазоне 0.4mA +-10% = [0.36;0.44]
          //отсчёты АЦП должны быть:
          //1912 --> 0.439844 mA
          //2184 --> 0.360156 mA

          //чтобы токи лежали в диапазоне 0.5mA +-10% = [0.45;0.55]
          //отсчёты АЦП должны быть:
          //1537 --> 0.549707 mA
          //1877 --> 0.450098 mA

          if( gl_ssh_current_1 >= 1537 && gl_ssh_current_1 <= 1877 &&
              gl_ssh_current_2 >= 1537 && gl_ssh_current_2 <= 1877) {
              //с токами всё в порядке
          }
          else {
            //отловили ситуацию когда просел ток
            gl_nLaserCurrentUnstableT2 = T2VAL; //запоминаем время (по прошествии 5 сек от него мы сбросим счётчик ошибок)
            gl_nLaserCurrentUnstableCnt++;      //увеличиваем счётчик ошибок

            if( gl_nLaserCurrentUnstableCnt >= 5) {
              //мы набрали 5 или более "выпадов" за 5 сек - выключаем ток

              GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
              GP4SET = 1 << (16 + 0);      //дублёр
              deadloop_current_unstable();
            }
          }
        }

        // ************************************************************************************
        // 2010-04-22
        // 2017-01-17 introducing moving average
        //автоматическая перестройка периметра
        // ************************************************************************************
        if( nSentPacksCounter == 5) {
          gl_shVprc[ gl_nVrpcCounter] = gl_ssh_Perim_Voltage;
          gl_nVrpcCounter = (++gl_nVrpcCounter) % VPRC_SLIDING_ROUND;

          if( gl_nVrpcCounter == 0) {
            dbl_V_piezo = ( ( CalcSlidingAverage() / 4095. * 3.) - 2.048) * 100.;

            if( fabs( dbl_V_piezo) > 90.) {
              flashParamStartMode = 125;
              DACConfiguration();
              GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
              nRppTimer = T2VAL;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              gl_b_PerimeterReset = 1;
            }
          }
        }

        //увеличиваем счетчик отправленных посылок данных
        nSentPacksCounter++;
        if     ( nSentPacksCounter == LONG_OUTPUT_PACK_LEN) { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == DEVICE_NUM_PACK_LEN)  { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == ORG_NAME_PACK_LEN)    { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == DATE_PACK_LEN)        { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == HV_APPLY_TIME_LEN)    { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else
          //ну тут остаётся только рабочий режим с 7 пачками
          nSentPacksCounter = ( nSentPacksCounter) % nSentPacksRound;

        //**********************************************************************
        //Стабилизация средней амплитуды частотной подставки (получение числа импульсов перенес выше перед отправкой данных)
        //**********************************************************************
        gl_sn_MeaningCounter = ( ++gl_sn_MeaningCounter) % gl_sn_MeaningCounterRound;

        //собственно сама подстройка напряжения RULA
        if( !gl_sn_MeaningCounter) {

          //от альтеры... сразу получаем амплитуду в виде числа импульсов
          dMeanImps = dMeaningSumm / ( double) gl_sn_MeaningCounterRound;
          dMeanImps = dMeanImps / 4.;

          //от ДУСа.... переводим в вольты из рассчёта 4095=2,5В,
          //потом рассчёт 2,2В=120"
          //и делением на масштабный коэффициент 2,9 мы получаем число импульсов
          //dMeanImps = dMeaningSumm / ( double) gl_sn_MeaningCounterRound / 4095. * 2.5 / 2.2 * 120. / 2.9;


          if( fabs( dMeanImps - ( double) flashParamAmplitudeCode) > 0.5) {
            if( dMeanImps > flashParamAmplitudeCode) {

              gl_un_RULAControl -= nDelta;

              /*
              if( nDelta >= gl_un_RULAControl) {
                //delta = cRULAControl;
                gl_un_RULAControl = gl_un_RULAControl / 2;
              }
              else
                gl_un_RULAControl -= nDelta;
              */

            }
            if( dMeanImps < flashParamAmplitudeCode) {

              gl_un_RULAControl += nDelta;

              /*
              if( gl_un_RULAControl + nDelta > RULA_MAX) {
                //delta = 255 - cRULAControl;
                gl_un_RULAControl = ( RULA_MAX + gl_un_RULAControl) / 2;
              }
              else
                gl_un_RULAControl += nDelta;
              */
            }
          }

          if( gl_un_RULAControl > RULA_MAX) gl_un_RULAControl = RULA_MAX;
          if( gl_un_RULAControl < RULA_MIN) gl_un_RULAControl = RULA_MIN;

          dMeaningSumm = 0.;

          //сокращение амплитуды "встряски" (если она еще > 1)
          nDelta = ( int) ( ( double) nDelta / 1.5);
          if( nDelta < 1) {
            nDelta = 1;
          }

          /*
          //повторная "встрясковая" подстройка после 7 секунд
          if( nT2RepeatBang) {
            if( ( T2LD + nT2RepeatBang - T2VAL) % T2LD >= 32768 * 7) {
              /*
              nDelta = ( RULA_MAX - RULA_MIN) / 4;
              gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
              */
              /*
              nT2RepeatBang = 0;
            }
          }
          */

          //включение выдачи данных после 2.5 секунд
          if( nT2RepeatBang) {
            if( ( T2LD + nT2RepeatBang - T2VAL) % T2LD >= 32768 * 1.8) {
              gl_bOutData = 1;
              nSentPacksCounter = 0;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            }
          }

          //рабочий режим (после подстройки амплитудой) тут мы подстраиваемся временем
          //2014.10.09 - добавил что если большая разница - подстроим и приращением
          if( nDelta == 1) {
            if( abs( flashParamAmplitudeCode - dMeanImps) > 10.) { nDelta = nDelta = ( RULA_MAX - RULA_MIN) / 4; gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100; }
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 5.) { nDelta = ( RULA_MAX - RULA_MIN) / 8; gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100; }
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 1.) { nDelta = ( RULA_MAX - RULA_MIN) / 16; gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100; }
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 0.9) gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 0.8) gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_200;
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 0.7) gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_300;
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 0.6) gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_400;
            else if( abs( flashParamAmplitudeCode - dMeanImps) > 0.5) gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_500;
            else gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_STABLE;
          }

          DACConfiguration();


          /*
          //"встрясковая" подстройка (включение посреди рабочего режима)
          if( gl_sn_MeaningCounterRound == MEANING_IMP_PERIOD_STABLE) {
            if( abs( flashParamAmplitudeCode - dMeanImps) > 5) {
              nDelta = ( RULA_MAX - RULA_MIN) / 4;
              gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
            }
          }*/


        }
        else {
          dMeaningSumm += gl_ush_MeanImpulses;    //от альтеры
          //dMeaningSumm += gl_ssh_ampl_angle;    //от ДУСа
        }




        //**********************************************************************
        //обработка флага сброса RP_P
        //**********************************************************************
        if( nRppTimer != 0) {
          if( (( T2LD + nRppTimer - T2VAL) % T2LD) > 32768. * 0.5) {    //длительность RESETа тут (1s = 0.5s + 0.5s)
            if( gl_b_PerimeterReset == 1) {
              //прошла первая часть (0.5 сек после выключения интегратора) ==> включаем интегратор, и засекаем ещё 0.5sec
              //включение интегратора в системе регулировки периметра
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0

              //засекаем повторно таймер
              nRppTimer = T2VAL;
              gl_b_PerimeterReset = 2;
            }
            else if( gl_b_PerimeterReset == 2) {
              //прошла вторая часть (0.5 сек после включения интегратора) ==> говорим что интегратор настроился, и данные достоверны, и таймер нам больше не нужен
              //сбрасываем флаг достоверности данных в состоние "достоверно"
              gl_b_PerimeterReset = 0;

              //больше заводить таймер не надо
              nRppTimer = 0;
              
            }
            else {
              //очень странная ситуация, но давайте сбросим всё как будто всё заработало
              //сбрасываем флаг достоверности данных в состоние "достоверно"
              gl_b_PerimeterReset = 0;

              //больше заводить таймер не надо
              nRppTimer = 0;
            }
          }
        }

        //поднимаем флаг о том что текущий высокий уровень SA мы обработали
       gl_b_SA_Processed = 1;

        //В тестовых целях делаем сигнал на линии P0.0
        /*GP0DAT |= 1 << ( 16);       //тестовая линия p0.0 set
        for( i=0; i<100; i++);*/
        GP0DAT &= ~( 1 << ( 16));   //тестовая линия p0.0 clear
      }
    }
    else {
      //если линия сигнала SA в низком уровне - то как только она поднимется начнется новый необработанный такт
      gl_b_SA_Processed = 0;

      /*
      //проверка тактирования
      ush_SA_check_time = ( T2LD + gl_ssh_prT2VAL - T2VAL) % T2LD;

      //2 sec = 32768 * 2.0 = 65536

      //как было в комменте - так и отмечаю
      //ГЛУПОСТЬ!!!!!
      //ДВУХБАЙТНЫЙ UNSIGNED SHORT  НЕ МОЖЕТ БЫТЬ НУ НИКАК больше 65535
      if( ush_SA_check_time > 65536) {
        //пропало тактирование

        //отключаем горение
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

        deadloop_no_tact( ERROR_TACT_SIGNAL_LOST);

      }
      */
    }
  }
}