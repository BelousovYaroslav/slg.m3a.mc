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

#define SHORT_OUTPUT_PACK_LEN 7       //� ����� �� ������ ���������� �� 7 ����� � ������. �����������               [0,6]
#define LONG_OUTPUT_PACK_LEN  26      //����� ����������� �� ������ 25 ����� (�� ����� ���. ����������� �� �������) [7,25]
#define DEVICE_NUM_PACK_LEN   28      //����� ����������� �������� ����� ������� �� ����� ��� ����� � ���������    [26,27]
#define ORG_NAME_PACK_LEN     44      //����� ����������� �������� ����������� �� ����� 16 ���� � ���������        [28,43]
#define DATE_PACK_LEN         47      //����� ����������� ���� �� ����� 3 ����� � ���������                        [44,45,46]
#define HV_APPLY_TIME_LEN     48      //����� ����������� ������������ ���������� HV �� ����� 1 ���� � ��������    [47]

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
//��������� �������� �� ����-������
//********************
unsigned short flashParamAmplitudeCode = 90;    //��������� ��������� ������������
unsigned short flashParamTactCode = 0;          //��� �����
unsigned short flashParamMCoeff = 4;            //����������� ���������
unsigned short flashParamStartMode = 5;         //��������� ����
unsigned int flashParamDeviceId = 0;            //ID ����������

char flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //�������� �����������

unsigned short flashParamDateYear = 0;          //���� �������.���
unsigned short flashParamDateMonth = 0;         //���� �������.�����
unsigned short flashParamDateDay = 0;           //���� �������.����

unsigned short flashParamI1min = 0;             //����������� �������� ���� ������� I1
unsigned short flashParamI2min = 0;             //����������� �������� ���� ������� I2
unsigned short flashParamAmplAngMin1 = 0;       //����������� �������� �������� � ����
unsigned short flashParamDecCoeff = 0;          //����������� ������
unsigned short flashParamSignCoeff = 2;         //�������� �����������
unsigned short flashParamPhaseShift = 0;        //[OBSOLETE] ������� �����

//���������� �������������
signed short flashParam_calibT1;
unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
signed short flashParam_calibT2;
unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;



//********************
//���������� ���������� �� ���� �������
//********************
char gl_c_EmergencyCode = 0;    //��� ������ �������

#define IN_COMMAND_BUF_LEN 3                //����� ������ �������� ������
char input_buffer[6] = { 0, 0, 0, 0, 0, 0}; //����� �������� ������
char pos_in_in_buf = 0;                     //������� ������ � ������ �������� ������


signed short gl_ssh_SA_time = 0;        //���������� ������� �����.������ SA/TA
int gl_n_prT1VAL = 0x1000;   //���������� ������� �����.�������� T1 � ������ ����������� ����� SA/TA


signed short gl_ssh_angle_inc = 0;      //���������� ����
signed short gl_ssh_angle_inc_prev = 0; //���������� ����
signed short gl_ssh_current_1 = 0;      //��������� ��� 1
signed short gl_ssh_current_2 = 0;      //��������� ��� 2
signed short gl_ssh_Perim_Voltage = 5;  //���������� �������� ����������������
signed short gl_ssh_ampl_angle = 0;     //���������� ���������������� ��������� ��������� ������� ��������� ������� ���. ��������
signed short gl_ssh_Utd1 = 0;           //���������� ���������� ������������
signed short gl_ssh_Utd2 = 0;           //���������� ��������� ������������
signed short gl_ssh_Utd3 = 0;           //���������� ��������� ������������
signed short gl_ssh_Utd1_cal = 0;       //���������� ���������� ������������ (������.)
signed short gl_ssh_Utd2_cal = 0;       //���������� ��������� ������������ (������.)
signed short gl_ssh_Utd3_cal = 0;       //���������� ��������� ������������ (������.)


signed short gl_ssh_angle_hanger = 0;       //������� ���� ���������� ������������
signed short gl_ssh_angle_hanger_prev = 0;  //���������� �������� ���� ���������� ������������

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

#define VPRC_SLIDING_ROUND 10              //���������� ������� ���������� ������� ����������� ���������
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

char gl_b_PerimeterReset = 0;         //���� ������� ������ ��������� (0 = ������ ����������, 1 = ������ ������������, ������ ���������� �����������, 2 = ������ ������������, ������ ��������� �����������)

char gl_b_SA_Processed = 0;           //���� ��������� ��������� ������� SA
char gl_b_SyncMode = 0;               //���� ������ ������ ���������:   0=�����. 1=������.
char bAsyncDu = 0;                    //���� �������� ������� SA ��� ������. ���� � ������. ������: 0-���������� SA 1-���������� dU

short nSentPacksCounter = 0;          //������� �������
int nSentPacksRound = SHORT_OUTPUT_PACK_LEN;    //���� �������� �������
char gl_c_OutPackCounter = 0;         //���������� ������ ������� �������

int ADCChannel = 1;     //�������� ����� ���
                    //0 = ADC1 = 78 ���� = UTD1
                    //1 = ADC2 = 79 ���� = UTD2
                    //2 = ADC3 = 80 ���� = I1
                    //3 = ADC4 =  1 ���� = I2
                    //4 = ADC5 =  2 ���� = CntrPC
                    //5 = ADC6 =  3 ���� = AmplAng

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

char gl_bOutData = 0;       //���� ���������� ������ ������ ������

//������� �������� �������� �����
char gl_bManualLaserOff = 0;          //���� ��� �� ���� ��������� ��� ������ (��������). ���� ����� ����� ��������� ��� ������� � ������������� �������� ����.
int gl_nLaserCurrentUnstableT2 = 0;   //��������� ������ ������� ����� ���� �������� (�������������, � ���� �� ��������� 5 ��� �������� �� ���� - ������������)
int gl_nLaserCurrentUnstableCnt = 0;  //������� �������� � ������� ��������� 5 ���

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���������� ����������
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
  //� �������� ����� ������ ������ �� ����� P3.7
  GP3DAT |= 1 << ( 16 + 7); //�������� ����� p3.7 set
  for( i=0; i<100; i++);
  GP3DAT &= ~( 1 << ( 16 + 7)); //�������� ����� p3.7 clear
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
	// ������ (������� ����� � ��� �������� �����)
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
      //������: ������ dN-dU
      signed int siAngleInc1 = ( signed short) angle_inc1;
      //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
      angle_inc_corr = angle_inc1;
      f_dN = ( float) ( ( signed int) angle_inc1);
      dbl_dN = ( double) ( ( signed int) angle_inc1);
    }
    else {
      //������: ���������� �����
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
    //���������� �����
    double dAngleInc1 = ( double) angle_inc1;
    //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
    angle_inc_corr = ( signed short) ( angle_inc1 * 100.);

    f_dN = ( float) ( ( signed int) angle_inc1);
	  dbl_dN = ( double) ( ( signed int) angle_inc1);
  }

  /*  
  putchar_nocheck( angle_inc_corr & 0xff);
  putchar_nocheck( ( angle_inc_corr & 0xff00) >> 8);*/

  //����������� f_dN �� �������� [-99 310; + 99 310]
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
  //�����. �����: SA TIME
  //������. �����: SA TIME ��� ���������� ���� ��������
  if( gl_b_SyncMode) {
    //������. �����
    if( bAsyncDu) {
      //�������� dU
      putchar_nocheck( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);
      cCheckSumm += ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff);

      putchar_nocheck( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
      cCheckSumm += ( ( ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev) & 0xff00) >> 8);
    }
    else {
      //�������� SA
      putchar_nocheck( gl_ssh_SA_time & 0xff);
      cCheckSumm += ( gl_ssh_SA_time & 0xff);

      putchar_nocheck( ( gl_ssh_SA_time & 0xff00) >> 8);
      cCheckSumm += ( ( gl_ssh_SA_time & 0xff00) >> 8);
    }
  }
  else {
    //���������� �����
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

  //1. ��� ����� ���������
  //�������� TactNoise0 (������� ��� ��������� "��� ����� ���������")
  if( ( flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //�������� TactNoise1 (������� ��� ��������� "��� ����� ���������")
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
  // �������� �����
  //**********************************************************************
  // ��� 0
  DAC0DAT = (( int) ( 4095.0 * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 ) / 3.0)) << 16; //�������� �� ������ ���0 1,0 �
  // ��� 1 (����)
  DAC1DAT = (( int) ( 4095.0 * ( ( double) flashParamMCoeff / 250. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 )) / 3.0)) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0
  //DAC1DAT = (( int) ( 4095.0 * ( ( double) flashParamParam3 / 250. * 0.25) / 3.0)) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0
  // ��� 2 (��������� ����)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) flashParamStartMode / 250. * 2.5) / 3.0)) << 16;
}

void FirstDecrementCoeffCalculation( void) {
  char lb, hb;
  int i, k, val;

  //****
  //������ ������ (��� ���� ������ �����)
  //****

  //�������� ����������� ����� 
  while( !(GP0DAT & 0x10));

  //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
  //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
  // ������ �������������� ���� �������� ������� cnvst->0
  GP3CLR |= 1 << (16 + 7);

  //������� WrCnt
  GP4DAT ^= 1 << (16 + 2);
  //pause( 1);
  GP4DAT ^= 1 << (16 + 2);

  //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
  //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
  // ��� �� cnvst ->1 (����� ������� ��� �� "�������" � shutdown �� ��������� ��������������)
  GP3SET |= 1 << (16 + 7);


  // ��������� ���������� ����:
  //��������� �������� ����� ���������� ����
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

  //��������� �������� ����� ���������� ����
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

  // �������� ���������� ��������� ���� ���������� ������� (������. �����)
  gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
  while( ( GP2DAT & 0x80)) {}                                 //���� ���� busy �� ������ � 0

  //������
  GP1CLR |= 1 << (16 + 7);                                    //cs->0
  for( k=0; k<100; k++);                                      //������� t4
  GP2SET |= 1 << (16 + 2);                                    //sclk->1
  val = 0;

  //������ 14 bit
  for( i=0; i<14; i++) {
    GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0
    for( k=0; k<50; k++);                                    //������� t5
    val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //������ ���
    GP2DAT |= 1 << (16 + 2);                                  //sclk->1
    for( k=0; k<50; k++);                                    //������� t8
  }

  GP1SET |= 1 << (16 + 7);                                    //cs->1
  gl_ssh_angle_hanger_prev = val << 2;
  gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger_prev / 4;

  //�������� ��������� �����
  while( (GP0DAT & 0x10));

  //****
  //� �����������
  //****
  do {
    //�������� ����������� �����
    while( !(GP0DAT & 0x10));

    //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
    //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
    // ������ �������������� ���� �������� ������� cnvst->0
    GP3CLR |= 1 << (16 + 7);


    //������� WrCnt
    GP4DAT ^= 1 << (16 + 2);
    //pause( 1);
    GP4DAT ^= 1 << (16 + 2);

    //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
    //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
    // ��� �� cnvst ->1 (����� ������� ��� �� "�������" � shutdown �� ��������� ��������������)
    GP3SET |= 1 << (16 + 7);


    // ��������� ���������� ����

    //��������� �������� ����� ���������� ����
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

    //��������� �������� ����� ���������� ����
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

    // �������� ���������� ��������� ���� ���������� ������� (������. �����)
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;
    while( ( GP2DAT & 0x80)) {}                                 //���� ���� busy �� ������ � 0

    //������
    GP1CLR |= 1 << (16 + 7);                                    //cs->0
    for( k=0; k<100; k++);                                      //������� t4
    GP2SET |= 1 << (16 + 2);                                    //sclk->1
    val = 0;

    //������ 14 bit
    for( i=0; i<14; i++) {
      GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0
      for( k=0; k<50; k++);                                    //������� t5
      val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //������ ���
      GP2DAT |= 1 << (16 + 2);                                  //sclk->1
      for( k=0; k<50; k++);                                    //������� t8
    }

    GP1SET |= 1 << (16 + 7);                                    //cs->1
    gl_ssh_angle_hanger = val << 2;
    gl_ssh_angle_hanger = gl_ssh_angle_hanger / 4;
      
    //�������� ��������� �����
    while( (GP0DAT & 0x10));

    gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
    gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);

    gl_ssh_angle_inc_prev = gl_ssh_angle_inc;
    gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

    gl_un_DecCoeffStatPoints++;

  } while( gl_un_DecCoeffStatPoints < DEC_COEFF_FIRST_CALCULATION_N);

  //������� ������ �������� ����. ������
  flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

void SendPhaseShift( void) {
  int i, k, base = 1;

  //������� ������
  GP2DAT |= 1 << (16 + 6);                                  //CLR (p2.6) -> 1
  for( k=0; k<100; k++);
  GP2DAT &= ~( 1 << (16 + 6));                              //CLR (p2.6) -> 0
  for( k=0; k<100; k++);

  for( i=0; i<6; i++) {
    //�������� ������ �� ����� DATA
    if( flashParamPhaseShift & base) {
      GP2DAT |= 1 << (16 + 5);                              //DATA (p2.5) -> 1
    }
    else {
      GP2DAT &= ~( 1 << (16 + 5));                          //DATA (p2.5) -> 0
    }

    //����������� ������� SCK
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
  //������� ���������� �������������
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

  //����� dN_dU_Cnt
  char bSimpleDnDu = 0;         //����� ������ dN,dU,counter: ����      0-������� ����� (��������), 1-����� dN_dU_C (�������)
  char cSimpleDnDuCounter = 0;  //����� ������ dN,dU,counter: ������� �����
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
                        //*** 01 - ����� ������� ������ ����� P1.0
                        //*** 00 - Reserved
                        //*** 01 - ����� ������� ������ ����� P1.1
                        //*** 00 - Reserved
                        //*** �������:
                        //***         00    01    10    11
                        //***   P1.0  GPIO  SIN   SCL0  PLAI[0]
                        //***   P1.1  GPIO  SOUT  SDA0  PLAI[1]

  // Start setting up UART
  COMCON0 = 0x080;      // Setting DLAB

  // Setting DIV0 and DIV1 to DL calculated


  //�������:
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
  // ������������ ��������
  //**********************************************************************	
  GP0DAT = 0x01000000;
  GP0DAT ^= (1 << 16+0);
  GP0CLR = (1 << 16+0);

#ifdef DEBUG
  printf("DBG: GPIO lines direction configuration\n");
#endif

  //**********************************************************************
  // ������������ GPIO (��������������� ������/������� ������ ����������)
  //**********************************************************************
  GP0DAT |= 1 << (24 + 0);	//������������ �������� ����� (��������) (p0.0) � �������� ������
  GP0DAT |= 1 << (24 + 3);	//������������ ����� ������������ ������ (p0.3) � �������� ������
  GP0DAT |= 1 << (24 + 5);	//������������ ����� RP_P                (p0.5) � �������� ������

  GP1DAT |= 1 << (24 + 2);	//������������ ����� RdCodeDac           (p1.2) � �������� ������
  GP1DAT |= 1 << (24 + 3);	//������������ ����� RdHbc               (p1.3) � �������� ������
  GP1DAT |= 1 << (24 + 4);	//������������ ����� RdLbc               (p1.4) � �������� ������
  GP1DAT |= 1 << (24 + 7);	//������������ ����� CS                  (p1.7) � �������� ������

  /*
  2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
  GP2DAT |= 1 << (24 + 1);	//������������ ����� CnvSt               (p2.1) � �������� ������
  */

  GP2DAT |= 1 << (24 + 2);	//������������ ����� SClck               (p2.2) � �������� ������	
  GP2DAT |= 1 << (24 + 4);	//������������ ����� RdN10N8             (p2.4) � �������� ������
  GP2DAT |= 1 << (24 + 5);	//������������ ����� ������ ������ Data  (p2.5) � �������� ������
  GP2DAT |= 1 << (24 + 6);	//������������ ����� ������ ������ CLR   (p2.6) � �������� ������

  GP3DAT |= 1 << (24 + 0);	//������������ ����� TactNoise0          (p3.0) � �������� ������
  GP3DAT |= 1 << (24 + 1);	//������������ ����� TactNoise1          (p3.1) � �������� ������ 
  GP3DAT |= 1 << (24 + 3);	//������������ ����� RdN7N0              (p3.3) � �������� ������
  //GP3DAT |= 1 << (24 + 4);	//������������ ����� Start               (p3.4) � �������� ������
  GP3DAT |= 1 << (24 + 5);	//������������ ����� OutLnfType          (p3.5) � �������� ������ 
  GP3DAT |= 1 << (24 + 7);	//������������ �������� �����            (p3.7) � �������� ������

  GP4DAT |= 1 << (24 + 0);	//������������ ����� ONHV                (p4.0) � �������� ������
  GP4DAT |= 1 << (24 + 1);	//������������ ����� OFF3KV              (p4.1) � �������� ������
  GP4DAT |= 1 << (24 + 2);	//������������ ����� WrCnt               (p4.2) � �������� ������
  GP4DAT |= 1 << (24 + 3);	//������������ ����� Reset               (p4.3) � �������� ������ 

#ifdef DEBUG
  printf("DBG: GPIO lines values configuration...\n");
#endif

  //��������� �������� ����������� - ��������
  GP0DAT |= 1 << (16 + 5);      //RP_P       (p0.5) = 1
  GP0SET = 1 << (16 + 5);       //�����

  GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
  GP4SET = 1 << (16 + 0);      //�����

  GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
  GP4SET = 1 << (16 + 1);      //�����

  GP4DAT &= ~( 1 << (16 + 2));  //WrCnt      (p4.2) = 0
  GP4CLR = ( 1 << (16 + 2));   //�����

  GP2DAT &= ~( 1 << (16 + 4));  //RdN10N8    (p2.4) = 0
  GP2CLR = 1 << (16 + 4);      //�����

  GP3DAT &= ~( 1 << (16 + 3));  //RdN7N0     (p3.3) = 0
  GP3CLR = 1 << (16 + 3);      //�����

#ifdef DEBUG
  printf("DBG: Pulsing Reset signal...\n");
#endif

  //**********************************************************************
  // ������� ������� Reset
  //**********************************************************************
  GP4DAT |= 1 << (16 + 3);      //Reset set
  for( i=0; i<100; i++);
  GP4DAT &= ~( 1 << (16 + 3));  //Reset clear

#ifdef DEBUG
  printf("DBG: Enabling UART0 FIQ...\n");
#endif

  //**********************************************************************
  // ��������� ���������� �� UART0
  //**********************************************************************	
  FIQEN |= UART_BIT;
  COMIEN0 |= 1;

#ifdef DEBUG
  printf("DBG: Internal ADC configuration...\n");
#endif

  //**********************************************************************
  // ������������ ���
  //**********************************************************************	
  ADCCON = 0x20;            // �������� ���
  while (time >=0)	        // ���� ��������� � datasheet ����� (5�����) ��� ������� ��������� ���
    time--;

  ADCCON = 0x624;           // ������������� ���:
                            // ����������� �������������� � ����������� �� ���������
                            // ������������ ����
                            // (���������� ������� ���)
                            // ����������� ADCBusy
                            // ��� ������ ��������������
                            // �������������� 8 ������
                            // ������������ [fADC / 4]
  ADCCP = 0x01;             // ������ 1�� ����� ���
  REFCON = 0x00;            // ��������� ���������� ��� �� ���� Vref


#ifdef DEBUG
  printf("DBG: External ADC AD7367 configuration...\n");
#endif

  //**********************************************************************
  // ������������ ��� AD7367
  //**********************************************************************	
  //���������� ���� DOUT, BUSY �� ����
  //��� � ��� �� ���� �� �������
  //���������� ���� SCLK, CNVST, CS �� �����
  //...(��� ���� ���)

  GP1DAT |= 1 << (16 + 7);                                      //cs->1
  GP2DAT &= ~( 1 << (16 + 2));                                  //sclk->0



  //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
  //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
  //���������� (�� ������ ������ �������� t power-up (70 usec))
  GP3DAT |= 1 << (16 + 7);                                      //cnvst -> 1
  for( k=0; k<1000; k++);                                       //70 usec



#ifdef DEBUG
  printf("DBG: Timers configuration...\n");
#endif
  //**********************************************************************
  // ������������ Timer0
  //**********************************************************************
  T0CON = 0x80;

  //**********************************************************************
  // ������������ Timer1
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
  // ������������ �������� �����������
  //**********************************************************************
/*#ifdef DEBUG
  printf("PLLCON=%x. Before change.\n", PLLCON);
#endif*/


  /*
  while(1) {
    for( i=0; i<50; i++) {
      GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
      GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear
    }
    GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
    pause( 1);
    GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear
    pause( 2);
    GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
    pause( 3);
    GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear
    pause_T0( 100);
    GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
    pause_T0( 500);
    GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear
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
  // ������������ Timer2
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
  // ���� ����� ����������
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
  // ������������ ����-������ FlashEE
  //**********************************************************************
  flashEE_configure();

#ifdef DEBUG
  printf("DBG: loading flash params.\n");
#endif

  //**********************************************************************
  // �������� ���������� �� ����-������
  //**********************************************************************
  load_params();
  ThermoCalibrationCalculation();

#ifdef DEBUG
  printf("DBG: DAC Configuration\n");
#endif

  //**********************************************************************
  // ������������ � �������� ���
  //**********************************************************************
  // ��� 0
  DAC0CON = 0x11;       // ������������ ��� 0:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���0 ����������� �� ������� ������ ����� ����

  // ��� 1
  DAC1CON = 0x11;       // ������������ ��� 1:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���1 ����������� �� ������� ������ ����� ����

  // ��� 2
  DAC2CON = 0x11;       // ������������ ��� 2:
                        // �������� 0-DAC(REF)
                        // �������� ���������� ���2 ����������� �� ������� ������ ����� ����
  DACConfiguration();

#ifdef DEBUG
  //printf("PLLCON=%x\n", PLLCON);
  printf("DBG: Hangerup configure\n");
#endif

  //**********************************************************************
  // ������������ ������� ?????
  //**********************************************************************
  configure_hanger();

  // *********************************************************************
  // ������������ �������� ������
  // *********************************************************************
  SendPhaseShift();

  // *********************************************************************
  // ����� 1 ���
  // *********************************************************************
  pause( 32768*1);


  //**********************************************************************
  // �������� �������� ������������
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
  // ������ ������
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




  //��������� �����
  GP4DAT &= ~( 1 << (16 + 0));      //ONHV   (p4.0) = 0
  GP4CLR = ( 1 << ( 16 + 0));      //�����

  //����� 0.5 ��� (32768 * 0.5 = 16384)
  pause( 16384);

  while( 1) {

    //�������� ��� I1
    ADCCP = 0x02;       //ADC2 --> I1
    //pause(10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
    gl_ssh_current_1 = (ADCDAT >> 16);

    //�������� ��� I2
    ADCCP = 0x01;       //ADC1 --> I2
    //pause(10);
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ���
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
      //�� ��������

      if( nFiringTry < 25) {

        if( nFiringTry > 0 && ( nFiringTry % 5) == 0) { //if( nFiringTry % 5) == 0)
#ifdef DEBUG
  printf( "DBG: Relax pause 1 sec\n");
#endif
          //��������� �����
          GP4DAT |= 1 << (16 + 0);        //ONHV       (p4.0) = 1
          GP4SET = 1 << (16 + 0);         //�����

          //����� 1 ���
          pause( 32786);

          //��������� �����
          GP4DAT &= ~( 1 << (16 + 0));    //ONHV   (p4.0) = 0
          GP4CLR = ( 1 << ( 16 + 0));     //�����
        }

#ifdef DEBUG
  printf( "DBG: Applying 3kV for 1 sec\n");
#endif

        //�������� ��������� ������
        GP4DAT &= ~( 1 << (16 + 1));      //OFF3KV (p4.1) = 0
        GP4CLR = ( 1 << ( 16 + 1));       //�����

        //����� 1 ���
        pause( 32786);

        //��������� ��������� ������
        GP4DAT |= 1 << (16 + 1);          //OFF3KV     (p4.1) = 1
        GP4SET = 1 << (16 + 1);           //�����

        //����������� ����� �������
        nFiringTry++;

        //����� 0.5 ���
        pause( 16384);
      }
      else {

#ifdef DEBUG
  printf( "DBG: fireup FAILS\n");
#endif

        //��������� �������
        GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
        GP4SET = 1 << (16 + 0);      //�����

        deadloop_no_firing();         //FAIL.FINISH
      }
    }
    else {
      //SUCCESS! ��������

      #ifdef DEBUG
        printf("DBG: Laser fireup... successfully passed\n");
      #endif

      break;
    }
  }
#endif


  //**********************************************************************
  // �������� ��������� �������
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
  //����� ������������ � ������� ����������� ���������
  GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
  pause( 327);                  //pause 10msec
  GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0
  */

  //**********************************************************************
  //������������
  //**********************************************************************

  GP3DAT |= ( 1 << (16 + 5));	//OutLnfType (p3.5) = 1  ���������� ������������ ����������� ������������

  //**********************************************************************
  // �������� ������� ����� SA. ���� ��� �� ����� � ������� 0.5 ��� - ���������� � ���������� �����
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

    //�������� � ���������� ������ - �������� ������� SA
    prt2val = T2VAL;
    while( 1) {
      if( GP0DAT & 0x10) {
        //SA ������ - ��� ��
        #ifdef DEBUG
          printf("DBG: Got SA signal! SA TEST PASSED\n");
        #endif
        break;
      }

      if( ( double) (( prt2val + T2LD - T2VAL) % T2LD) / 32768. > 0.5) {
        //SA �� ������ � ������� 0.5 ��� - ����� � deadloop

        #ifdef DEBUG
          printf("FAILED\n");
        #endif

        deadloop_no_tact( ERROR_NO_TACT_SIGNAL);
        break;
      }
    }
  }

  /*
  //�������������� ���������� �����
  gl_b_SyncMode = 0;
  GP3DAT &= ~(1 << (16 + 5)); //OutLnfType (p3.5) = 0
  */

  #ifdef DEBUG
    printf("passed\n");
    printf("DBG: Internal ADC start...");
  #endif

#endif

  //**********************************************************************
  // ������ �������������� ���
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
  //������� (����� ? N ������) ������� SA
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

  //��������� ����������� ������� ����������� ���������
  GP0DAT &= ~( 1 << (16 + 5));  //RP_P       (p0.5) = 0
  GP0CLR = ( 1 << (16 + 5));   //�����

//�������� ��������� ������ ������ ���������� (����� ����������)
#ifdef DEBUG
  //�� �� � DEBUG'e!!!!!
  gl_bOutData = 1;
#else
  gl_bOutData = 0;
#endif

  //**********************************************************************
  // �������� ���� ������ ���������
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
    //����� SIMPLE dN_dU_Counter
    if( bSimpleDnDu == 1) {

      while( !(GP0DAT & 0x10)) {}

      if( gl_b_SA_Processed == 0) {

        //����������� � ���������:
        //����� �� �� ���������� ������� SA:
        //1. � ������ ������������ ������ ��������� �������������� ���� ��������
        //2. � ����� ������ ������ ��� ���� ������ ������� WrCnt


        //**********************************************************************
        // ������ ��� ������� ���� ���������� ������� (������. �����)
        //**********************************************************************
        if( gl_b_SyncMode) {
          //����������� �����


          //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
          //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
          //**********************************************************************
          // ������ �������������� ���� �������� ������� cnvst->0
          //**********************************************************************

          //GP3CLR |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 0
          GP3DAT ^= 1 << (16 + 7);

          //**********************************************************************
          //������� WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause_T0( 20);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);  //WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);  //WrCnt clear
          */


          //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
          //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
          //**********************************************************************
          // ��� �� cnvst ->1 (����� ������� ��� �� "�������" � shutdown �� ��������� ��������������)
          //**********************************************************************
          //GP3SET |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 1
          GP3DAT ^= 1 << (16 + 7);

        }
        else {
          //���������� �����

          //**********************************************************************
          //������� WrCnt
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
        // ��������� ���������� ����
        //**********************************************************************

        //��������� �������� ����� ���������� ����
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

        //��������� �������� ����� ���������� ����
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
        // �������� ���������� ��������� ���� ���������� ������� (������. �����)
        //**********************************************************************
        if( gl_b_SyncMode) {
          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

          while( ( GP2DAT & 0x80)) {}                                 //���� ���� busy �� ������ � 0

          //������
          GP1CLR |= 1 << (16 + 7);                                    //cs->0

          //pause_T0( 20);
          //for( k=0; k<10; k++);                                      //������� t4
          //pause( 1);


          GP2SET |= 1 << (16 + 2);                                    //sclk->1
          val = 0;

          //������ 14 bit
          for( i=0; i<14; i++) {
            //GP2CLR |= 1 << (16 + 2);                                  //sclk->0
            GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0

            //pause_T0( 20);
            //for( k=0; k<5; k++);                                    //������� t5
            //pause( 1);

            val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //������ ���

            //GP2SET |= 1 << (16 + 2);                                  //sclk->1
            GP2DAT |= 1 << (16 + 2);                                  //sclk->1

            //pause_T0( 20);
            //for( k=0; k<5; k++);                                    //������� t8
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

        //�������
        cSimpleDnDuCounter++;

        ssh_dN = ( gl_ssh_angle_inc - gl_ssh_angle_inc_prev);
        ssh_dU = ( gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev);

        cCheckSumm = 0;
        //�����.������
        putchar_nocheck( 0xCC);
        //�����.dN
        putchar_nocheck( ssh_dN & 0xFF);            cCheckSumm += ssh_dN & 0xFF;
        putchar_nocheck( ( ssh_dN & 0xFF00) >> 8);  cCheckSumm += ( ssh_dN & 0xFF00) >> 8;
        //�����.dU
        putchar_nocheck( ssh_dU & 0xFF);            cCheckSumm += ssh_dU & 0xFF;
        putchar_nocheck( ( ssh_dU & 0xFF00) >> 8);  cCheckSumm += ( ssh_dU & 0xFF00) >> 8;
        //�����.�������
        putchar_nocheck( cSimpleDnDuCounter);       cCheckSumm += cSimpleDnDuCounter;
        //�����.CS
        putchar_nocheck( cCheckSumm);

        //��������� ���� � ���, ��� ������� ������� ������� SA (����) �� ����������
        gl_b_SA_Processed = 1;
      }
      else {
        while( ( GP0DAT & 0x10)) {}
        gl_b_SA_Processed = 0;
      }
    }



    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //���������� ��� ���������
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

        case 1: //���������� ��� ����� ���������
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          configure_hanger(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 2: //���������� ����������� M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 3: //���������� ��������� ����
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          DACConfiguration();

          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1

          nRppTimer = T2VAL;
          gl_b_PerimeterReset = 1;

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 4: //���������� ����������� ��� I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
/*#ifdef DEBUG
          printf("DBG: Input command (0x04 - SetControlI1) accepted. Param: 0x%04x\n", flashParamI1min);
#endif*/
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 5: //���������� ����������� ��� I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 6: //���������� 1�� ������� ������� AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 7: //���������� ����������� ������
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 8: //���������� �������� ����������� dU
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);

          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 9: //� ������. ������ �������� ����� SA (�������� ����� dU)
          bAsyncDu = 1;
        break;

        case 10: //� ������. ������ ��������� ����� dU (�������� ����� SA)
          bAsyncDu = 0;
        break;

        case 11: //���������� ������������� (�������� - �������� �����������)
          bCalibrated = 0;
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
              flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //� ��� ���� ���������� ����������� ����� ����������

              //������� �� ��, ����� �� ������ ��� ����� ������ ����
              if( in_param_temp == flashParam_calibT1) {
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
                break;
              }

              if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
                  flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
                //� ��� ���� ���������� ����������� � ������������ ����� ����������
                //��������� ����� ���� ��������
                if( in_param_temp < flashParam_calibT1) {
                  //���� �������� �����������
                  bCalibProcessState = 1;
                  flashParam_calibT1 = in_param_temp;
                }
                else {
                  //���� �������� ������������
                  bCalibProcessState = 4;
                  flashParam_calibT2 = in_param_temp;
                }
              }
              else {
                //� ��� ���� ���������� ����������� ����� ����������, �� ��� ���������� ������������
                bCalibProcessState = 3;
                flashParam_calibT2 = in_param_temp;
              }

          }
          else {
            //� ��� ��� ���� ����������� �����!! ������ ��� ����� ����������� :)
            flashParam_calibT1 = in_param_temp;
            bCalibProcessState = 1;
          }
        break;

        case 12:    //������� ������ ������ ���������� �������������
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

        case 14:    //������� ������ �������� ������
          in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          flashParamPhaseShift = in_param_temp;
          SendPhaseShift();
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 15:    //������� ���������� ������
          GP4DAT |= 1 << (16 + 0);	    //ONHV       (p4.0) = 1
          GP4DAT |= 1 << (16 + 1);	    //OFF3KV     (p4.1) = 1
          gl_bManualLaserOff = 1;
        break;

        case 16:    //������� ���������� �����������
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_b_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
        //nRppTimer = ;               //<-- ����� �� ��������! ���� �� ��������� ������� (������ �� ������� �������� ����������)
        break;

        case 17:    //������� ��������� �����������
          GP0DAT &= ~( 1 << (16 + 5));   //RP_P   (p0.5) = 0 (���������)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          gl_b_PerimeterReset = 2;      //������������� ���� ��������������� ������ (2=������ ���������, ������ ������������)
          nRppTimer = T2VAL;            //<-- �������� �����! � ����� [����������� �����] ���� ����� ����
        break;

        case 18:    //������� ������ �����������
          GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;

          nRppTimer = T2VAL;            //<-- �������� �����! � ����� [����������� �����] ���������� ���������, ���� ������� � ��������� 2, � ��� ����� [����������� �����] ���� ������� � ��������� 0 (������ ����������)
          gl_b_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
        break;

        case 49: //������ ������� ����� ����������                          49 = 0x31 = "1"
          nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        case 50: //��������� ��������� �� ���� ������                       50 = 0x32 = "2"
          save_params();
        break;

        case 51: //������������� ��������� �� ����-������ � �������� ��     51 = 0x33 = "3"
          load_params(); nSentPacksRound = LONG_OUTPUT_PACK_LEN;
        break;

        //DEVICE_id
        case 'S': //������ DeviceID                                         83 = 0x53 = "S"
          #ifdef DEBUG
            printf("DBG: 'S' cmd in\n");
          #endif
          nSentPacksCounter = 26; nSentPacksRound = DEVICE_NUM_PACK_LEN;
        break;

        case 'T': //���������� DeviceID                                     84 = 0x54 = "T"
          flashParamDeviceId = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
        break;

        //DATE
        case 'U': //������ Date                                             85 = 0x55 = "U"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'V': //��������� Date.YEAR                                     86 = 0x56 = "V"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'W': //��������� Date.MONTH                                    87 = 0x57 = "W"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        case 'X': //��������� Date.DAY                                      88 = 0x58 = "X"
          nSentPacksCounter = 44; nSentPacksRound = DATE_PACK_LEN;
        break;

        //HV_apply_duration
        case 'Y': //������ Hv.apply.duration                                89 = 0x59 = "Y"
          nSentPacksCounter = 47; nSentPacksRound = HV_APPLY_TIME_LEN;
        break;

        //ORGANIZATION
        case 'a': //������ Organization                                     97 = 0x61 = "a"
          nSentPacksCounter = 28; nSentPacksRound = ORG_NAME_PACK_LEN;
        break;

        case 'b': //���������� Organization.byte1                           98 = 0x62 = "b"
          flashParamOrg[0] = input_buffer[1];        break;
        case 'c': //���������� Organization.byte2                           99 = 0x63 = "c"
          flashParamOrg[1] = input_buffer[1];        break;
        case 'd': //���������� Organization.byte3                           100= 0x64 = "d"
          flashParamOrg[2] = input_buffer[1];        break;
        case 'e': //���������� Organization.byte4                           101= 0x65 = "e"
          flashParamOrg[3] = input_buffer[1];        break;
        case 'f': //���������� Organization.byte5                           102= 0x66 = "f"
          flashParamOrg[4] = input_buffer[1];        break;
        case 'g': //���������� Organization.byte6                           103= 0x67 = "g"
          flashParamOrg[5] = input_buffer[1];        break;
        case 'h': //���������� Organization.byte7                           104= 0x68 = "h"
          flashParamOrg[6] = input_buffer[1];        break;
        case 'i': //���������� Organization.byte8                           105= 0x69 = "i"
          flashParamOrg[7] = input_buffer[1];        break;
        case 'j': //���������� Organization.byte9                           106= 0x6A = "j"
          flashParamOrg[8] = input_buffer[1];        break;
        case 'k': //���������� Organization.byte10                          107= 0x6B = "k"
          flashParamOrg[9] = input_buffer[1];        break;
        case 'l': //���������� Organization.byte11                          108= 0x6C = "l"
          flashParamOrg[10] = input_buffer[1];        break;
        case 'm': //���������� Organization.byte12                          109= 0x6D = "m"
          flashParamOrg[11] = input_buffer[1];        break;
        case 'n': //���������� Organization.byte13                          110= 0x6E = "n"
          flashParamOrg[12] = input_buffer[1];        break;
        case 'o': //���������� Organization.byte14                          111= 0x6F = "o"
          flashParamOrg[13] = input_buffer[1];        break;
        case 'p': //���������� Organization.byte15                          112= 0x70 = "p"
          flashParamOrg[14] = input_buffer[1];        break;
        case 'q': //���������� Organization.byte16                          113= 0x71 = "q"
          flashParamOrg[15] = input_buffer[1];        break;

        case 'r': //FAKE! ���������� ��� ������ (���� ������ �������)       114= 0x72 = "r"
          gl_c_EmergencyCode = input_buffer[1];
        break;

        case 's': //���������-���������� ������ dN_dU_counter               115= 0x73 = "s"
          bSimpleDnDu ^= 1;
        break;

      }


      pos_in_in_buf = 0;
    }


    if( bSimpleDnDu == 1) {
      continue;
    }

    /*
    //�������� ����� - �������� ����� 10(5?) ��� � �������
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;
    */

    //if( bSAFake) {	// �� ���� P0.4 ���� FAKE ������
    if( GP0DAT & 0x10) {  //�� ���� P0.4 ���� ������

      if( gl_b_SA_Processed == 0) { //���� � ���� SA ����� �� ��� ��� �� ������������

        gl_ssh_SA_time = ( unsigned short) ( ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD);
        gl_n_prT1VAL = T1VAL;

        //� �������� ����� ������ ������ �� ����� P0.0
        GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
        /*for( i=0; i<100; i++);
        GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear*/

        //����������� � ���������:
        //����� �� �� ���������� ������� SA:
        //1. � ������ ������������ ������ ��������� �������������� ���� ��������
        //2. � ����� ������ ������ ��� ���� ������ ������� WrCnt


        //**********************************************************************
        // ������ ��� ������� ���� ���������� ������� (������. �����)
        //**********************************************************************
        if( gl_b_SyncMode) {
          //����������� �����


          //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
          //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
          //**********************************************************************
          // ������ �������������� ���� �������� ������� cnvst->0
          //**********************************************************************

          //GP3CLR |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 0
          GP3DAT ^= 1 << (16 + 7);

          //**********************************************************************
          //������� WrCnt
          //**********************************************************************
          GP4DAT ^= 1 << (16 + 2);
          //pause( 1);
          GP4DAT ^= 1 << (16 + 2);
          /*
          GP4SET |= 1 << (16 + 2);  //WrCnt set
          for( i=0; i<100; i++);
          GP4CLR |= 1 << (16 + 2);  //WrCnt clear
          */


          //2014-09-17 ���������� ��� ������� CnvSt ������ ������� � � ����� ����� �������... ���������
          //2015-02-11 ����������� ��� ������� ����� ������� ����� CNVST. ���������� ������������ ���
          //**********************************************************************
          // ��� �� cnvst ->1 (����� ������� ��� �� "�������" � shutdown �� ��������� ��������������)
          //**********************************************************************
          //GP3SET |= 1 << (16 + 7);    //CNVST = TEST_p3.7 -> 1
          GP3DAT ^= 1 << (16 + 7);

        }
        else {
          //���������� �����

          //**********************************************************************
          //������� WrCnt
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
        // ��������� ���������� ����
        //**********************************************************************

        //��������� �������� ����� ���������� ����
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

        //��������� �������� ����� ���������� ����
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
        // �������� ���������� ��������� ���� ���������� ������� (������. �����)
        //**********************************************************************
        if( gl_b_SyncMode) {
          gl_ssh_angle_hanger_prev = gl_ssh_angle_hanger;

          while( ( GP2DAT & 0x80)) {}                                 //���� ���� busy �� ������ � 0

          //������
          GP1CLR |= 1 << (16 + 7);                                    //cs->0

          //for( k=0; k<10; k++);                                      //������� t4
          //pause( 1);

          GP2SET |= 1 << (16 + 2);                                    //sclk->1
          val = 0;

          //������ 14 bit
          for( i=0; i<14; i++) {
            //GP2CLR |= 1 << (16 + 2);                                  //sclk->0
            GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0

            //for( k=0; k<5; k++);                                    //������� t5
            //pause( 1);

            val += ( ( ( GP1DAT & 0x40) >> 6) << (13-i));             //������ ���

            //GP2SET |= 1 << (16 + 2);                                  //sclk->1
            GP2DAT |= 1 << (16 + 2);                                  //sclk->1

            for( k=0; k<5; k++);                                    //������� t8
            //pause( 1);
          }

          GP1SET |= 1 << (16 + 7);                                    //cs->1
          gl_ssh_angle_hanger = val << 2;
          gl_ssh_angle_hanger = gl_ssh_angle_hanger / 4;
          //printf("\n** %d (codes) = %f V\n\n", si, ( double) si / 8192. * 5.);
        }


        //**********************************************************************
        // ��������� ����������� ���������
        //**********************************************************************
        while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)
        
        switch( ADCChannel) { //����������� ��� �� ������������ � ��������� � ��������������� ����������

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
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 0) {
                flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 2;
              }
            break;

            case 2:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 1) {
                flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 3;
              }
            break;

            case 3:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 2) {
                flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;

            case 4:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 0) {
                flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                bCalibProcessState = 5;
              }
            break;

            case 5:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 1) {
                flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                bCalibProcessState = 6;
              }
            break;

            case 6:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 2) {
                flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                bCalibProcessState = 0;
                SaveThermoCalibPoint();
                nSentPacksRound = LONG_OUTPUT_PACK_LEN;
              }
            break;
          }

          if( !bCalibProcessState)
            //���� ��� ����������� ���������� ����� ���� ����� - ������������� ������������� ���������
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


        ADCCP = 0x01 + ADCChannel;              //���������� ����� ����� ���
        //pause(10);
        ADCCON |= 0x80;                         //������ ������ �������������� (���� ����� � ��������� ����� SA)


        //**********************************************************************
        // ��������� ����� ��������� �� �������
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
        // ������ ������ �������� ���������
        //**********************************************************************
        if( gl_bOutData == 1) {
          switch( nSentPacksCounter) {
            case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_ssh_Utd1);      break;      //UTD1
            //case 0: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 0, gl_ssh_ampl_angle);      break; //AmplAng � ����

            case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_ssh_Utd2);      break;      //UTD2
            //case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, gl_un_RULAControl); break;     //RULA
            //case 1: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 1, dMeanImps); break;             //dMeanImps (������ ������� �� �������� ���������� � �������� ���������)

            case 2: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 2, gl_ssh_Utd3);      break;      //UTD3

            case 3: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 3, gl_ssh_current_1);      break; //I1
            case 4: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 4, gl_ssh_current_2);      break; //I2
            case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_Perim_Voltage);  break; //CntrPc

            case 6: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 6, gl_ush_MeanImpulses); break; //AmplAng �� �������
            //case 5: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 5, gl_ssh_ampl_angle); break; //AmplAng � ����

            case 7: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 7, flashParamAmplitudeCode);    break; //��� ���������
            case 8: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 8, flashParamTactCode);    break;      //��� ����� ���������
            case 9: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 9, flashParamMCoeff);    break;        //����������� �
            case 10: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 10, flashParamStartMode);    break;     //��������� ����
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
            case 43: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 43, flashParamOrg[15]);        break;    //Organization.Byte16    ��� ������������ 0 �� �����!!!!!

            case 44: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 44, flashParamDateYear);     break;    //Date.Year
            case 45: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 45, flashParamDateMonth);    break;    //Date.Month
            case 46: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 46, flashParamDateDay);      break;    //Date.Day

            case 47: send_pack( ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536, 47, /*(( n3kvApplyStart + T2LD - n3kvApplyEnd) % T2LD)*/ 0 );    break;    //HV apply duration
          }
        }

        //������� �������������� ������������ ������
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
        // �������� �� ���������� ������
        // ************************************************************************************
        if( gl_bManualLaserOff == 0 && nSentPacksCounter == 2) {
          //���� �� �� ��������� ��������

          //����������� 5 ��� � ���������� ������� �������� �����
          if( gl_nLaserCurrentUnstableT2 != 0) {
            if( ( gl_nLaserCurrentUnstableT2 + T2LD - T2VAL) >= 163840) { //5sec (*32kHz = 163840)
              //5��� - �������� �� ����. ���������� ����.
              gl_nLaserCurrentUnstableT2 = 0;
              gl_nLaserCurrentUnstableCnt = 0;
            }
          }

          //����� ���� ������ � ��������� 0.4mA +-10% = [0.36;0.44]
          //������� ��� ������ ����:
          //1912 --> 0.439844 mA
          //2184 --> 0.360156 mA

          //����� ���� ������ � ��������� 0.5mA +-10% = [0.45;0.55]
          //������� ��� ������ ����:
          //1537 --> 0.549707 mA
          //1877 --> 0.450098 mA

          if( gl_ssh_current_1 >= 1537 && gl_ssh_current_1 <= 1877 &&
              gl_ssh_current_2 >= 1537 && gl_ssh_current_2 <= 1877) {
              //� ������ �� � �������
          }
          else {
            //�������� �������� ����� ������ ���
            gl_nLaserCurrentUnstableT2 = T2VAL; //���������� ����� (�� ���������� 5 ��� �� ���� �� ������� ������� ������)
            gl_nLaserCurrentUnstableCnt++;      //����������� ������� ������

            if( gl_nLaserCurrentUnstableCnt >= 5) {
              //�� ������� 5 ��� ����� "�������" �� 5 ��� - ��������� ���

              GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
              GP4SET = 1 << (16 + 0);      //�����
              deadloop_current_unstable();
            }
          }
        }

        // ************************************************************************************
        // 2010-04-22
        // 2017-01-17 introducing moving average
        //�������������� ����������� ���������
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

        //����������� ������� ������������ ������� ������
        nSentPacksCounter++;
        if     ( nSentPacksCounter == LONG_OUTPUT_PACK_LEN) { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == DEVICE_NUM_PACK_LEN)  { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == ORG_NAME_PACK_LEN)    { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == DATE_PACK_LEN)        { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else if( nSentPacksCounter == HV_APPLY_TIME_LEN)    { nSentPacksCounter = 0; nSentPacksRound = SHORT_OUTPUT_PACK_LEN; }
        else
          //�� ��� ������� ������ ������� ����� � 7 �������
          nSentPacksCounter = ( nSentPacksCounter) % nSentPacksRound;

        //**********************************************************************
        //������������ ������� ��������� ��������� ��������� (��������� ����� ��������� ������� ���� ����� ��������� ������)
        //**********************************************************************
        gl_sn_MeaningCounter = ( ++gl_sn_MeaningCounter) % gl_sn_MeaningCounterRound;

        //���������� ���� ���������� ���������� RULA
        if( !gl_sn_MeaningCounter) {

          //�� �������... ����� �������� ��������� � ���� ����� ���������
          dMeanImps = dMeaningSumm / ( double) gl_sn_MeaningCounterRound;
          dMeanImps = dMeanImps / 4.;

          //�� ����.... ��������� � ������ �� �������� 4095=2,5�,
          //����� ������� 2,2�=120"
          //� �������� �� ���������� ����������� 2,9 �� �������� ����� ���������
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

          //���������� ��������� "��������" (���� ��� ��� > 1)
          nDelta = ( int) ( ( double) nDelta / 1.5);
          if( nDelta < 1) {
            nDelta = 1;
          }

          /*
          //��������� "�����������" ���������� ����� 7 ������
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

          //��������� ������ ������ ����� 2.5 ������
          if( nT2RepeatBang) {
            if( ( T2LD + nT2RepeatBang - T2VAL) % T2LD >= 32768 * 1.8) {
              gl_bOutData = 1;
              nSentPacksCounter = 0;
              nSentPacksRound = LONG_OUTPUT_PACK_LEN;
            }
          }

          //������� ����� (����� ���������� ����������) ��� �� �������������� ��������
          //2014.10.09 - ������� ��� ���� ������� ������� - ��������� � �����������
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
          //"�����������" ���������� (��������� ������� �������� ������)
          if( gl_sn_MeaningCounterRound == MEANING_IMP_PERIOD_STABLE) {
            if( abs( flashParamAmplitudeCode - dMeanImps) > 5) {
              nDelta = ( RULA_MAX - RULA_MIN) / 4;
              gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
            }
          }*/


        }
        else {
          dMeaningSumm += gl_ush_MeanImpulses;    //�� �������
          //dMeaningSumm += gl_ssh_ampl_angle;    //�� ����
        }




        //**********************************************************************
        //��������� ����� ������ RP_P
        //**********************************************************************
        if( nRppTimer != 0) {
          if( (( T2LD + nRppTimer - T2VAL) % T2LD) > 32768. * 0.5) {    //������������ RESET� ��� (1s = 0.5s + 0.5s)
            if( gl_b_PerimeterReset == 1) {
              //������ ������ ����� (0.5 ��� ����� ���������� �����������) ==> �������� ����������, � �������� ��� 0.5sec
              //��������� ����������� � ������� ����������� ���������
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0

              //�������� �������� ������
              nRppTimer = T2VAL;
              gl_b_PerimeterReset = 2;
            }
            else if( gl_b_PerimeterReset == 2) {
              //������ ������ ����� (0.5 ��� ����� ��������� �����������) ==> ������� ��� ���������� ����������, � ������ ����������, � ������ ��� ������ �� �����
              //���������� ���� ������������� ������ � �������� "����������"
              gl_b_PerimeterReset = 0;

              //������ �������� ������ �� ����
              nRppTimer = 0;
              
            }
            else {
              //����� �������� ��������, �� ������� ������� �� ��� ����� �� ����������
              //���������� ���� ������������� ������ � �������� "����������"
              gl_b_PerimeterReset = 0;

              //������ �������� ������ �� ����
              nRppTimer = 0;
            }
          }
        }

        //��������� ���� � ��� ��� ������� ������� ������� SA �� ����������
       gl_b_SA_Processed = 1;

        //� �������� ����� ������ ������ �� ����� P0.0
        /*GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
        for( i=0; i<100; i++);*/
        GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear
      }
    }
    else {
      //���� ����� ������� SA � ������ ������ - �� ��� ������ ��� ���������� �������� ����� �������������� ����
      gl_b_SA_Processed = 0;

      /*
      //�������� ������������
      ush_SA_check_time = ( T2LD + gl_ssh_prT2VAL - T2VAL) % T2LD;

      //2 sec = 32768 * 2.0 = 65536

      //��� ���� � �������� - ��� � �������
      //��������!!!!!
      //����������� UNSIGNED SHORT  �� ����� ���� �� ����� ������ 65535
      if( ush_SA_check_time > 65536) {
        //������� ������������

        //��������� �������
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

        deadloop_no_tact( ERROR_TACT_SIGNAL_LOST);

      }
      */
    }
  }
}