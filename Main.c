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
#include "AnalogueParamsConstList.h"
#include "debug.h"
#include "Main.h"

#define RULA_MAX 4090
#define RULA_MIN 25

extern void processIncomingCommand();

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
//page 1
unsigned short gl_ush_flashParamAmplitudeCode = 90;    //��������� ��������� ������������
unsigned short gl_ush_flashParamTactCode = 0;          //��� �����
unsigned short gl_ush_flashParamMCoeff = 4;            //����������� ���������
unsigned short gl_ush_flashParamStartMode = 5;         //��������� ����
unsigned short gl_ush_flashParamDecCoeff = 0;          //����������� ������
unsigned short gl_ush_flashLockDev = 0;                //���� ���������� ����������

//page 2
unsigned short gl_ush_flashParamI1min = 0;             //����������� �������� ���� ������� I1
unsigned short gl_ush_flashParamI2min = 0;             //����������� �������� ���� ������� I2
unsigned short gl_ush_flashParamAmplAngMin1 = 0;       //����������� �������� �������� � ����
unsigned short gl_ush_flashParamHvApplyCount = 0;      //HV_applies tries amount
unsigned short gl_ush_flashParamHvApplyDurat = 0;      //HV_applies tries duration [msec]
unsigned short gl_ush_flashParamHvApplyPacks = 0;      //HV_applies packs

//page 3
unsigned short gl_ush_flashParamSignCoeff = 2;         //�������� �����������
unsigned short gl_ush_flashParamDeviceId = 0;          //ID ����������
unsigned short gl_ush_flashParamDateYear = 0;          //���� �������.���
unsigned short gl_ush_flashParamDateMonth = 0;         //���� �������.�����
unsigned short gl_ush_flashParamDateDay = 0;           //���� �������.����
char gl_ac_flashParamOrg[17] = { 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0};    //�������� �����������

//���������� �������������
signed short    gl_ush_flashParam_calibT1;
unsigned short  gl_ush_flashParamT1_TD1_val, gl_ush_flashParamT1_TD2_val, gl_ush_flashParamT1_TD3_val;
signed short    gl_ush_flashParam_calibT2;
unsigned short  gl_ush_flashParamT2_TD1_val, gl_ush_flashParamT2_TD2_val, gl_ush_flashParamT2_TD3_val;



//********************
//���������� ���������� �� ���� �������
//********************
char gl_c_EmergencyCode = 0;    //��� ������ �������

char gl_acInput_buffer[6] = { 0, 0, 0, 0, 0, 0}; //����� �������� ������
char gl_cPos_in_in_buf = 0;                     //������� ������ � ������ �������� ������


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


//unsigned char cRulaH = RULA_MAX, cRulaL = RULA_MIN;
//unsigned char cRULAControl = 40; //( RULA_MAX - RULA_MIN) / 2;      127 = 1.25V (Dac0)

//unsigned int gl_un_RULAControl = 0;       //0    = 0.000 V
//unsigned int gl_un_RULAControl = 64;      //64   = 0.039 V
//unsigned int gl_un_RULAControl = 1638;    //1638 = 1.000 V
unsigned int gl_un_RULAControl = 3276;      //3276 = 2.000 V
//unsigned int gl_un_RULAControl = 4095;    //4095 = 2.500 V


unsigned int nDelta = ( RULA_MAX - RULA_MIN) / 4;

//������� ������������ ���������
int   gl_snMeaningCounter = 0;              //������� �������
int   gl_snMeaningCounterRound = 128;       //���������� ��������
int   gl_snMeaningShift = 7;                //������� - ��������� �������� ����� (log2 �� gl_snMeaningCounterRound)
long  gl_lnMeaningSumm = 0;                //����� ��������
long  gl_lnMeanImps = 0;                   //������� ��������� (� ��������� ������. ��������)
int   gl_nActiveRegulationT2 = 0;           //������� ������� ��� ������ ����� �������� ����������� ���������, �� �� ���� ����������� ���������

//������� ������� ��� ������� "����� X ��� ����� �������"
int   gl_nT2StartDataOut;

//������� ������� ��� ������� ��������� ������� ������ ���������
int   gl_nRppTimer = 0;

//�����
char gl_b_SyncMode = 0;               //���� ������ ������ ���������:   0=�����. 1=������.
char gl_chAngleOutput = 0;            //���� ������ ���������� ����: 0 = dW (4b)         1 = dN (2b), dU(2b)
char gl_chLockBit = 0;                //���� ������������ ����������: 0 - ����� "���������"   1 - ����� "������������"
char gl_bOutData = 0;                 //���� ������ ������ ������ (���������� ����� 2.5 ��� ����� ��������� �������) 0 - ��� ������; 1 - ������ ������;
char gl_b_PerimeterReset = 0;         //���� ������� ������ ��������� (0 = ������ ����������, 1 = ������ ������������, ������ ���������� �����������, 2 = ������ ������������, ������ ��������� �����������)
char gl_b_SA_Processed = 0;           //���� ��������� ��������� ������� SA
char gl_bManualLaserOff = 0;          //���� ��� �� ���� ��������� ��� ������ (��������). ���� ����� ����� ��������� ��� ������� � ������������� �������� ����.

short gl_nSentPackIndex;                //������ ���������� �������
char gl_c_OutPackCounter = 0;           //���������� ������ ������� �������

//������� �������� �������� �����
int gl_nLaserCurrentUnstableT2 = 0;   //��������� ������ ������� ����� ���� �������� (�������������, � ���� �� ��������� 5 ��� �������� �� ���� - ������������)
int gl_nLaserCurrentUnstableCnt = 0;  //������� �������� � ������� ��������� 5 ���

//������� �������� �������� ��������
int gl_nVibrationWatchT2 = 0;        //��������� ������ ������� ����� ���� �������� (�������������, � ���� �� ��������� 5 ��� �������� �� ���� - ������������)
int gl_nVibrationWatchCnt = 0;       //������� �������� �������� � ������� ��������� 5 ���

int ADCChannel = 0;           //�������� ����� ���
                              //0 = ADC1 = 78 ���� = I2
                              //1 = ADC2 = 79 ���� = I1
                              //2 = ADC3 = 80 ���� = UINT (previous U_td3)
                              //3 = ADC4 =  1 ���� = CntrPC
                              //4 = ADC5 =  2 ���� = UTD1
                              //5 = ADC6 =  3 ���� = UTD2
                              //6 = ADC7 = ?? ���� = UTD3
                              //7 = ADC8 = ?? ���� = AmplAng

//����� ��������� HV ��� �������
unsigned short gl_ushFiringTry = 0;

char gl_cCalibProcessState;

char bCalibrated;
double TD1_K, TD1_B;
double TD2_K, TD2_B;
double TD3_K, TD3_B;

//2014-08-27 - enabling external oscillator
static int new_clock_locked = 0;



////////////////////////////////////////////////////////////////////////////////////////////////////////////
//���������� ����������
void FIQ_Handler (void) __fiq
{
  char tmp;
  if( ( FIQSTA & UART_BIT) != 0) {
    if( gl_cPos_in_in_buf < IN_COMMAND_BUF_LEN)
      gl_acInput_buffer[ gl_cPos_in_in_buf++] = COMRX;
    else
      tmp = COMRX;
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

void send_pack( short analog_param) {
  short param_indicator = gl_nSentPackIndex;
  signed short angle_inc1 = ( 65536 + gl_ssh_angle_inc - gl_ssh_angle_inc_prev) % 65536;
  char cCheckSumm = 0;
  char *pntr;
  char b1, b2, b3, b4;

#ifndef DEBUG
  //float angle_inc_corr;
  signed short angle_inc_corr;
  float f_dN;
  double dbl_dN;

  float Coeff = (( float) gl_ush_flashParamDecCoeff) / 65535.;

  signed short ssh_dN, ssh_dU;
  signed int n_dN, n_dU;
  double result;

  char bt;
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
    //����������� �����

    ssh_dN = gl_ssh_angle_inc - gl_ssh_angle_inc_prev;
    ssh_dU = gl_ssh_angle_hanger - gl_ssh_angle_hanger_prev;

    if( gl_chAngleOutput == 1) {
      //����������� �����: ������ dN, dU
      pntr = ( char *) &ssh_dN;
      b1 = pntr[0];
      b2 = pntr[1];

      pntr = ( char *) &ssh_dU;
      b3 = pntr[0];
      b4 = pntr[1];
    }
    else {
      //����������� �����: ������ phi
      /*
      double db_dN1 = ( double) gl_ssh_angle_inc_prev;
      double db_dN2 = ( double) gl_ssh_angle_inc;
      double dbU1 = ( double) gl_ssh_angle_hanger_prev;
      double dbU2 = ( double) gl_ssh_angle_hanger;
      */

      n_dN = ( signed int) ssh_dN;
      n_dU = ( signed int) ssh_dU;

      result =  ( double) n_dN - ( ( double) n_dU) * Coeff * (( signed short) gl_ush_flashParamSignCoeff - 1);
      //printf("\n%.3f %.3f %.3f %.3f %.3f\n", db_dN1, db_dN2, dbU1, dbU2, result);
      angle_inc_corr = ( signed short) ( result * 100.);

      f_dN = ( float) ( ( signed short) result);
      dbl_dN = ( double) ( ( signed short) result);

      //����������� f_dN �� �������� [-99 310; + 99 310]
      dbl_dN = ( dbl_dN / 99310.) * 2147483647.;
      n_dN = ( int) dbl_dN;

      pntr = ( char *) &n_dN;
      b1 = pntr[0];
      b2 = pntr[1];
      b3 = pntr[2];
      b4 = pntr[3];
    }

    putchar_nocheck( b1);   cCheckSumm += b1;
    putchar_nocheck( b2);   cCheckSumm += b2;
    putchar_nocheck( b3);   cCheckSumm += b3;
    putchar_nocheck( b4);   cCheckSumm += b4;
  }
  else {
    //���������� �����
    double dAngleInc1 = ( double) angle_inc1;
    //angle_inc_corr = (( float) (  siAngleInc1)) * 10.;
    angle_inc_corr = ( signed short) ( angle_inc1 * 100.);

    f_dN = ( float) ( ( signed int) angle_inc1);
    dbl_dN = ( double) ( ( signed int) angle_inc1);

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
  }


  //***************************************************************************
  //ANALOG PARAMETER INDICATOR
  //***************************************************************************
  putchar_nocheck( param_indicator & 0xFF);
  cCheckSumm += ( param_indicator & 0xFF);

  //***************************************************************************
  //ANALOG PARAMETER
  //***************************************************************************
  putchar_nocheck( analog_param & 0xFF);
  cCheckSumm += (analog_param & 0xFF);

  putchar_nocheck( ( analog_param & 0xFF00) >> 8);
  cCheckSumm += ( ( analog_param & 0xFF00) >> 8);

  //***************************************************************************
  //SA TIME
  putchar_nocheck( gl_ssh_SA_time & 0xFF);
  cCheckSumm += ( gl_ssh_SA_time & 0xFF);

  putchar_nocheck( ( gl_ssh_SA_time & 0xFF00) >> 8);
  cCheckSumm += ( ( gl_ssh_SA_time & 0xFF00) >> 8);

  //***************************************************************************
  //PACK COUNTER
  //***************************************************************************
  putchar_nocheck( gl_c_OutPackCounter);
  cCheckSumm += gl_c_OutPackCounter;

  gl_c_OutPackCounter++;

  //***************************************************************************
  //EMERGENCY CODE
  //***************************************************************************
  //8 bit - 0x80 - veracity
  bt = ( gl_b_PerimeterReset ? 0x80 : 0x00);

  //7 bit - 0x40 - lock bit
  bt += ( gl_chLockBit ? 0x40 : 0x00);

  //6 bit - 0x20 - Sync (0)/Async(1) regime
  bt += ( gl_b_SyncMode ? 0x20 : 0x00);

  //5 bit - 0x10 - W (0) / dNdU (1) regime
  bt += ( gl_chAngleOutput ? 0x10 : 0x00);

  //Error code (lower 4 byte)
  bt += gl_c_EmergencyCode;

  putchar_nocheck( bt);
  cCheckSumm += bt;

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
  if( ( gl_ush_flashParamTactCode & 0x01))  //Set TactNoise0 to TactCode parameter bit0
    GP3DAT |= ( 1 << (16 + 0));
  else
    GP3DAT &= ~( 1 << (16 + 0));

  //�������� TactNoise1 (������� ��� ��������� "��� ����� ���������")
  if( ( gl_ush_flashParamTactCode & 0x02))  //Set TactNoise1 to TactCode parameter bit1
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
  DAC1DAT = (( int) ( 4095.0 * ( ( double) gl_ush_flashParamMCoeff / 250. * ( ( double) gl_un_RULAControl / ( double) RULA_MAX * 2.5 )) / 3.0)) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0
  //DAC1DAT = (( int) ( 4095.0 * ( ( double) flashParamParam3 / 250. * 0.25) / 3.0)) << 16;  //(1.0) - ��� RULA � ������� ������� �� DAC0
  // ��� 2 (��������� ����)
  DAC2DAT = (( int) ( 4095.0 * ( ( double) gl_ush_flashParamStartMode / 250. * 2.5) / 3.0)) << 16;
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
  gl_ush_flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);

  gl_un_DecCoeffStatPoints = 0;
  gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
}

/*
2015.09.24 ������ ������� �����
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

*/

void ThermoCalibrationCalculation( void)
{
  double x1, y1, x2, y2;
  //������� ���������� �������������
  if( gl_ush_flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
      gl_ush_flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION) &&

      gl_ush_flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) &&
      gl_ush_flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {


    //TD1
    x1 = gl_ush_flashParam_calibT1 - THERMO_CALIB_PARAMS_BASE;
    x2 = gl_ush_flashParam_calibT2 - THERMO_CALIB_PARAMS_BASE;

    y1 = gl_ush_flashParamT1_TD1_val;
    y2 = gl_ush_flashParamT2_TD1_val;

    TD1_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD1_K = ( y2 - y1) / ( x2 - x1);


    //TD2
    y1 = gl_ush_flashParamT1_TD2_val;
    y2 = gl_ush_flashParamT2_TD2_val;

    TD2_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD2_K = ( y2 - y1) / ( x2 - x1);


    //TD3
    y1 = gl_ush_flashParamT1_TD3_val;
    y2 = gl_ush_flashParamT2_TD3_val;

    TD3_B = ( x2 * y1 - x1 * y2) / ( x1 - x2);
    TD3_K = ( y2 - y1) / ( x2 - x1);

    bCalibrated = 1;
  }
  else
    bCalibrated = 0;
}

void InitBaudRate115200() {
  /*
  //WORK PARAMETERS FOR PRECISE 115200
  COMDIV0 = 0x0B;
  COMDIV1 = 0x00;
  COMDIV2 = 0x0029;
  */
}

void InitBaudRate256000() {
  //DL = 41.78 * 10^6 / 2 ^ (CD=0) * 1/16 * 1/ (2*Baudrate)
  //RESULT = (M+N/2048) = 41.78 * 10^6 / ( 2 ^ (CD=0) * 16 *  2 * Baudrate)
  // M = Floor ( RESULT)
  // N = (RESULT - M) * 2048

  
  //WORK PARAMETERS FOR PRECISE 256000
  /*COMDIV0 = 0x05;
  COMDIV1 = 0x00;
  COMDIV2 = 0x8829;
  */
}

void InitBaudRate512000() {
  //DL = 41.78e6 / 16/2/512000 = 2.xxxxxx
  //M+N/2048 = 41.78e6 / 16/2/2/512000 = 1.275xxxxxx
  //M=1
  //N=563=0x233
  //      8             A
  //  1 0 0   0     1   0 1 0
  //  0 0 1   1     0   0 1 1
  //     3             3
  //COMDIV2 = 0x8A33
  
  //COMDIV0 = 2;
  //COMDIV1 = 0x00;
  //COMDIV2 = 0x8A33;
}

void InitBaudRate921600() {
  /****************************************************************************************** */
  /*   921 600 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / /921600 / (2^(CD=0)=1) / 16 / 2 = 1,416693793
  //DL=1
  //M+N/2048 = 41.78e6 / /921600 / (2^(CD=0)=1) / 16/ (DL=1) / 2 = 1,416693793
  //M=1
  //N=853=0x355
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  1 0   1     0   1 0 1
  //     5             5
  //COMDIV2 = 0x8B55
  
  COMDIV0 = 1;      //������� ���� �������� (DL)
  COMDIV1 = 0x00;   //������� ���� �������� (DL)
  COMDIV2 = 0x8B55; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
  
  /****************************************************************************************** */
}

void InitBaudRate930000() {
  /****************************************************************************************** */
  /*   930 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 930 000 / (2^(CD=0)=1) / 16 / 2 = 1,404
  //DL=1
  //M+N/2048 = 41.78e6 / 930 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,404
  //M=1
  //N=827=0x33B
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  0 1   1     1   0 1 1
  //     3             B
  //COMDIV2 = 0x8B3B

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8B3B;
}

void InitBaudRate940000() {
  /****************************************************************************************** */
  /*   940 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 940 000 / (2^(CD=0)=1) / 16 / 2 = 1,389
  //DL=1
  //M+N/2048 = 41.78e6 / 940 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,389
  //M=1
  //N=797=0x31D
  //      8             B
  //  1 [0 0] [0     1]  0 1 1
  //  0  0 0   1     1   1 0 1
  //     1             D
  //COMDIV2 = 0x8B1D

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8B1D;
}

void InitBaudRate950000() {
  /****************************************************************************************** */
  /*   950 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 950 000 / (2^(CD=0)=1) / 16 / 2 = 1,374
  //DL=1
  //M+N/2048 = 41.78e6 / 950 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,374
  //M=1
  //N=766=0x2FE
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  1 1   1     1   1 1 0
  //     F             E
  //COMDIV2 = 0x8AFE

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8AFE;
}

void InitBaudRate960000() {
  /****************************************************************************************** */
  /*   960 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 960 000 / (2^(CD=0)=1) / 16 / 2 = 1,36
  //DL=1
  //M+N/2048 = 41.78e6 / 960 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,36
  //M=1
  //N=737=0x2E1
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  1 1   0     0   0 0 1
  //     E             1
  //COMDIV2 = 0x8AE1

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8AE1;
}

void InitBaudRate990000() {
  //DL = 41.78e6 / 990 000 / (2^(CD=0)=1) / 16 / 2 = 1,319
  //DL=1
  //M+N/2048 = 41.78e6 / 990 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,319
  //M=1
  //N=653=0x28D
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  1  0 0   0     1   1 0 1
  //     8             D
  //COMDIV2 = 0x8A8D

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8A8D; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
}

void InitBaudRate1000000() {
  /****************************************************************************************** */
  /*   1 000 000 ********/
  /****************************************************************************************** */
  //DL = 41.78e6 / 1 000 000 / (2^(CD=0)=1) / 16 / 2 = 1,305625
  //DL=1
  //M+N/2048 = 41.78e6 / 1 000 000 / (2^(CD=0)=1) / 16 / (DL=1) / 2 = 1,305625
  //M=1
  //N=626=0x272
  //      8             A
  //  1 [0 0] [0     1]  0 1 0
  //  0  1 1   1     0   0 1 0
  //     7             2
  //COMDIV2 = 0x8A72

  //COMDIV0 = 1;      //������� ���� �������� (DL)
  //COMDIV1 = 0x00;   //������� ���� �������� (DL)
  //COMDIV2 = 0x8A72; //16-��������� ������� �������� ��������
                    //15 - FBEN - ��� ���������� ������ ���������� � ������� ���������
                    //14-13 reserved
                    //12-11 M
                    //10-0  N
  
  /****************************************************************************************** */
}

void SimpleTest() {
  unsigned short sh1 = 500;
  long l1;
  signed int i1 = 1000;
  float dblResult;

  dblResult = (( float) i1) * (( float) sh1) / 65535.;

  dblResult = (( float) i1);
  dblResult *= (( float) sh1);
  dblResult /= 65535.;

  l1 = ( long ) sh1 * (long) i1;
  l1 = l1 >> 16;

  while(1);
}

void main() {
  //unsigned short ush_SA_check_time;
  //unsigned char jchar = 0x30; 
  int time = 20000;
  int i, k, val;
  char lb, hb;

  /*
  char *pntr;
  char b1, b2, b3, b4;
  */

  /*
  short q;
  unsigned short q1;
  unsigned int i1;
  */

  char bSAFake = 0;
  int prt2val;

  double db_dN1, db_dN2, dbU1, dbU2;
  float Coeff;

  double temp_t;

  //int n;
  int t2val_old;      //2014-08-27 - enabling external oscillator

  //�������� ������������ ������� ��������� ��������
  double dStartAmplAngCheck = 0.5;

  //���������� ����������� � �������� ���������� ��� ������ ���������
  int nRpcResetCounter = 0;
  int nRpcSumm = 0;

  gl_cCalibProcessState = 0;    //0 - no calibration
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
  //InitBaudRate115200();
  //InitBaudRate256000();
  //InitBaudRate512000();
  InitBaudRate921600();
  //InitBaudRate930000();
  //InitBaudRate940000();
  //InitBaudRate950000();
  //InitBaudRate960000();
  //InitBaudRate990000();
  //InitBaudRate1000000();


  COMCON0 = 0x007;      // Clearing DLAB

  //SimpleTest();

#ifdef DEBUG
  printf("DBG MODE\n");
  printf("T39-SLG. (C) SLC Alcor Laboratories, 2016.\n");
  printf("Software version: %d.%d.%d\n", VERSION_MAJOR, VERSION_MIDDLE, VERSION_MINOR);
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

  /*
  2015.09.24 ������ ������� �����
  GP2DAT |= 1 << (24 + 5);	//������������ ����� ������ ������ Data  (p2.5) � �������� ������
  GP2DAT |= 1 << (24 + 6);	//������������ ����� ������ ������ CLR   (p2.6) � �������� ������
  */

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
  T1LD = 0x08FFFFFF;
  T1CON = 0x0C4;
  //0x0C4 = 0000 1100 0100
  // 000 0   1 1 00   0100
  // |   |   | | |    |------ SourceClock/16
  // |   |   | | |----------- Binary
  // |   |   | -------------- Periodic
  // |   |   ---------------- Enable
  // |   -------------------- Count down
  // ------------------------ CoreClock (41.78 Mhz, � ���� ������ 32768*1275=41779200, � �� �������� ��������!)

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
  t2val_old = T2VAL;
  T2LD = 0x050;   // 5; 5 clocks @32.768kHz = 152ms
  T2CON = 0x480;  // Enable Timer2;

  //while ((T2VAL == t2val_old) || (T2VAL > 3)); // ensures timer is started...

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
  IRQCLR |= 0x10;   // RIGHT WAY disable TIMER2 Wakeup IRQ

/*
#ifdef DEBUG
  printf("PLLCON=%x. After change.\n", PLLCON);
#endif
*/

  //**********************************************************************
  // ������������ Timer2
  //**********************************************************************
  T2LD = 0x00FFFFFF;
  T2CON = 0x2C0;
  //0x2C0 = 0010 1100 0000
  // 0 01 0   1 1 00   0000
  //   |  |   | | |    |------ SourceClock/1
  //   |  |   | | |----------- Binary
  //   |  |   | -------------- Periodic
  //   |  |   ---------------- Enable
  //   |  -------------------- Count down
  //   ----------------------- External Crystal


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
  gl_chLockBit = gl_ush_flashLockDev;

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

  /*
  2015.09.24 ������ ������� �����
  // *********************************************************************
  // ������������ �������� ������
  // *********************************************************************
  SendPhaseShift();
  */

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

  dStartAmplAngCheck = ( double) gl_ush_flashParamAmplAngMin1 / 65535. * 3.;
  prt2val = T2VAL;
  ADCCP = 0x08;   //AmplAng channel = ADC8
  //pause(10);
  while( 1) {
    ADCCON |= 0x80;
    while (!( ADCSTA & 0x01)){}
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    if( ( ( double) gl_ssh_ampl_angle / 4095. * 3.) > dStartAmplAngCheck) {

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

  gl_ushFiringTry = 0;

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


    /*if( ( ( double) gl_ssh_current_1 / 4095. * 3. / 3.973 < ( double) gl_ush_flashParamI1min / 65535. * 0.75)  ||
        ( ( double) gl_ssh_current_2 / 4095. * 3. / 3.973 < ( double) gl_ush_flashParamI2min / 65535. * 0.75)) {*/

    #ifdef DEBUG
      printf("DBG: try %d\n", gl_ushFiringTry);
      printf("DBG: I1: Measured: %.02f  Goal: %.02f\n", ( 2.5 - ( double) gl_ssh_current_1 / 4095. * 3.) / 2.5, ( double) gl_ush_flashParamI1min / 65535. * 0.75);
      printf("DBG: I2: Measured: %.02f  Goal: %.02f\n", ( 2.5 - ( double) gl_ssh_current_2 / 4095. * 3.) / 2.5, ( double) gl_ush_flashParamI2min / 65535. * 0.75);
    #endif

    if( ( ( 2.5 - ( double) gl_ssh_current_1 / 4095. * 3.) / 2.5  < ( double) gl_ush_flashParamI1min / 65535. * 0.75)  ||
        ( ( 2.5 - ( double) gl_ssh_current_2 / 4095. * 3.) / 2.5 < ( double) gl_ush_flashParamI2min / 65535. * 0.75)) {
      //�� ��������

      if( gl_ushFiringTry < gl_ush_flashParamHvApplyCount * gl_ush_flashParamHvApplyPacks) {

        if( gl_ushFiringTry > 0 && ( gl_ushFiringTry % gl_ush_flashParamHvApplyCount) == 0) {

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
  printf( "DBG: Applying 3kV for %.2f sec\n", gl_ush_flashParamHvApplyDurat / 1000.);
#endif

        //�������� ��������� ������
        GP4DAT &= ~( 1 << (16 + 1));    //OFF3KV (p4.1) = 0
        GP4CLR = ( 1 << ( 16 + 1));     //�����

        //����� N ���
        pause( ( int) ( 32768. * ( double) gl_ush_flashParamHvApplyDurat / 1000.));

        //��������� ��������� ������
        GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
        GP4SET = 1 << (16 + 1);      //�����

#ifdef DEBUG
  printf( "DBG: 3kV is off\n");
#endif

        //����� 0.5 ���
        pause( 16384);

        //����������� ����� �������
        gl_ushFiringTry++;
      }
      else {

#ifdef DEBUG
  printf( "DBG: fireup FAILS\n");
#endif

        //��������� �������
        GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
        GP4SET = 1 << (16 + 0);      //�����

        deadloop_no_firing( ERROR_NO_LASER_FIRING);   //FAIL.FINISH
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

  //�������� ���� �������� ����������� ��������� (����� �������)
  gl_nActiveRegulationT2 = T2VAL;
  if( gl_nActiveRegulationT2 == 0) gl_nActiveRegulationT2 = 1;

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
  printf("VALUE=%.2f  ", gl_ush_flashParamDecCoeff / 65535.);
#endif

#ifdef DEBUG
  if( gl_b_SyncMode)
    printf("passed\n");
*/

  printf("DBG: Configuration has been passed. Main loop starts!\n");
#endif

/*
#ifdef DEBUG
  //*************************************************
  // ����������
  //*************************************************
  while(1) {
    while( (GP0DAT & 0x10) == 0);
    for( i=0; i<100; i++);
    while( (GP0DAT & 0x10) == 0x10);
    for( i=0; i<100; i++);
    while( (GP0DAT & 0x10) == 0);
    gl_n_prT1VAL = T1VAL;
    while( (GP0DAT & 0x10) == 0x10);
    for( i=0; i<100; i++);
    while( (GP0DAT & 0x10) == 0);
    gl_un_RULAControl = T1VAL;
    printf("0x%x\t0x%x\t0x%x\t", T2VAL, gl_n_prT1VAL, gl_un_RULAControl);
    gl_un_RULAControl = ( T1LD + gl_n_prT1VAL - gl_un_RULAControl) % T1LD;
    //gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    dStartAmplAngCheck = ( double) gl_un_RULAControl / 2611200.;
    printf("%d\t%.8f\t%.8f\n", gl_un_RULAControl, dStartAmplAngCheck, 1. / dStartAmplAngCheck);
    pause( 16384);
  }
  //*************************************************
#endif
*/

  /*
  while( 1) {
    printf( "T2CON=0x%04x\tT2VAL=0x%04x\t\n", T2CON, T2VAL);
    for( i=0; i<100000; i++);
  }
  */

  gl_nT2StartDataOut = T2VAL;

  //FAKE ('VERACITY DATA' flag on)
  gl_b_PerimeterReset = 1;

  //Starting packs.VERSION
  gl_nSentPackIndex = VERSION;
  send_pack( ( ( VERSION_MINOR * 16) << 8) + ( VERSION_MAJOR * 16 + VERSION_MIDDLE));

  //Starting packs.Device_Num
  gl_nSentPackIndex = DEVNUM;
  send_pack( gl_ush_flashParamDeviceId);

  //FAKE ('VERACITY DATA' flag off)
  gl_b_PerimeterReset = 0;

  gl_nSentPackIndex = UTD1;


  while( 1) {

    //**********************************************************************
    // ��������� ������ �������� ������
    //**********************************************************************
    processIncomingCommand();

    /*
    //�������� ����� - �������� ����� 10(5?) ��� � �������
    prt2val = T2VAL;
    while( (( 0x1000 + T2VAL - prt2val) % 0x1000) < 3276);
    bSAFake ^= 1;
    */

    //if( bSAFake) {	// �� ���� P0.4 ���� FAKE ������
    if( GP0DAT & 0x10) {  //�� ���� P0.4 ���� ������

      if( gl_b_SA_Processed == 0) { //���� � ���� SA ����� �� ��� ��� �� ������������
#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: processing tact\n");
  #endif
#endif

        //� �������� ����� ������ ������ �� ����� P0.0
        GP0DAT |= 1 << ( 16);       //�������� ����� p0.0 set
        /*for( i=0; i<100; i++);
        GP0DAT &= ~( 1 << ( 16));   //�������� ����� p0.0 clear*/

        //�������� ����� ������ ��������� �����
        gl_ssh_SA_time = ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
        //gl_ssh_SA_time+=3;   // <<<---- ���������� �����... � ��������� ������������ 24.01.2017: ����� ������ ������� - �������� ��������
                               // � ��������� ������������ 02.02.2017 ������� ��� ������ ��� ����� ������ �������� 32768Hz, ������� ��������� ������ ����
                               // 03.02.2017 �� ���� ������ ����� ��� �� ��� - ������� ������ ����� ��� ��� ����������� ����� PLL (1085), �� ��������� �� �������� ������ �� ����
        gl_n_prT1VAL = T1VAL;

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

#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: acquiring angle increment\n");
  #endif
#endif
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

          for( k=0; k<10; k++);                                      //������� t4
          //pause( 1);

          GP2SET |= 1 << (16 + 2);                                    //sclk->1
          val = 0;

          //������ 14 bit
          for( i=0; i<14; i++) {
            //GP2CLR |= 1 << (16 + 2);                                  //sclk->0
            GP2DAT &= ~( 1 << (16 + 2));                              //sclk->0

            for( k=0; k<5; k++);                                    //������� t5
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
#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: acquiring analogue param\n");
  #endif
#endif
        while (!( ADCSTA & 0x01)){}     // ������� ����� �������������� ��� (������������ ����� �� �������� ���� �� ��� ������ ���� �����)
        
        switch( ADCChannel) { //����������� ��� �� ������������ � ��������� � ��������������� ����������

          case 0:
            gl_ssh_current_2 = (ADCDAT >> 16);
          break;                       //ADC1 = I2

          case 1:
            gl_ssh_current_1 = (ADCDAT >> 16);
          break;                       //ADC2 = I1

          case 2:                                                                 //ADC3 = UINT (previous U_td3)
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

        if( gl_cCalibProcessState) {
          switch( gl_cCalibProcessState) {
            case 1:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 0) {
                gl_ush_flashParamT1_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 2;
              }
            break;

            case 2:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 1) {
                gl_ush_flashParamT1_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 3;
              }
            break;

            case 3:
              //��������� ������ ������ �� ����������� �����������
              if( ADCChannel == 2) {
                gl_ush_flashParamT1_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentPackIndex = CALIB_T1;
              }
            break;

            case 4:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 0) {
                gl_ush_flashParamT2_TD1_val = gl_ssh_Utd1_cal;
                gl_cCalibProcessState = 5;
              }
            break;

            case 5:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 1) {
                gl_ush_flashParamT2_TD2_val = gl_ssh_Utd2_cal;
                gl_cCalibProcessState = 6;
              }
            break;

            case 6:
              //��������� ������ ������ �� ������������ �����������
              if( ADCChannel == 2) {
                gl_ush_flashParamT2_TD3_val = gl_ssh_Utd3_cal;
                gl_cCalibProcessState = 0;
                save_params_p4();
                gl_nSentPackIndex = CALIB_T1;
              }
            break;
          }

          if( !gl_cCalibProcessState)
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

#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: data output\n");
  #endif
#endif

         // ************************************************************************************
        // 2018-03-15
        // �������� �� ���������
        // ************************************************************************************
        if( gl_bManualLaserOff == 0 && gl_nSentPackIndex == AMPLANG_ALTERA) {
          //���� �� �� ��������� ����� �������

          //����������� 5 ��� � ���������� ������� �������� �����
          if( gl_nVibrationWatchT2 != 0) {
            if( ( gl_nVibrationWatchT2 + T2LD - T2VAL) >= 163840) { //5sec (*32kHz = 163840)
              //5��� - �������� �� ����. ���������� ����.
              gl_nVibrationWatchT2 = 0;
              gl_nVibrationWatchCnt = 0;
            }
          }

          if( gl_ush_MeanImpulses < 100) {
            //�������� �������� ����� �������� ����� ����
            gl_nVibrationWatchT2 = T2VAL; //���������� ����� (�� ���������� 5 ��� �� ���� �� ������� ������� ������)
            gl_nVibrationWatchCnt++;      //����������� ������� ������

            if( gl_nVibrationWatchCnt >= 20) {
              //�� ����� ��� �� ��������� 5 ���, �� ������� 20 ��� ����� ���������� ����� �������� ���������- ��������� ���

              GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
              GP4SET = 1 << (16 + 0);      //�����
              deadloop_no_tact( ERROR_VIBRATION_LOST);
            }
          }
        }

        //**********************************************************************
        // ������ ������ �������� ���������
        //**********************************************************************
        if( gl_bOutData == 1) {
          switch( gl_nSentPackIndex) {
            //****************************************************************************************************************************************************************
            //REGULAR PACK
            //****************************************************************************************************************************************************************
            //case UTD1:            send_pack( ( short) gl_snMeaningCounterRound);           gl_nSentPackIndex = UTD2;           break; //UTD1
            case UTD1:            send_pack( gl_ssh_Utd1);           gl_nSentPackIndex = UTD2;           break; //UTD1

            //case UTD2:            send_pack( ( short) gl_lnMeanImps);           gl_nSentPackIndex = UTD3;           break; //UTD2
            case UTD2:            send_pack( gl_ssh_Utd2);           gl_nSentPackIndex = UTD3;           break; //UTD2

            //case UTD3:            send_pack( gl_un_RULAControl);           gl_nSentPackIndex = I1;             break; //UTD3
            case UTD3:            send_pack( gl_ssh_Utd3);           gl_nSentPackIndex = I1;             break; //UTD3


            case I1:              send_pack( gl_ssh_current_1);      gl_nSentPackIndex = I2;             break; //I1
            case I2:              send_pack( gl_ssh_current_2);      gl_nSentPackIndex = CNTRPC;         break; //I2
            case CNTRPC:          send_pack( gl_ssh_Perim_Voltage);  gl_nSentPackIndex = AMPLANG_ALTERA; break; //CntrPc
            case AMPLANG_ALTERA:  send_pack( gl_ush_MeanImpulses);   gl_nSentPackIndex = UTD1;           break; //AmplAng �� �������
            //case AMPLANG_DUS:    send_pack( gl_ssh_ampl_angle);     gl_nSentPackIndex = UTD1;           break; //AmplAng � ����
            //case RULA:          send_pack( gl_un_RULAControl);     gl_nSentPackIndex = UTD3;           break; //RULA

            //****************************************************************************************************************************************************************
            // PARAMETERS BY REQUEST
            //****************************************************************************************************************************************************************

            case AMPLITUDE:           send_pack( gl_ush_flashParamAmplitudeCode);   gl_nSentPackIndex = UTD1;       break;  //������� ��������� ���������
            case TACT_CODE:           send_pack( gl_ush_flashParamTactCode);        gl_nSentPackIndex = UTD1;       break;  //������� ���� ����� ���������
            case M_COEFF:             send_pack( gl_ush_flashParamMCoeff);          gl_nSentPackIndex = UTD1;       break;  //������� ������������ ���������
            case STARTMODE:           send_pack( gl_ush_flashParamStartMode);       gl_nSentPackIndex = UTD1;       break;  //������� ��������� ����
            case DECCOEFF_CURRENT:    send_pack( gl_ush_flashParamDecCoeff);        gl_nSentPackIndex = UTD1;       break;  //����������� ������
            case CONTROL_I1:          send_pack( gl_ush_flashParamI1min);           gl_nSentPackIndex = CONTROL_I2; break;  //flashParamI1min
            case CONTROL_I2:          send_pack( gl_ush_flashParamI2min);           gl_nSentPackIndex = CONTROL_AA; break;  //flashParamI2min
            case CONTROL_AA:          send_pack( gl_ush_flashParamAmplAngMin1);     gl_nSentPackIndex = UTD1;       break;  //flashParamAmplAngMin1

            case HV_APPLY_COUNT_SET:  send_pack( gl_ush_flashParamHvApplyCount);    gl_nSentPackIndex = UTD1;       break;  //HV apply cycles in pack
            case HV_APPLY_COUNT_TR:   send_pack( gl_ushFiringTry);                  gl_nSentPackIndex = UTD1;       break;  //HV apply cycles applied in this run
            case HV_APPLY_DURAT_SET:  send_pack( gl_ush_flashParamHvApplyDurat);    gl_nSentPackIndex = UTD1;       break;  //HV apply cycle duration
            case HV_APPLY_PACKS:      send_pack( gl_ush_flashParamHvApplyPacks);    gl_nSentPackIndex = UTD1;       break;  //HV apply packs

            case SIGNCOEFF:           send_pack( gl_ush_flashParamSignCoeff);       gl_nSentPackIndex = UTD1;       break;  //������� ��������� ������������
            case DEVNUM:              send_pack( gl_ush_flashParamDeviceId);        gl_nSentPackIndex = UTD1;       break;  //Device_Num

            case DATE_Y:              send_pack( gl_ush_flashParamDateYear);        gl_nSentPackIndex = UTD1;       break;  //Date.Year
            case DATE_M:              send_pack( gl_ush_flashParamDateMonth);       gl_nSentPackIndex = UTD1;       break;  //Date.Month
            case DATE_D:              send_pack( gl_ush_flashParamDateDay);         gl_nSentPackIndex = UTD1;       break;  //Date.Day

            case ORG_B1:              send_pack( gl_ac_flashParamOrg[ 0]);          gl_nSentPackIndex = ORG_B2;     break;  //Organization.Byte1
            case ORG_B2:              send_pack( gl_ac_flashParamOrg[ 1]);          gl_nSentPackIndex = ORG_B3;     break;  //Organization.Byte2
            case ORG_B3:              send_pack( gl_ac_flashParamOrg[ 2]);          gl_nSentPackIndex = ORG_B4;     break;  //Organization.Byte3
            case ORG_B4:              send_pack( gl_ac_flashParamOrg[ 3]);          gl_nSentPackIndex = ORG_B5;     break;  //Organization.Byte4
            case ORG_B5:              send_pack( gl_ac_flashParamOrg[ 4]);          gl_nSentPackIndex = ORG_B6;     break;  //Organization.Byte5
            case ORG_B6:              send_pack( gl_ac_flashParamOrg[ 5]);          gl_nSentPackIndex = ORG_B7;     break;  //Organization.Byte6
            case ORG_B7:              send_pack( gl_ac_flashParamOrg[ 6]);          gl_nSentPackIndex = ORG_B8;     break;  //Organization.Byte7
            case ORG_B8:              send_pack( gl_ac_flashParamOrg[ 7]);          gl_nSentPackIndex = ORG_B9;     break;  //Organization.Byte8
            case ORG_B9:              send_pack( gl_ac_flashParamOrg[ 8]);          gl_nSentPackIndex = ORG_B10;    break;  //Organization.Byte9
            case ORG_B10:             send_pack( gl_ac_flashParamOrg[ 9]);          gl_nSentPackIndex = ORG_B11;    break;  //Organization.Byte10
            case ORG_B11:             send_pack( gl_ac_flashParamOrg[10]);          gl_nSentPackIndex = ORG_B12;    break;  //Organization.Byte11
            case ORG_B12:             send_pack( gl_ac_flashParamOrg[11]);          gl_nSentPackIndex = ORG_B13;    break;  //Organization.Byte12
            case ORG_B13:             send_pack( gl_ac_flashParamOrg[12]);          gl_nSentPackIndex = ORG_B14;    break;  //Organization.Byte13
            case ORG_B14:             send_pack( gl_ac_flashParamOrg[13]);          gl_nSentPackIndex = ORG_B15;    break;  //Organization.Byte14
            case ORG_B15:             send_pack( gl_ac_flashParamOrg[14]);          gl_nSentPackIndex = ORG_B16;    break;  //Organization.Byte15
            case ORG_B16:             send_pack( gl_ac_flashParamOrg[15]);          gl_nSentPackIndex = UTD1;       break;  //Organization.Byte16    ��� ������������ 0 �� �����!!!!!

            case VERSION:             send_pack( ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE)); gl_nSentPackIndex = UTD1; break; //SOFTWARE VERSION

            case CALIB_T1:            send_pack( gl_ush_flashParam_calibT1);        gl_nSentPackIndex = T1_TD1;     break;  //min thermo-calib point T
            case T1_TD1:              send_pack( gl_ush_flashParamT1_TD1_val);      gl_nSentPackIndex = T1_TD2;     break;  //min thermo-calib point thermo1 data
            case T1_TD2:              send_pack( gl_ush_flashParamT1_TD2_val);      gl_nSentPackIndex = T1_TD3;     break;  //min thermo-calib point thermo2 data
            case T1_TD3:              send_pack( gl_ush_flashParamT1_TD3_val);      gl_nSentPackIndex = CALIB_T2;   break;  //min thermo-calib point thermo3 data
            case CALIB_T2:            send_pack( gl_ush_flashParam_calibT2);        gl_nSentPackIndex = T2_TD1;     break;  //max thermo-calib point T
            case T2_TD1:              send_pack( gl_ush_flashParamT2_TD1_val);      gl_nSentPackIndex = T2_TD2;     break;  //max thermo-calib point thermo1 data
            case T2_TD2:              send_pack( gl_ush_flashParamT2_TD2_val);      gl_nSentPackIndex = T2_TD3;     break;  //max thermo-calib point thermo2 data
            case T2_TD3:              send_pack( gl_ush_flashParamT2_TD3_val);      gl_nSentPackIndex = UTD1;       break;  //max thermo-calib point thermo3 data

          }
        }

#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: decrement coefficient recalculation\n");
  #endif
#endif
        //������� �������������� ������������ ������
        if( gl_b_SyncMode) {
          db_dN1 = ( double) gl_ssh_angle_inc_prev;
          db_dN2 = ( double) gl_ssh_angle_inc;
          dbU1 = ( double) gl_ssh_angle_hanger_prev;
          dbU2 = ( double) gl_ssh_angle_hanger;
          Coeff = ( ( float) gl_ush_flashParamDecCoeff) / 65535.;
          gl_dbl_Omega =  ( db_dN2 - db_dN1) - ( dbU2 - dbU1) * Coeff * ( ( signed short) gl_ush_flashParamSignCoeff - 1);
          if( fabs( gl_dbl_Omega) < 5) {
            gl_dbl_Nsumm += fabs( ( double) gl_ssh_angle_inc - ( double) gl_ssh_angle_inc_prev);
            gl_dbl_Usumm += fabs( ( double) gl_ssh_angle_hanger - ( double) gl_ssh_angle_hanger_prev);
            gl_un_DecCoeffStatPoints++;
            if( !( gl_un_DecCoeffStatPoints % DEC_COEFF_CONTINUOUS_CALCULATION_N)) {
              gl_ush_flashParamDecCoeff = ( short) ( gl_dbl_Nsumm / gl_dbl_Usumm * 65535.);
              gl_dbl_Nsumm = gl_dbl_Usumm = 0.;
              gl_un_DecCoeffStatPoints = 0;
              gl_nSentPackIndex = DECCOEFF_CURRENT;
            }
          }
        }

        gl_ssh_angle_inc_prev = gl_ssh_angle_inc;

        // ************************************************************************************
        // 2016-12-14
        // �������� �� ���������� ������
        // ************************************************************************************
        if( gl_bManualLaserOff == 0 && gl_nSentPackIndex == CNTRPC) {
          //���� �� �� ��������� �������� � ������ ��� ������ ������ ��� i2 (� ��� ������ i1 � i2

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
              deadloop_current_unstable( ERROR_CURRENT_UNSTABLE);
            }
          }
        }


#ifdef DEBUG
  #if DEBUG == 2
    printf( "DBG: perimeter adjustment\n");
  #endif
#endif

        //��������� ������ ������ ����� 1.8 ������
        if( gl_nT2StartDataOut) {
          if( ( T2LD + gl_nT2StartDataOut - T2VAL) % T2LD >= 32768 * 1.8) {
            gl_bOutData = 1;
            gl_nSentPackIndex= UTD1;
            gl_nT2StartDataOut = 0;
          }
        }

        // ************************************************************************************
        // 2010-04-22
        // 2017-02-05 - ���� �� ���������� � ��������� ������
        //�������������� ����������� ���������
        // ************************************************************************************
        if( gl_bOutData == 1) {
          if( gl_nSentPackIndex == AMPLANG_ALTERA || gl_nSentPackIndex == AMPLANG_DUS) {
            //V_piezo = ( ( gl_ssh_Perim_Voltage / 4095. * 3.) - 2.048) * 100.;

            //1564 --> -90.2212
            //1565 --> -90.1480
            //1566 --> -90.0747
            //1567 --> -90.0015
            //1568 --> -89.9282

            //4024 --> +89.99853
            //4025 --> +90.07179
            //4026 --> +90.14505

            nRpcResetCounter = (++nRpcResetCounter) % 16;
            nRpcSumm += gl_ssh_Perim_Voltage;
            if( nRpcResetCounter == 0) {
              nRpcSumm = nRpcSumm >> 4;

              if( nRpcSumm < 1565 || nRpcSumm > 4025) {
              
                gl_ush_flashParamStartMode = 125;
                DACConfiguration();
                GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1
                gl_nRppTimer = T2VAL;
                gl_b_PerimeterReset = 1;
              }

              nRpcSumm = 0;
            }
          }
        }

#ifdef DEBUG
  #if DEBUG == 2
          printf( "DBG: gl_ush_MeanImpulses=%d\n", gl_ush_MeanImpulses);
          printf( "DBG: gl_snMeaningCounter=%d gl_lnMeaningSumm=%d\n", gl_snMeaningCounter, gl_lnMeaningSumm);
          printf( "DBG: gl_lnMeanImps=%ld gl_snMeaningShift=%d\n", gl_lnMeanImps, gl_snMeaningShift);
  #endif
#endif

        //**********************************************************************
        //������������ ������� ��������� ��������� ��������� (��������� ����� ��������� ������� ���� ����� ��������� ������)
        //**********************************************************************
        gl_snMeaningCounter = ( ++gl_snMeaningCounter) % gl_snMeaningCounterRound;

        //���������� ���� ���������� ���������� RULA
        if( gl_snMeaningCounter == 0) {

          //�� �������... ����� �������� ��������� � ���� ����� ���������
          gl_lnMeanImps = gl_lnMeaningSumm >> ( gl_snMeaningShift - 4);
          gl_lnMeanImps = gl_lnMeanImps >> 2;

          if( gl_lnMeanImps > 90) {  //90 = 16 * 5
            //�� ����.... ��������� � ������ �� �������� 4095=2,5�, (2016.02.05 14:15, ��������: 2.5 ��� 3?)
            //����� ������� 2,2�=120"
            //� �������� �� ���������� ����������� 2,9 �� �������� ����� ���������
            //gl_dMeanImps = gl_dMeaningSumm / ( double) gl_snMeaningCounterRound / 4095. * 2.5 / 2.2 * 120. / 2.9;

            if( abs( gl_lnMeanImps - ( gl_ush_flashParamAmplitudeCode << 4)) > 1) {    //�� ���� ��������� �� ������� ���� ������� �� ������ 1/16 �� ��������
              if( gl_lnMeanImps > ( gl_ush_flashParamAmplitudeCode << 4)) {

                //gl_un_RULAControl -= nDelta;

                if( nDelta >= gl_un_RULAControl) {
                  //delta = cRULAControl;
                  gl_un_RULAControl = gl_un_RULAControl / 2;
                }
                else
                  gl_un_RULAControl -= nDelta;

              }
              if( gl_lnMeanImps < ( gl_ush_flashParamAmplitudeCode << 4)) {

                //gl_un_RULAControl += nDelta;

                if( gl_un_RULAControl + nDelta > RULA_MAX) {
                  //delta = 255 - cRULAControl;
                  gl_un_RULAControl = ( RULA_MAX + gl_un_RULAControl) / 2;
                }
                else
                  gl_un_RULAControl += nDelta;

              }
            }

            if( gl_un_RULAControl > RULA_MAX) gl_un_RULAControl = RULA_MAX;
            if( gl_un_RULAControl < RULA_MIN) gl_un_RULAControl = RULA_MIN;

            //���������� ��������� "��������" (���� ��� ��� > 1)
            nDelta = nDelta >> 1;
            if( nDelta < 1) {
              nDelta = 1;
            }

            /*
            //��������� "�����������" ���������� ����� 7 ������
            if( gl_nT2StartDataOut) {
              if( ( T2LD + gl_nT2StartDataOut - T2VAL) % T2LD >= 32768 * 7) {
                /*
                nDelta = ( RULA_MAX - RULA_MIN) / 4;
                gl_snMeaningCounterRound = MEANING_IMP_PERIOD_100;
                */
                /*
                gl_nT2StartDataOut = 0;
              }
            }
            */

            

            //������� ����� (����� ���������� ����������) ��� �� �������������� ��������
            //2014.10.09 - ������� ��� ���� ������� ������� - ��������� � �����������
            if( nDelta == 1) {
              if( gl_nActiveRegulationT2 != 0) {
                //�������� ����������� ���������
                if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 160)     { nDelta = 50; gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 90) { nDelta = 25; gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 16) { nDelta = 12; gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 14) { nDelta = 6;  gl_snMeaningCounterRound = 128;  gl_snMeaningShift = 7; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 13) { nDelta = 3;  gl_snMeaningCounterRound = 256;  gl_snMeaningShift = 8; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 11) {              gl_snMeaningCounterRound = 256;  gl_snMeaningShift = 8; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 10) {              gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) >  8) {              gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else                                                                 {              gl_snMeaningCounterRound = 1024; gl_snMeaningShift = 10; }
              }
              else {
                //����������� ��������� � �������� ������ �������
                if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 160)     { nDelta = 8; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 90) { nDelta = 4; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 16) { nDelta = 2; gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 14) {             gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 13) {             gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 11) {             gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) > 10) {             gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else if( abs( ( gl_ush_flashParamAmplitudeCode << 4) - gl_lnMeanImps) >  8) {             gl_snMeaningCounterRound = 512;  gl_snMeaningShift = 9; }
                else                                                                 {             gl_snMeaningCounterRound = 1024; gl_snMeaningShift = 10; }
              }
            }


            DACConfiguration();


            /*
            //"�����������" ���������� (��������� ������� �������� ������)
            if( gl_snMeaningCounterRound == MEANING_IMP_PERIOD_STABLE) {
              if( abs( gl_ush_flashParamAmplitudeCode - gl_dMeanImps) > 5) {
                nDelta = ( RULA_MAX - RULA_MIN) / 4;
                gl_snMeaningCounterRound = MEANING_IMP_PERIOD_100;
              }
            }*/

          }

          gl_lnMeaningSumm = 0;

        }
        else {
          gl_lnMeaningSumm += gl_ush_MeanImpulses;    //�� �������
          //gl_dMeaningSumm += gl_ssh_ampl_angle;    //�� ����
        }

        //**********************************************************************
        //��������� ������ ����� �������� ����������� ���������
        //**********************************************************************
        if( gl_nActiveRegulationT2 != 0) {
          if( (( T2LD + gl_nActiveRegulationT2 - T2VAL) % T2LD) > 32768. * 10.0) {    //������������ ���� �������� ����������� ��������� 10 ���
            gl_nActiveRegulationT2 = 0;
          }
        }

        //**********************************************************************
        //��������� ����� ������ RP_P
        //**********************************************************************
        if( gl_nRppTimer != 0) {
          if( (( T2LD + gl_nRppTimer - T2VAL) % T2LD) > 32768. * 0.5) {    //������������ RESET� ��� (0.5s + 0.5s)
            if( gl_b_PerimeterReset == 1) {
              //������ ������ ����� (0.5 ��� ����� ���������� �����������) ==> �������� ����������, � �������� ��� 0.5sec
              //��������� ����������� � ������� ����������� ���������
              GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0

              //�������� �������� ������
              gl_nRppTimer = T1VAL;
              gl_b_PerimeterReset = 2;
            }
            else if( gl_b_PerimeterReset == 2) {
              //������ ������ ����� (0.5 ��� ����� ��������� �����������) ==> ������� ��� ���������� ����������, � ������ ����������, � ������ ��� ������ �� �����
              //���������� ���� ������������� ������ � �������� "����������"
              gl_b_PerimeterReset = 0;

              //������ �������� ������ �� ����
              gl_nRppTimer = 0;
              
            }
            else {
              //����� �������� ��������, �� ������� ������� �� ��� ����� �� ����������
              //���������� ���� ������������� ������ � �������� "����������"
              gl_b_PerimeterReset = 0;

              //������ �������� ������ �� ����
              gl_nRppTimer = 0;
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
      ush_SA_check_time = ( T1LD + gl_ssh_prT1VAL - T1VAL) % T1LD;

      //2 sec = 32768 * 2.0 = 65536
      if( ush_SA_check_time > 65536) {
        //������� ������������

        //��������� �������
        GP4DAT |= ( 1 << (16 + 0));   //ONHV   (p4.0) = 1

        deadloop_no_tact( ERROR_TACT_SIGNAL_LOST);

      }*/
    }
  }
}