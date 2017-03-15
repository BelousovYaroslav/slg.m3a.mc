#include <ADuC7026.h>
#include <stdio.h>
#include "AnalogueParamsConstList.h"
#include "settings.h"
#include "McCommands.h"

#include "debug.h"

//declared in Main.c
extern char pos_in_in_buf;
extern char input_buffer[];

extern char gl_c_EmergencyCode;         //Device error code

extern int gl_sn_MeaningCounter;        //Amplitude control module: counter of measured values
extern int gl_sn_MeaningCounterRound;   //Amplitude control module: round of measured values

extern short gl_nSentPackIndex;         //Analogue Parameter Index (what are we sending now)

extern char gl_chAngleOutput;           //���� ������ ���������� ����: 0 = dW (4b)         1 = dN (2b), dU(2b)
extern char gl_b_SyncMode;              //���� ������ ������ ���������:   0=�����. 1=������.
extern char gl_bManualLaserOff;         //���� ��� �� ���� ��������� ��� ������ (��������). ���� ����� ����� ��������� ��� ������� � ������������� �������� ����.

extern char cCalibProcessState;
extern char bCalibrated;


extern unsigned int gl_un_RULAControl;
extern unsigned int nDelta;

extern int gl_nRppTimer;

extern char gl_b_PerimeterReset;

extern double gl_dMeaningSumm;
extern char gl_bAsyncDu;

extern double gl_dMeanImps;

//page 1
extern unsigned short flashParamAmplitudeCode;     //Hanger Vibration Amplitude set
extern unsigned short flashParamTactCode;          //Tact Code set
extern unsigned short flashParamMCoeff;            //Noise Coefficient set
extern unsigned short flashParamStartMode;         //Start mode set
extern unsigned short flashParamDecCoeff;          //����������� ������
extern unsigned short flashLockDev;                //���� ���������� ����������

//page 2
extern unsigned short flashParamI1min;             //����������� �������� ���� ������� I1
extern unsigned short flashParamI2min;             //����������� �������� ���� ������� I2
extern unsigned short flashParamAmplAngMin1;       //����������� �������� �������� � ����
extern unsigned short flashParamHvApplyCount;      //HV applies tries count
extern unsigned short flashParamHvApplyDurat;      //HV applies tries duration
extern unsigned short flashParamHvApplyPacks;      //HV applies packs

//page 3
extern unsigned short flashParamSignCoeff;         //�������� �����������
extern unsigned short flashParamDeviceId;          //ID ����������
extern unsigned short flashParamDateYear;          //���� �������.���
extern unsigned short flashParamDateMonth;         //���� �������.�����
extern unsigned short flashParamDateDay;           //���� �������.����
extern char flashParamOrg[17];                     //�������� �����������

//page 4. ���������� �������������
extern signed short flashParam_calibT1;
extern unsigned short flashParamT1_TD1_val, flashParamT1_TD2_val, flashParamT1_TD3_val;
extern signed short flashParam_calibT2;
extern unsigned short flashParamT2_TD1_val, flashParamT2_TD2_val, flashParamT2_TD3_val;

extern char gl_chLockBit;         //���� ������������ ����������: 0 - ����� "���������"   1 - ����� "������������"

//DOUBLE DEFINITION!!!!!!!!!!! Original is declared in Main.c
#define IN_COMMAND_BUF_LEN 4                //����� ������ �������� ������

extern void configure_hanger( void);
extern void DACConfiguration( void);

void processIncomingCommand( void) {
  short in_param_temp;

  if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {

#ifdef DEBUG
  printf("Incoming command: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
              input_buffer[ 0], input_buffer[ 1], input_buffer[ 2],
              input_buffer[ 3], input_buffer[ 4], input_buffer[ 5]);

#endif
    //LOCK �������
    if( gl_chLockBit == 1) {
      switch( input_buffer[ 0]) {
        case MC_COMMAND_ACT_UNLOCK_DEVICE:
          if( input_buffer[ 1] == 0x5A &&
              input_buffer[ 2] == 0x55 &&
              input_buffer[ 3] == 0x5A) {
                gl_chLockBit = 0;
                flashLockDev = 0;
          }
        break;
        case MC_COMMAND_REQ:
          switch( input_buffer[ 1]) {
            case VERSION:     gl_nSentPackIndex = VERSION;      break;
          }
        break;
        default:
#ifdef DEBUG
          printf("DBG: Device is locked!\n");
#endif
        break;
      }
      pos_in_in_buf = 0;
      return;
    }

    switch( input_buffer[0]) {

      case MC_COMMAND_SET:
        switch( input_buffer[1]) {
          case AMPLITUDE:   //Set Amplitude of Hangreup Vibration
            flashParamAmplitudeCode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = AMPLITUDE;

            /*
            cRULAControl = ( RULA_MAX - RULA_MIN) / 2;
            delta = ( RULA_MAX - RULA_MIN) / 4;
            */

            /*
            gl_un_RULAControl = 64;
            nDelta = ( RULA_MAX - RULA_MIN) / 2;
            */

            gl_sn_MeaningCounter = 0;
            gl_sn_MeaningCounterRound = MEANING_IMP_PERIOD_100;
            gl_dMeaningSumm = 0.;
            gl_dMeanImps = 0.;
          break;

          case TACT_CODE:   //Set CodeTact
            flashParamTactCode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            configure_hanger();
            gl_nSentPackIndex = TACT_CODE;
          break;

          case M_COEFF: //Set NoiseCoefficient M
            flashParamMCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            DACConfiguration();
            gl_nSentPackIndex = M_COEFF;
          break;

          case STARTMODE: //Set StartMode
            flashParamStartMode = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            DACConfiguration();
            GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1

            gl_nRppTimer = T2VAL;
            gl_b_PerimeterReset = 1;

            gl_nSentPackIndex = STARTMODE;
          break;

          case DECCOEFF: //Set decrement coeff
            flashParamDecCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DECCOEFF;
          break;

          case CONTROL_I1:  //Set control_i1
            flashParamI1min = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
/*#ifdef DEBUG
        printf("DBG: Input command (0x04 - SetControlI1) accepted. Param: 0x%04x\n", flashParamI1min);
#endif*/
            gl_nSentPackIndex = CONTROL_I1;
          break;

          case CONTROL_I2:  //Set control_i2
            flashParamI2min = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = CONTROL_I2;
          break;

          case CONTROL_AA:  //Set control_aa
            flashParamAmplAngMin1 = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = CONTROL_AA;
          break;

          case HV_APPLY_COUNT_SET:  //Set HV_applies_count
            flashParamHvApplyCount = input_buffer[2];
            gl_nSentPackIndex = HV_APPLY_COUNT_SET;
          break;

          case HV_APPLY_DURAT_SET:  //Set HV_applies_duration
            flashParamHvApplyDurat = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = HV_APPLY_DURAT_SET;
          break;

          case HV_APPLY_PACKS:   //Set HV_applies_packs
            flashParamHvApplyPacks = input_buffer[2];
            gl_nSentPackIndex = HV_APPLY_PACKS;
          break;

          case SIGNCOEFF:  //Set sign coeff
            flashParamSignCoeff = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = SIGNCOEFF;
          break;

          case DEVNUM:    //Set device
            flashParamDeviceId = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DEVNUM;
          break;

          /*
          case DEVNUM_BH:  //Set device id high byte
            flashParamDeviceId &= ( 0xFF00);
            flashParamDeviceId &= ( ( ( short) input_buffer[2]) << 8);
            gl_nSentPackIndex = DEVNUM_BH;
          break;
          */

          case DATE_Y:    //Set Date.YEAR
            flashParamDateYear = input_buffer[2] + ( ( ( short) input_buffer[3]) << 8);
            gl_nSentPackIndex = DATE_Y;
          break;

          case DATE_M:    //Set Date.MONTH
            flashParamDateMonth = input_buffer[2];
            gl_nSentPackIndex = DATE_Y;
          break;

          case DATE_D:    //Set Date.DAY
            flashParamDateDay = input_buffer[2];
            gl_nSentPackIndex = DATE_Y;
          break;

          case ORG_B1:    //���������� Organization.byte1
            flashParamOrg[0] = input_buffer[2];  break;
          case ORG_B2:    //���������� Organization.byte2
            flashParamOrg[1] = input_buffer[2];  break;
          case ORG_B3:    //���������� Organization.byte3
            flashParamOrg[2] = input_buffer[2];  break;
          case ORG_B4:    //���������� Organization.byte4
            flashParamOrg[3] = input_buffer[2];  break;
          case ORG_B5:    //���������� Organization.byte5
            flashParamOrg[4] = input_buffer[2];  break;
          case ORG_B6:    //���������� Organization.byte6
            flashParamOrg[5] = input_buffer[2];  break;
          case ORG_B7:    //���������� Organization.byte7
            flashParamOrg[6] = input_buffer[2];  break;
          case ORG_B8:    //���������� Organization.byte8
            flashParamOrg[7] = input_buffer[2];  break;
          case ORG_B9:    //���������� Organization.byte9
            flashParamOrg[8] = input_buffer[2];  break;
          case ORG_B10:   //���������� Organization.byte10
            flashParamOrg[9] = input_buffer[2];  break;
          case ORG_B11:   //���������� Organization.byte11
            flashParamOrg[10] = input_buffer[2]; break;
          case ORG_B12:   //���������� Organization.byte12
            flashParamOrg[11] = input_buffer[2]; break;
          case ORG_B13:   //���������� Organization.byte13
            flashParamOrg[12] = input_buffer[2]; break;
          case ORG_B14:   //���������� Organization.byte14
            flashParamOrg[13] = input_buffer[2]; break;
          case ORG_B15:   //���������� Organization.byte15
            flashParamOrg[14] = input_buffer[2]; break;
          case ORG_B16:   //���������� Organization.byte16
            flashParamOrg[15] = input_buffer[2]; break;
        }
      break;

      case MC_COMMAND_REQ:
        switch( input_buffer[1]) {
          case AMPLITUDE:   gl_nSentPackIndex = AMPLITUDE;    break;
          case TACT_CODE:   gl_nSentPackIndex = TACT_CODE;    break;
          case M_COEFF:     gl_nSentPackIndex = M_COEFF;      break;
          case STARTMODE:   gl_nSentPackIndex = STARTMODE;    break;
          case DECCOEFF:    gl_nSentPackIndex = DECCOEFF;     break;
          case CONTROL_I1:  gl_nSentPackIndex = CONTROL_I1;   break;
          case CONTROL_I2:  gl_nSentPackIndex = CONTROL_I2;   break;
          case CONTROL_AA:  gl_nSentPackIndex = CONTROL_AA;   break;

          case HV_APPLY_COUNT_SET:  gl_nSentPackIndex = HV_APPLY_COUNT_SET;   break;
          case HV_APPLY_COUNT_TR:   gl_nSentPackIndex = HV_APPLY_COUNT_TR;    break;
          case HV_APPLY_DURAT_SET:  gl_nSentPackIndex = HV_APPLY_DURAT_SET;   break;
          case HV_APPLY_PACKS:      gl_nSentPackIndex = HV_APPLY_PACKS;       break;

          case SIGNCOEFF:   gl_nSentPackIndex = SIGNCOEFF;    break;
          case DEVNUM:      gl_nSentPackIndex = DEVNUM;       break;

          case DATE_Y:      gl_nSentPackIndex = DATE_Y;       break;
          case DATE_M:      gl_nSentPackIndex = DATE_M;       break;
          case DATE_D:      gl_nSentPackIndex = DATE_D;       break;

          case ORG:         gl_nSentPackIndex = ORG_B1;       break;

          case VERSION:     gl_nSentPackIndex = VERSION;      break;
        }
      break;



      case MC_COMMAND_ACT_SWC_DW_DNDU_OUTPUT: //Within async mode switch DN,DU or dW output
        if( gl_b_SyncMode == 1) {
          if( input_buffer[1] == 0) gl_chAngleOutput = 0; //switch to dW output
          if( input_buffer[1] == 1) gl_chAngleOutput = 1; //switch to dNdU output
        }
      break;

      case MC_COMMAND_ACT_T_CALIBRATION: //Thermo calibration (parameter here is current temperature)
        bCalibrated = 0;
        in_param_temp  = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
        if( flashParam_calibT1 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION) && 
            flashParam_calibT1 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
            //� ��� ���� ���������� ����������� ����� ����������

            //������� �� ��, ����� �� ������ ��� ����� ������ ����
            if( in_param_temp == flashParam_calibT1) {
              gl_nSentPackIndex = CALIB_T1;
              break;
            }

            if( flashParam_calibT2 >= ( THERMO_CALIB_PARAMS_BASE + MIN_T_THERMO_CALIBRATION)  &&
              flashParam_calibT2 <= ( THERMO_CALIB_PARAMS_BASE + MAX_T_THERMO_CALIBRATION)) {
              //� ��� ���� ���������� ����������� � ������������ ����� ����������
              //��������� ����� ���� ��������
              if( in_param_temp < flashParam_calibT1) {
                //���� �������� �����������
                cCalibProcessState = 1;
                flashParam_calibT1 = in_param_temp;
              }
              else {
                //���� �������� ������������
                cCalibProcessState = 4;
                flashParam_calibT2 = in_param_temp;
              }
            }
            else {
              //� ��� ���� ���������� ����������� ����� ����������, �� ��� ���������� ������������
              cCalibProcessState = 3;
              flashParam_calibT2 = in_param_temp;
            }

        }
        else {
          //� ��� ��� ���� ����������� �����!! ������ ��� ����� ����������� :)
          flashParam_calibT1 = in_param_temp;
          cCalibProcessState = 1;
        }
      break;

      case MC_COMMAND_ACT_RESET_T_CALIB:    //Reset thermo calibration data
        bCalibrated = 0;
        flashParam_calibT1 = 0;
        flashParamT1_TD1_val = 0;
        flashParamT1_TD2_val = 0;
        flashParamT1_TD3_val = 0;

        flashParam_calibT2 = 0;
        flashParamT2_TD1_val = 1;
        flashParamT2_TD2_val = 1;
        flashParamT2_TD3_val = 0;

        save_params_p4();
        gl_nSentPackIndex = CALIB_T1;
      break;

      case MC_COMMAND_ACT_LASER_OFF:    //Laser turn off
        GP4DAT |= 1 << (16 + 0);      //ONHV       (p4.0) = 1
        GP4DAT |= 1 << (16 + 1);      //OFF3KV     (p4.1) = 1
        gl_bManualLaserOff = 1;
      break;

      case MC_COMMAND_ACT_INTEGR_OFF: //Integrator turn off
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)

        gl_b_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
        //nRppTimer = ;               //<-- ����� �� ��������! ���� �� ��������� ������� (������ �� ������� �������� ����������)
      break;

      case MC_COMMAND_ACT_INTEGR_ON:  //Integrator turn on
        GP0DAT &= ~( 1 << (16 + 5));  //RP_P   (p0.5) = 0 (���������)

        gl_b_PerimeterReset = 2;      //������������� ���� ��������������� ������ (2=������ ���������, ������ ������������)
        gl_nRppTimer = T2VAL;         //<-- �������� �����! � ����� [����������� �����] ���� ����� ����
      break;

      case MC_COMMAND_ACT_INTEGR_RESET:    //Integrator reset
        GP0DAT |= ( 1 << (16 + 5));   //RP_P   (p0.5) = 1 (����������)

        gl_nRppTimer = T2VAL;         //<-- �������� �����! � ����� [����������� �����] ���������� ���������, ���� ������� � ��������� 2, � ��� ����� [����������� �����] ���� ������� � ��������� 0 (������ ����������)
        gl_b_PerimeterReset = 1;      //������������� ���� ��������������� ������ (1=������ ����������, ������ ������������)
      break;

      case MC_COMMAND_ACT_SAVE_FLASH_PARAMS:
        switch( input_buffer[1]) {
          case 0: save_params_p1(); break;
          case 1: save_params_p2(); break;
          case 2: save_params_p3(); break;
          case 3: save_params_p4(); break;
        }
      break;

      case MC_COMMAND_ACT_RELOAD_FLASH_PARAMS:
        switch( input_buffer[1]) {
          case 0: load_params_p1(); check_params_p1(); break;
          case 1: load_params_p2(); check_params_p2(); break;
          case 2: load_params_p3(); check_params_p3(); break;
          case 3: load_params_p4(); check_params_p4(); break;
          case 4: load_params();                       break;
        }
      break;

      case MC_COMMAND_ACT_LOCK_DEVICE:
        if( input_buffer[1] == 0x55 &&
            input_buffer[2] == 0x5A &&
            input_buffer[3] == 0x55) {
#ifdef DEBUG
  printf("DBG: After saving page0 device will be locked\n");
#endif
                  //gl_chLockBit = 1;     ���� ����� � ������� ������, � �� ������ ����� ��������� ����������! :)
                  flashLockDev = 1;
        }
        else {
#ifdef DEBUG
  printf("DBG: Lock command has wrong parameters!\n");
#endif
        }
      break;
    }
    pos_in_in_buf = 0;
  }
}