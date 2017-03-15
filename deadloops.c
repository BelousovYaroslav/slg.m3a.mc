#include <ADuC7026.h>
#include "settings.h"
#include "version.h"
#include "errors.h"

//declared in Main.c
extern char gl_c_EmergencyCode;
extern int gl_n_prT1VAL;
extern signed short gl_ssh_SA_time;
extern signed short gl_ssh_ampl_angle;

#define IN_COMMAND_BUF_LEN 3
extern char pos_in_in_buf;
extern char input_buffer[];

//flash-stored params declared in Main.c
extern unsigned short flashParamAmplitudeCode;    //амплитуда колебания виброподвеса
extern unsigned short flashParamTactCode;         //код такта
extern unsigned short flashParamMCoeff;           //коэффициент ошумления
extern unsigned short flashParamStartMode;        //начальная мода
extern unsigned short flashParamI1min;            //контрольное значение токв поджига I1
extern unsigned short flashParamI2min;            //контрольное значение тока поджига I2
extern unsigned short flashParamAmplAngMin1;      //контрольное значение раскачки с ДУСа
extern unsigned short flashParamDecCoeff;         //коэффициент вычета
extern unsigned short flashParamSignCoeff;        //знаковый коэффициент

//implemented in Main.c
extern void send_pack( signed short angle_inc1, short param_indicator, short analog_param);
extern void pause( int n);


void deadloop_no_firing( void) {
  //ОБРАБОТКА ОТКАЗА ПОДЖИГА
#ifdef DEBUG
  printf("DBG: NO LASER FIREUP! DEADLOOP.\n");
#endif

  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_NO_LASER_FIRING;

  //высылка настроечных параметров
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( unsigned short) ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //установить знаковый коэффициент dU
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
        	bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
        	bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));
        break;

        case 50: //сохранить параметры во флэш память
          save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 0, 0);

  } //"мертвый" while
}

void deadloop_no_hangerup( void) {
  //ОБРАБОТКА ОТКАЗА РАСКАЧКИ ВИБРОПОДВЕСА
#ifdef DEBUG
  printf("DBG: NO HANGER VIBRATION! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_INITIAL_AMPL_ANG_TEST_FAIL;

  ADCCP = 0x08;     //мы будем посылать ТОЛЬКО AmplAng
  pause(10);
  ADCCON |= 0x80;   //запуск преобразования

  //высылка настроечных параметров
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
  	//пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( unsigned short) ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //измерение AmplAng (и ТОЛЬКО ЕГО)
    while (!( ADCSTA & 0x01)){}			// ожидаем конца преобразования АЦП (теоретически когда мы приходим сюда он уже должен быть готов)
    gl_ssh_ampl_angle = (ADCDAT >> 16);
    ADCCON |= 0x80;                 //запуск преобразования

    //**********************************************************************
	  // Обработка буфера входящих команд
	  //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
    	switch( input_buffer[0]) {
      	case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
        	flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
        	flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
        	flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
        	flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
        	flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
        	flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //установить знаковый коэффициент dU
        	flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
        	bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
        	bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));
        break;

        case 50: //сохранить параметры во флэш память
        	save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
        	load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
    	//Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 6, gl_ssh_ampl_angle);

  } //"мертвый" захват отказа раскачки виброподвеса
}

void deadloop_no_tact( int nError) {
  //ОБРАБОТКА ОТСУТСТВИЯ ТАКТИРОВАНИЯ
#ifdef DEBUG
  printf("DBG: NO TACT SIGNAL! DEADLOOP.\n");
#endif
  //выставляем код ошибки
  gl_c_EmergencyCode = nError;

  //высылка настроечных параметров
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( unsigned short) ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: ////установить знаковый коэффициент dU
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
          bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
          bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));
        break;

        case 50: //сохранить параметры во флэш память
          save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); break;
      }

      pos_in_in_buf = 0;
    }
    else {
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 0, 0);
    }

  } //"мертвый" while
}

void deadloop_current_unstable( void) {
  //ОБРАБОТКА deadloop ПАДЕНИЯ ТОКА

  //выставляем код ошибки
  gl_c_EmergencyCode = ERROR_CURRENT_UNSTABLE;

  
  //высылка настроечных параметров
  send_pack( 0, 7, flashParamAmplitudeCode);
  send_pack( 0, 8, flashParamTactCode);
  send_pack( 0, 9, flashParamMCoeff);
  send_pack( 0, 10, flashParamStartMode);
  send_pack( 0, 11, flashParamI1min);
  send_pack( 0, 12, flashParamI2min);
  send_pack( 0, 13, flashParamAmplAngMin1);
  send_pack( 0, 14, flashParamDecCoeff);
  send_pack( 0, 15, flashParamSignCoeff);
  send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));

  gl_n_prT1VAL = T1VAL;
  while( 1) {
    //пауза 0,1 секунда
    pause( 327);

    gl_ssh_SA_time = ( unsigned short) ( T1LD + gl_n_prT1VAL - T1VAL) % T1LD;
    gl_n_prT1VAL = T1VAL;

    //**********************************************************************
    // Обработка буфера входящих команд
    //**********************************************************************
    if( pos_in_in_buf == IN_COMMAND_BUF_LEN) {
      switch( input_buffer[0]) {
        case 0: //установить код амплитуды
          flashParamAmplitudeCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 7, flashParamAmplitudeCode);
        break;

        case 1: //установить код такта подставки
          flashParamTactCode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 8, flashParamTactCode);
        break;

        case 2: //установить коэффициент M
          flashParamMCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 9, flashParamMCoeff);
        break;

        case 3: //установить начальную моду
          flashParamStartMode = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 10, flashParamStartMode);
        break;
        
        case 4: //установить минимальный ток I1
          flashParamI1min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 11, flashParamI1min);
        break;

        case 5: //установить минимальный ток I2
          flashParamI2min = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 12, flashParamI2min);
        break;

        case 6: //установить 1ый минимум сигнала AmplAng
          flashParamAmplAngMin1 = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 13, flashParamAmplAngMin1);
        break;

        case 7: //установить коэффициент вычета
          flashParamDecCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 14, flashParamDecCoeff);
        break;

        case 8: //установить знаковый коэффициент dU
          flashParamSignCoeff = input_buffer[1] + ( ( ( short) input_buffer[2]) << 8);
          send_pack( 0, 15, flashParamSignCoeff);
        break;

        /*
        case 9: //в асинхр. режиме вылючить вывод SA (включить вывод dU)
        	bAsyncDu = 1;
        break;

        case 10: //в асинхр. режиме выключить вывод dU (включить вывод SA)
        	bAsyncDu = 0;
        break; */

        case 49: //запрос параметров
          send_pack( 0, 7, flashParamAmplitudeCode);
          send_pack( 0, 8, flashParamTactCode);
          send_pack( 0, 9, flashParamMCoeff);
          send_pack( 0, 10, flashParamStartMode);
          send_pack( 0, 11, flashParamI1min);
          send_pack( 0, 12, flashParamI2min);
          send_pack( 0, 13, flashParamAmplAngMin1);
          send_pack( 0, 14, flashParamDecCoeff);
          send_pack( 0, 15, flashParamSignCoeff);
          send_pack( 0, 16, ( ( VERSION_MINOR * 16) << 8) + (VERSION_MAJOR * 16 + VERSION_MIDDLE));
        break;

        case 50: //сохранить параметры во флэш память
          save_params(); break;

        case 51: //перезагрузить параметры из флэш-памяти и показать их
          load_params(); break;
      }
      
      pos_in_in_buf = 0;
    }
    else
      //Если входящих команд не было, то
      //посылка пустого сообщения с ошибкой
      send_pack( 0, 0, 0);

  } //"мертвый" while
}