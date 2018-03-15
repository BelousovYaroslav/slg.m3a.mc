#ifndef T39_M3A_ANALOGUE_PARAMETERS_LIST
#define T39_M3A_ANALOGUE_PARAMETERS_LIST

//ПАРАМЕТРЫ СОРТИРОВАНЫЕ ПО ID
#define UTD1                0   //0x00        термодатчик 1
#define UTD2                1   //0x01        термодатчик 2
#define UTD3                2   //0x02        термодатчик 3
#define I1                  3   //0x03        разрядный ток i1
#define I2                  4   //0x04        разрядный ток i2
#define CNTRPC              5   //0x05        напряжение на пьезокорректорах
#define AMPLANG_ALTERA      6   //0x06        амплитуда получаемая от alter'ы
#define AMPLANG_DUS         7   //0x07        амплитуда получаемая с ДУСа
#define RULA                8   //0x08        напряжение RULA
#define AMPLITUDE           9   //0x09        заданная амплитуда колебания

#define TACT_CODE           10  //0x0A        код такта подставки
#define M_COEFF             11  //0x0B        Коэффициент M
#define STARTMODE           12  //0x0C        Начальная мода
#define DECCOEFF_CURRENT    13  //0x0D        Текущий коэффициент вычета
#define CONTROL_I1          14  //0x0E        Контрольный ток поджига I1
#define CONTROL_I2          15  //0x0F        Контрольный ток поджига I2
#define CONTROL_AA          16  //0x10        Контрольный сигнал раскачки AmplAng
#define HV_APPLY_COUNT_SET  17  //0x11        Алгоритм поджига. Заданное кол-во HV тычков
#define HV_APPLY_COUNT_TR   18  //0x12        Алгоритм поджига. Количество HV тычков в этом запуске
#define HV_APPLY_DURAT_SET  19  //0x13        Алгоритм поджига. Заданная длительность HV тычков

#define SIGNCOEFF           20  //0x14        Знаковый коэффициент
#define DEVNUM              21  //0x15        Номер прибора
//#define DEVNUM              22  //0x16        Номер прибора. Старший байт
#define DATE_Y              23  //0x17        Дата.год
#define DATE_M              24  //0x18        Дата.месяц
#define DATE_D              25  //0x19        Дата.день
#define ORG                 26  //0x1A        Организация целиком (для запроса колбасы)
#define ORG_B1              27  //0x1B        Организация.Байт1
#define ORG_B2              28  //0x1C        Организация.Байт2
#define ORG_B3              29  //0x1D        Организация.Байт3

#define ORG_B4              30  //0x1E        Организация.Байт4
#define ORG_B5              31  //0x1F        Организация.Байт5
#define ORG_B6              32  //0x20  " "   Организация.Байт6
#define ORG_B7              33  //0x21  "!"   Организация.Байт7
#define ORG_B8              34  //0x22  """   Организация.Байт8
#define ORG_B9              35  //0x23  "#"   Организация.Байт9
#define ORG_B10             36  //0x24  "$"   Организация.Байт10
#define ORG_B11             37  //0x25  "%"   Организация.Байт11
#define ORG_B12             38  //0x26  "&"   Организация.Байт12
#define ORG_B13             39  //0x27  "'"   Организация.Байт13

#define ORG_B14             40  //0x28  "("   Организация.Байт14
#define ORG_B15             41  //0x29  ")"   Организация.Байт15
#define ORG_B16             42  //0x2A  "*"   Организация.Байт16 БЕЗ завершающего 0 на конце!
#define VERSION             43  //0x2B  "+"   Версия
#define CALIB_T1            44  //0x2C  ","   Температура нижней температурной точки калибровки
#define T1_TD1              45  //0x2D  "-"   показания датчика TD1 на нижней темп. точке
#define T1_TD2              46  //0x2E  "."   показания датчика TD2 на нижней темп. точке
#define T1_TD3              47  //0x2F  "/"   показания датчика TD3 на нижней темп. точке
#define CALIB_T2            48  //0x30  "0"   Температура верхней температурной точки калибровки
#define T2_TD1              49  //0x31  "1"   показания датчика TD1 на верхней темп. точке

#define T2_TD2              50  //0x32  "2"   показания датчика TD2 на верхней темп. точке
#define T2_TD3              51  //0x33  "3"   показания датчика TD3 на верхней темп. точке
#define HV_APPLY_PACKS      52  //0x34  "4"   Алгоритм поджига. Количество пачек 3kV импульсов поджига

#endif