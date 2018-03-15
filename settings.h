#ifndef SETTINGS_H
#define SETTINGS_H

#define MIN_T_THERMO_CALIBRATION -60
#define MAX_T_THERMO_CALIBRATION 60
#define THERMO_CALIB_PARAMS_BASE 10000

void load_params( void);                        //загрузить параметры из флэш-памяти в переменные
void save_params( void);                        //сохранить параметры из переменных во флэш-память
void SaveThermoCalibPoint( void);               //сохранить калибровочные данные термодатчиков

#endif