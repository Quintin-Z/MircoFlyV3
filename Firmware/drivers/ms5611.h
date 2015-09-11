#ifndef __MS5611_H_
#define __MS5611_H_

#include <stdint.h>
#include <stdint.h>
#include <rtthread.h>

//#define MS5611_CS_PORT              GPIOC
//#define MS5611_CS_CLK               RCC_AHB1Periph_GPIOC
//#define MS5611_CS_PIN               GPIO_Pin_4
//#define Set_MS5611_CS               MS5611_CS_PORT->BSRRL = MS5611_CS_PIN
//#define Clr_MS5611_CS               MS5611_CS_PORT->BSRRH = MS5611_CS_PIN



// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_OSR_DEFAULT MS5611_OSR_4096

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// Self test parameters. Only checks that values are sane
#define MS5611_ST_PRESS_MAX   (1100.0f) //mbar
#define MS5611_ST_PRESS_MIN   (450.0f)  //mbar
#define MS5611_ST_TEMP_MAX    (60.0f)   //degree celcius
#define MS5611_ST_TEMP_MIN    (-20.0f)  //degree celcius

// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958f //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f

int MS5611_Init(void);

//返回值:0，成功;1，失败
uint8_t MS5611_SelfTest(void);

uint8_t MS5611_EvaluateSelfTest(float min, float max, float value, char* string);

float MS5611_GetPressure(uint8_t osr);
float MS5611_CalcPressure(int32_t rawPress, int32_t dT);
float MS5611_GetTemperature(uint8_t osr);
float MS5611_CalcTemp(int32_t deltaT);
int32_t MS5611_GetDeltaTemp(uint8_t osr);
int32_t MS5611_CalcDeltaTemp(int32_t rawTemp);
int32_t MS5611_RawPressure(uint8_t osr);
int32_t MS5611_RawTemperature(uint8_t osr);

uint8_t MS5611_ReadPROM(void);

void MS5611_StartConversion(uint8_t command);
int32_t MS5611_GetConversion(uint8_t command);

void MS5611_GetData(float* pressure, float* temperature, float* asl);
float MS5611_PressureToAltitude(float* pressure);

#endif
