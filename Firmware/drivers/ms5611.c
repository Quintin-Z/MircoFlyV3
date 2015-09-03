#include "ms5611.h"
#include "rtthread.h"
#include "rtdevice.h"
#include <math.h>
//#include "hrt.h"
#include <stdio.h>
struct rt_spi_device *ms5611_dev = RT_NULL;

#define    MS5611_GET_TIME_MS()           hrt_absolute_ms()

//#define    MS5611_SPI_WRITE(data)         SPIx_ReadWriteByte(SPI1, data)
//#define    MS5611_SPI_READ()              SPIx_ReadWriteByte(SPI1, 0x00)


#define EXTRA_PRECISION      5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME_MS   10 // conversion time in milliseconds. 10 is minimum
#define PRESSURE_PER_TEMP 5 // Length of reading cycle: 1x temp, rest pressure. Good values: 1-10
#define FIX_TEMP 25         // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
// TLDR: Adjusting for temp changes does more harm than good.


typedef struct
{
	uint16_t psens;
	uint16_t off;
	uint16_t tcs;
	uint16_t tco;
	uint16_t tref;
	uint16_t tsens;
} CalReg;


static CalReg   calReg;
static uint32_t lastPresConv;
static uint32_t lastTempConv;
static int32_t  tempCache;

static uint8_t readState = 0;
static int32_t tempDeltaT;




static void MS5611_DelayMs(uint32_t ms)
{	
//	uint64_t current_ms;
//	current_ms = hrt_absolute_ms();
//	
//	while(current_ms+ms >= hrt_absolute_ms());
	rt_thread_delay(ms);
	
}
static rt_uint8_t MS5611_SPI_WRITE(rt_uint8_t data)
{
	rt_uint8_t recv;
	RT_ASSERT(ms5611_dev != RT_NULL);

	rt_spi_transfer(ms5611_dev, &data,&recv,1);
	return 1;
	
}
//static rt_uint8_t MS5611_SPI_READ(void)
//{
//	rt_uint8_t send=0x00;
//	rt_uint8_t recv;
//	
//	RT_ASSERT(ms5611_dev != RT_NULL);

//	rt_spi_transfer(ms5611_dev, &send,&recv,1);
//	
//	return recv;
//}


uint8_t MS5611_ReadPROM(void)
{
	uint8_t buffer[MS5611_PROM_REG_SIZE];
	uint16_t* pCalRegU16 = (uint16_t*)&calReg;

	rt_uint8_t send_buff;
	for (uint8_t i = 0; i < MS5611_PROM_REG_COUNT; i++)
	{
		send_buff = MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE);
		rt_spi_send_then_recv(ms5611_dev,&send_buff,1,buffer,2);
		pCalRegU16[i] = ((uint16_t)buffer[0] << 8) | buffer[1];
//		MS5611_DelayMs(1);
	}
	return 0;
}

// see page 10 of the datasheet
void MS5611_StartConversion(uint8_t command)
{

	MS5611_SPI_WRITE(command);

}

int32_t MS5611_GetConversion(uint8_t command)
{
	int32_t conversion = 0;
	uint8_t buffer[MS5611_D1D2_SIZE];
	rt_uint8_t send=0x00;

	rt_spi_send_then_recv(ms5611_dev,&send,1,buffer,MS5611_D1D2_SIZE);

	conversion = ((int32_t)buffer[0] << 16) |
	             ((int32_t)buffer[1] << 8) | buffer[2];

	return conversion;
}

uint8_t MS5611_EvaluateSelfTest(float min, float max, float value, char* string)
{
	char buff[100];
	if (value < min || value > max)
	{
		sprintf(buff,"Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
		            string, min, max, value);
		rt_kprintf("%s",buff);
		return RT_FALSE;
	}
	return RT_TRUE;
}

float MS5611_GetPressure(uint8_t osr)
{
	// see datasheet page 7 for formulas
	int64_t off, sens;
	int32_t rawPress = MS5611_RawPressure(osr);
	int64_t dT = (int64_t)MS5611_GetDeltaTemp(osr);
	if (dT == 0)
	{
		return 0;
	}
	off = (((int64_t)calReg.off) << 16) + ((calReg.tco * dT) >> 7);
	sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * dT) >> 8);
	if (rawPress != 0)
	{
		return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
		       / ((1 << EXTRA_PRECISION) * 100.0f);
	}
	else
	{
		return 0;
	}
}

float MS5611_CalcPressure(int32_t rawPress, int32_t dT)
{
	int64_t off;
	int64_t sens;

	if (rawPress == 0 || dT == 0)
	{
		return 0;
	}

	off = (((int64_t)calReg.off) << 16) + ((calReg.tco * (int64_t)dT) >> 7);
	sens = (((int64_t)calReg.psens) << 15) + ((calReg.tcs * (int64_t)dT) >> 8);

	return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
	       / ((1 << EXTRA_PRECISION) * 100.0f);
}

float MS5611_GetTemperature(uint8_t osr)
{
	// see datasheet page 7 for formulas
	int32_t dT;

	dT = MS5611_GetDeltaTemp(osr);
	if (dT != 0)
	{
		return MS5611_CalcTemp(dT);
	}
	else
	{
		return 0;
	}
}

int32_t MS5611_GetDeltaTemp(uint8_t osr)
{
	int32_t rawTemp = MS5611_RawTemperature(osr);
	if (rawTemp != 0)
	{
		return MS5611_CalcDeltaTemp(rawTemp);
	}
	else
	{
		return 0;
	}
}

float MS5611_CalcTemp(int32_t deltaT)
{
	if (deltaT == 0)
	{
		return 0;
	}
	else
	{
		return (float)(((1 << EXTRA_PRECISION) * 2000.0f)
		               + (((int64_t)deltaT * calReg.tsens) >> (23 - EXTRA_PRECISION)))
		       / ((1 << EXTRA_PRECISION) * 100.0f);
	}
}

int32_t MS5611_CalcDeltaTemp(int32_t rawTemp)
{
	if (rawTemp == 0)
	{
		return 0;
	}
	else
	{
		return rawTemp - (((int32_t)calReg.tref) << 8);
	}
}

int32_t MS5611_RawPressure(uint8_t osr)
{
	uint64_t now = rt_tick_get();
	if (lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME_MS)
	{
		lastPresConv = 0;
		return MS5611_GetConversion(MS5611_D1 + osr);
	}
	else
	{
		if (lastPresConv == 0 && lastTempConv == 0)
		{
			MS5611_StartConversion(MS5611_D1 + osr);
			lastPresConv = now;
		}
		return 0;
	}
}

int32_t MS5611_RawTemperature(uint8_t osr)
{
	uint64_t now = rt_tick_get();
	if (lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME_MS)
	{
		lastTempConv = 0;
		tempCache = MS5611_GetConversion(MS5611_D2 + osr);
		return tempCache;
	}
	else
	{
		if (lastTempConv == 0 && lastPresConv == 0)
		{
			MS5611_StartConversion(MS5611_D2 + osr);
			lastTempConv = now;
		}
		return tempCache;
	}
}

uint8_t MS5611_Init(void)
{
	struct rt_spi_configuration cfg;

	ms5611_dev = (struct rt_spi_device *)rt_device_find("ms5611");
//	rt_device_open(ms5611_dev,RT_DEVICE_OFLAG_RDWR);
	
	cfg.data_width = 8;
	cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
	
	cfg.max_hz = 1000*1000;
	rt_spi_configure(ms5611_dev, &cfg);
	
	MS5611_SPI_WRITE(MS5611_RESET);// reset the device to populate its internal PROM registers
	MS5611_DelayMs(5);
	MS5611_ReadPROM();// reads the PROM into object variables for later use
	
//	calReg.off=%d,calReg.psens=%d,calReg.tco=%d,calReg.tcs=%d,calReg.tref=%d,calReg.tsens=%d\n
//	rt_enter_critical();
//	rt_kprintf("\ncalReg.off=%d,calReg.psens=%d,calReg.tco=%d\ncalReg.tcs=%d,calReg.tref=%d,calReg.tsens=%d\n",calReg.off,calReg.psens,calReg.tco,calReg.tcs,calReg.tref,calReg.tsens);
//	rt_exit_critical();

	if(!MS5611_SelfTest())
		return RT_EOK;
	else
		return RT_ERROR;
}

//返回值:0，成功;1，失败
uint8_t MS5611_SelfTest(void)
{
	int32_t rawPress;
	int32_t rawTemp;
	int32_t deltaT;
	float pressure;
	float temperature;

	MS5611_StartConversion(MS5611_D1 + MS5611_OSR_4096);
	MS5611_DelayMs(CONVERSION_TIME_MS);
	rawPress = MS5611_GetConversion(MS5611_D1 + MS5611_OSR_4096);

	MS5611_StartConversion(MS5611_D2 + MS5611_OSR_4096);
	MS5611_DelayMs(CONVERSION_TIME_MS);
	rawTemp = MS5611_GetConversion(MS5611_D2 + MS5611_OSR_4096);

	deltaT = MS5611_CalcDeltaTemp(rawTemp);
	temperature = MS5611_CalcTemp(deltaT);
	pressure = MS5611_CalcPressure(rawPress, deltaT);

	if (MS5611_EvaluateSelfTest(MS5611_ST_PRESS_MIN, MS5611_ST_PRESS_MAX, pressure, "pressure") &&
	        MS5611_EvaluateSelfTest(MS5611_ST_TEMP_MIN, MS5611_ST_TEMP_MAX, temperature, "temperature"))
	{
		rt_kprintf("MS5611 self test OK !\n");
		return 0;
	}
	else
	{
		rt_kprintf("ms5611 self test falid !\n");
		return 1;
	}
}

/**
 * Gets pressure, temperature and above sea level altitude estimate (asl).
 * Best called at 100hz. For every PRESSURE_PER_TEMP-1 pressure readings temp is read once.
 * Effective 50-90hz baro update and 50-10hz temperature update if called at 100hz.
 */
void MS5611_GetData(float* pressure, float* temperature, float* asl)
{
	int32_t tempPressureRaw, tempTemperatureRaw;

	if (readState == 0)
	{
		// read temp
		++readState;
		tempTemperatureRaw = MS5611_GetConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
		tempDeltaT = MS5611_CalcDeltaTemp(tempTemperatureRaw);
		*temperature = MS5611_CalcTemp(tempDeltaT);
		// cmd to read pressure
		MS5611_StartConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
	}
	else
	{
		// read pressure
		++readState;
		tempPressureRaw = MS5611_GetConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
		*pressure = MS5611_CalcPressure(tempPressureRaw, tempDeltaT);
		*asl = MS5611_PressureToAltitude(pressure);
		if (readState == PRESSURE_PER_TEMP)
		{
			// cmd to read temp
			MS5611_StartConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
			readState = 0;
		}
		else
		{
			// cmd to read pressure
			MS5611_StartConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
		}
	}
}

//TODO: pretty expensive function. Rather smooth the pressure estimates and only call this when needed

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
float MS5611_PressureToAltitude(float* pressure/*, float* ground_pressure, float* ground_temp*/)
{
	if (*pressure > 0)
	{
		return ((powf((1015.7f / *pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;
	}
	else
	{
		return 0;
	}
}

INIT_COMPONENT_EXPORT(MS5611_Init);
