/*
 * File      : mpu9250.cpp
 * This file is part of MircoFly Project
 * COPYRIGHT (C) 2015, MircoFly Development Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 *
 */

#include <string.h>
#include <stdio.h>
#include <rtdevice.h>

#include "mpu9250.hpp"

MPU9250::MPU9250(int sensor_type, const char* spi_bus)
	: SensorBase(sensor_type)	
{
	struct rt_spi_configuration cfg;
	this->spi_device = (struct rt_spi_device *)rt_device_find(spi_bus);
	if(this->spi_device == RT_NULL)
	{
		rt_kprintf("MPU9250: No SPI device:%s\n",spi_bus);
	}else
	{
		cfg.data_width = 8;
		cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible Modes 0 */
		//SPI3 = 84M/4,8,16,32 = 21M, 10.5M, 5.25M, ...
		cfg.max_hz = 1000; /* 11000kbit/s */ 
		rt_spi_configure(this->spi_device, &cfg);	
	}
	
    /* register to sensor manager */
    SensorManager::registerSensor(this);	
}

int MPU9250::read_reg(rt_uint8_t reg, rt_uint8_t *value)
{
	RT_ASSERT(this->spi_device != RT_NULL);

	reg = (0x80 | reg);
	return rt_spi_send_then_recv(this->spi_device,&reg,1,value,1);
}

int MPU9250::read_buffer(rt_uint8_t reg, rt_uint8_t* value, rt_size_t size)
{
	rt_uint8_t send_data = (0x80 | reg);
	RT_ASSERT(this->spi_device != RT_NULL);
	
	return rt_spi_send_then_recv(this->spi_device,&send_data,1,value,size);
}

int MPU9250::write_reg(rt_uint8_t reg, rt_uint8_t value)
{
	rt_uint8_t send_buffer[2];
	rt_uint8_t recv_buffer[2];
	
	RT_ASSERT(this->spi_device != RT_NULL);

	send_buffer[0] = reg;
	send_buffer[1] = value;
	return rt_spi_transfer(this->spi_device, send_buffer, recv_buffer, 2);
}

MPU9250_Accelerometer::MPU9250_Accelerometer(const char* spi_name)
    : MPU9250(SENSOR_TYPE_ACCELEROMETER, spi_name)
{
	this->acc_sensor_info.name = "Accelerometer";
	this->acc_sensor_info.vendor = "Invensense";
	this->acc_sensor_info.version = 0;
	this->acc_sensor_info.handle = 0;
	this->acc_sensor_info.type = SENSOR_TYPE_ACCELEROMETER;
	this->acc_sensor_info.maxRange = SENSOR_ACCEL_RANGE_16G;
	this->acc_sensor_info.resolution = 1.0f;
	this->acc_sensor_info.power = 0.5f;	
	this->acc_sensor_info.minDelay = 10000;
	this->acc_sensor_info.fifoReservedEventCount = 0;
	this->acc_sensor_info.fifoMaxEventCount = 64;
		
	rt_uint8_t id = 0;
	read_reg(MPU9250_WHO_AM_I, &id);
	if(id == WHOAMI_RESET_VAL)
	{
		rt_kprintf("MPU6500 detection\n");
	}
	
    SensorConfig config = {SENSOR_MODE_NORMAL, SENSOR_DATARATE_400HZ, SENSOR_ACCEL_RANGE_2G};

	/* initialize MPU9250 */
    write_reg(MPU9250_PWR_MGMT_1,   0x80);			/* reset MPU9250 device 									*/
	write_reg(MPU9250_SMPLRT_DIV,   0x00);			/* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 	*/
    write_reg(MPU9250_PWR_MGMT_1,   0x03);			/* Wake up device , set device clock Z axis gyroscope		*/
	write_reg(MPU9250_CONFIG,   	0X01);			/* set DLPF_CFG 42Hz										*/
    write_reg(MPU9250_GYRO_CONFIG,  0x18);			/* set gyro 2000deg/s 										*/
    write_reg(MPU9250_ACCEL_CONFIG, 0x08);			/* set acc +-4g/s 											*/

	this->enable = RT_FALSE;
	this->sensitivity = SENSOR_ACCEL_SENSITIVITY_2G;
	this->config = config;

}


int 
MPU9250_Accelerometer::configure(SensorConfig *config)
{
	int range;
	uint8_t value;

	if (config == RT_NULL) return -1;

	/* TODO: set datarate */

	/* get range and calc the sensitivity */
	range = config->range.accel_range;
	switch (range)
	{
	case SENSOR_ACCEL_RANGE_2G:
		this->sensitivity = SENSOR_ACCEL_SENSITIVITY_2G;
		range = 0;
		break;
	case SENSOR_ACCEL_RANGE_4G:
		this->sensitivity = SENSOR_ACCEL_SENSITIVITY_4G;
		range = 0x01 << 2;
		break;
	case SENSOR_ACCEL_RANGE_8G:
		this->sensitivity = SENSOR_ACCEL_SENSITIVITY_8G;
		range = 0x02 << 2;
		break;
	case SENSOR_ACCEL_RANGE_16G:
		this->sensitivity = SENSOR_ACCEL_SENSITIVITY_16G;
		range = 0x03 << 2;
		break;
	default:
		return -1;
	}

	/* set range to sensor */
	read_reg(MPU9250_ACCEL_CONFIG, &value);
	value &= ~(0x3 << 2);
	value |= range;
	write_reg(MPU9250_ACCEL_CONFIG, value);

    return 0;
}

int 
MPU9250_Accelerometer::activate(int enable)
{
	uint8_t value;

    if (enable && this->enable == RT_FALSE)
    {
        /* enable accelerometer */
		read_reg(MPU9250_PWR_MGMT_2, &value); 
		value &= ~(0x07 << 2);
		write_reg(MPU9250_PWR_MGMT_2, value);
    }

	if (!enable && this->enable == RT_TRUE)
    {
        /* disable accelerometer */
		read_reg(MPU9250_PWR_MGMT_2, &value); 
		value |= (0x07 << 2);
		write_reg(MPU9250_PWR_MGMT_2, value);
    }

	if (enable) this->enable = RT_TRUE;
	else this->enable = RT_FALSE;

    return 0;
}

int 
MPU9250_Accelerometer::poll(sensors_event_t *event)
{

	rt_uint8_t value[6];
	rt_int16_t x, y, z;

    /* parameters check */
    if (event == NULL) return -1;

	/* get event data */
	event->version = sizeof(sensors_event_t);
	event->sensor = (int32_t) this;
	event->timestamp = rt_tick_get();
	event->type = SENSOR_TYPE_ACCELEROMETER;

	read_buffer(MPU9250_ACCEL_XOUT_H, value, 6);

	/* get raw data */
	x = (((rt_int16_t)value[0] << 8) | value[1]);
	y = (((rt_int16_t)value[2] << 8) | value[3]);
	z = (((rt_int16_t)value[4] << 8) | value[5]);

	if (config.mode == SENSOR_MODE_RAW)
	{
		event->raw_acceleration.x = x;
		event->raw_acceleration.y = y;
		event->raw_acceleration.z = z;
	}
	else
	{

		x -= x_offset; y -= y_offset; z -= z_offset;
		event->acceleration.x = x * this->sensitivity * SENSORS_GRAVITY_STANDARD;
		event->acceleration.y = y * this->sensitivity * SENSORS_GRAVITY_STANDARD;
		event->acceleration.z = z * this->sensitivity * SENSORS_GRAVITY_STANDARD;
	}
	
	return 0;
}

void 
MPU9250_Accelerometer::getSensor(sensor_t *sensor)
{
    /* get sensor description */
    if (sensor)
    {
		  rt_memcpy(sensor, &this->acc_sensor_info, sizeof(sensor_t));
    }
}

MPU9250_Gyroscope::MPU9250_Gyroscope(const char* spi_name)
    : MPU9250(SENSOR_TYPE_GYROSCOPE, spi_name)
{
	int index;
	rt_uint8_t value[6];
	rt_int32_t x, y, z;

    /* initialize MPU9250 */
    write_reg(MPU9250_PWR_MGMT_1,   0x80);			/* reset MPU9250 device 									*/
	write_reg(MPU9250_SMPLRT_DIV,   0x00);			/* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 	*/
    write_reg(MPU9250_PWR_MGMT_1,   0x03);			/* Wake up device , set device clock Z axis gyroscope		*/
	write_reg(MPU9250_CONFIG,   	0x03);			/* set DLPF_CFG 42Hz 										*/
    write_reg(MPU9250_GYRO_CONFIG,  0x18);			/* set gyro 2000deg/s										*/
    write_reg(MPU9250_ACCEL_CONFIG, 0x08);			/* set acc +-4g/s 											*/

	x_offset = y_offset = z_offset = 0;
	x = y = z = 0;

	/* get offset */
	for (index = 0; index < 200; index ++)
	{
		read_buffer(MPU9250_GYRO_XOUT_H, value, 6);

		x += (((rt_int16_t)value[0] << 8)   | value[1]);
		y += (((rt_int16_t)value[2] << 8)   | value[3]);
		z += (((rt_int16_t)value[4] << 8)   | value[5]);		
	}
	x_offset = x / 200;
	y_offset = y / 200;
	z_offset = z / 200;

	this->enable = RT_FALSE;
	this->sensitivity = SENSOR_GYRO_SENSITIVITY_250DPS;
}

int 
MPU9250_Gyroscope::configure(SensorConfig *config)
{
	int range;
	uint8_t value;

	if (config == RT_NULL) return -1;

	/* TODO: set datarate */

	/* get range and calc the sensitivity */
	range = config->range.gyro_range;
	switch (range)
	{
	case SENSOR_GYRO_RANGE_250DPS:
		this->sensitivity = SENSOR_GYRO_SENSITIVITY_250DPS;
		range = 0;
		break;
	case SENSOR_GYRO_RANGE_500DPS:
		this->sensitivity = SENSOR_GYRO_SENSITIVITY_500DPS;
		range = 0x01 << 2;
		break;
	case SENSOR_GYRO_RANGE_1000DPS:
		this->sensitivity = SENSOR_GYRO_SENSITIVITY_1000DPS;
		range = 0x02 << 2;
		break;
	case SENSOR_GYRO_RANGE_2000DPS:
		this->sensitivity = SENSOR_GYRO_SENSITIVITY_2000DPS;
		range = 0x03 << 2;
		break;
	default:
		return -1;
	}

	/* set range to sensor */
	read_reg(MPU9250_GYRO_CONFIG, &value);
	value &= ~(0x3 << 2);
	value |= range;
	write_reg(MPU9250_GYRO_CONFIG, value);

    return 0;
}

int 
MPU9250_Gyroscope::activate(int enable)
{
	uint8_t value;
	
    if (enable && this->enable == RT_FALSE)
    {
        /* enable gyroscope */
		read_reg(MPU9250_PWR_MGMT_1, &value);
		value &= ~(0x01 << 4);
		write_reg(MPU9250_PWR_MGMT_1, value);

		read_reg(MPU9250_PWR_MGMT_2, &value); 
		value &= ~(0x07 << 0);
		write_reg(MPU9250_PWR_MGMT_2, value);
    }

	if (!enable && this->enable == RT_TRUE)
    {
        /* disable gyroscope */
		read_reg(MPU9250_PWR_MGMT_2, &value); 
		value |= (0x07 << 0);
		write_reg(MPU9250_PWR_MGMT_2, value);
    }

	if (enable) this->enable = RT_TRUE;
	else this->enable = RT_FALSE;

    return 0;
}

int 
MPU9250_Gyroscope::poll(sensors_event_t *event)
{
	rt_uint8_t value[6];
	rt_int16_t x, y, z;

	/* parameters check */
	if (event == NULL) return -1;

	/* get event data */
	event->version = sizeof(sensors_event_t);
	event->sensor = (int32_t) this;
	event->timestamp = rt_tick_get();
	event->type = SENSOR_TYPE_GYROSCOPE;

	read_buffer(MPU9250_GYRO_XOUT_H, value, 6);

	/* get raw data */
	x = (((rt_int16_t)value[0] << 8) | value[1]);
	y = (((rt_int16_t)value[2] << 8) | value[3]);
	z = (((rt_int16_t)value[4] << 8) | value[5]);

	
	if (config.mode == SENSOR_MODE_RAW)
	{
		event->raw_gyro.x = x;
		event->raw_gyro.y = y;
		event->raw_gyro.z = z;
	}
	else
	{
		x -= x_offset; y -= y_offset; z -= z_offset;
		event->gyro.x = x * this->sensitivity * SENSORS_DPS_TO_RADS;
		event->gyro.y = y * this->sensitivity * SENSORS_DPS_TO_RADS;
		event->gyro.z = z * this->sensitivity * SENSORS_DPS_TO_RADS;
	}
	
	return 0;
}

void 
MPU9250_Gyroscope::getSensor(sensor_t *sensor)
{
    /* get sensor description */
    if (sensor)
    {
		
    }
}

MPU9250_Magnetometer::MPU9250_Magnetometer(const char* spi_name)
    : MPU9250(SENSOR_TYPE_MAGNETIC_FIELD, spi_name)
{
	rt_uint8_t id = 0x00;
	
	write_reg(MPU9250_I2C_MST_CTRL, 0x40); 
	write_reg(MPU9250_USER_CTRL, 0x20); 
	write_reg(MPU9250_I2C_MST_DELAY_CTRL,0x01); 
	
	write_reg(MPU9250_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	write_reg(MPU9250_I2C_SLV0_REG, MPU9250_MAG_WIA);     // Set Write Reg
	write_reg(MPU9250_I2C_SLV0_CTRL, 0x81);          // Start Read
	delay_ms(1);
	read_reg(MPU9250_EXT_SENS_DATA_00, &id);   // Read Data
	if(id == 0x48)
		rt_kprintf("AK8963 detection\n");
	  
//	//SLV0
//	write_reg(MPU9250_I2C_SLV0_ADDR, 0X8C);//AK8975 Address READ
//	write_reg(MPU9250_I2C_SLV0_REG, 0X03);//AK8975 HXL
//	write_reg(MPU9250_I2C_SLV0_CTRL, 0x86);//enable		

//	//SLV1
//	write_reg(MPU9250_I2C_SLV1_ADDR, 0X0C);//AK8975 Address READ
//	write_reg(MPU9250_I2C_SLV1_REG, 0X0A);//AK8975 CNTL1
//	write_reg(MPU9250_I2C_SLV1_CTRL, 0x81);//enable	
//	write_reg(100, 0X11);//16BIT Single measurement mode 
//	
//	write_reg(52, 0x04);//I2C_MST_DLY = 4
//	write_reg(103, 0x03);//I2C_SLV0_DLY_EN 
}

void  
MPU9250_Magnetometer::delay_ms(int ms)		
{
	int i = 0;
	while(ms--)
	{
		i = 12000;
		while(i--);
	}
}

int 
MPU9250_Magnetometer::configure(SensorConfig *config)
{

    return 0;
}

int 
MPU9250_Magnetometer::activate(int enable)
{

    return 0;
}

int 
MPU9250_Magnetometer::poll(sensors_event_t *event)
{

	
	return 0;
}

void 
MPU9250_Magnetometer::getSensor(sensor_t *sensor)
{
    /* get sensor description */
    if (sensor)
    {
		
    }
}
