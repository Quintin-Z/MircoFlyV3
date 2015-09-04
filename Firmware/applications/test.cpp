#include <stdio.h>
#include <finsh.h>
#include <sensor.h>

int accel_test(int argc, char** argv)
{
    rt_sensor_t sensor;
    sensors_event_t event;
    char line[64];

    sensor = rt_sensor_get_default(SENSOR_TYPE_ACCELEROMETER);
    if (sensor != RT_NULL)
    {
        int index;
        SensorConfig config = {SENSOR_MODE_NORMAL, SENSOR_DATARATE_400HZ, SENSOR_ACCEL_RANGE_2G};

        rt_sensor_activate(sensor, 1);
        rt_sensor_configure(sensor, &config);

        index = 0;
        index = index;

        while (1) 
        // for (index = 0; index < 10; index ++)
        {
            rt_sensor_poll(sensor, &event);

            sprintf(line, "type: %d, x:%f, y:%f, z:%f", 
                event.type, 
                event.acceleration.x, 
                event.acceleration.y,
                event.acceleration.z);
            rt_kprintf("%s\n", line);
        }
    }
    
    return 0;
}
FINSH_FUNCTION_EXPORT(accel_test, sensor test);
