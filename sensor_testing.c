//this code was used to test if the robot would print out the values or not from teh built in sensors 
#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64

int main() {
    wb_robot_init();

    WbDeviceTag distance_sensor = wb_robot_get_device("ps0");
    WbDeviceTag light_sensor = wb_robot_get_device("ls0");

    wb_distance_sensor_enable(distance_sensor, TIME_STEP);
    wb_light_sensor_enable(light_sensor, TIME_STEP);

    printf("Sensor readings:\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        double distance_value = wb_distance_sensor_get_value(distance_sensor);
        double light_value = wb_light_sensor_get_value(light_sensor);

        printf("Distance Sensor Value: %.2f\n", distance_value);
        printf("Light Sensor Value: %.2f\n", light_value);
    }

    wb_robot_cleanup();
    return 0;
}

/*results  Sensor Value: 68.15
Light Sensor Value: 4052.14
Distance Sensor Value: 67.11
Light Sensor Value: 4172.96
Distance Sensor Value: 70.73
Light Sensor Value: 4127.03
Distance Sensor Value: 67.63
Light Sensor Value: 4050.12
Distance Sensor Value: 67.90
Light Sensor Value: 4092.71
Distance Sensor Value: 66.86
Light Sensor Value: 4135.17
Distance Sensor Value: 65.90
Light Sensor Value: 4115.11
*//

