//left wall follow algo, turning is done via changing motors speed, wall detection is via distance sensor , no light reading 


#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

void detect_wall(WbDeviceTag *ps, int *front_left, int *front_right, int *side_left, int *side_right) {
    double ps_values[8];
    for (int i = 0; i < 8; i++) {
        ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }
    *front_left = ps_values[7] > 80;   
    *front_right = ps_values[0] > 80;  
    *side_left = ps_values[6] > 120;   
    *side_right = ps_values[1] > 120;  
}

int main() {
    wb_robot_init();

    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    WbDeviceTag ps[8];
    char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    for (int i = 0; i < 8; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    int front_left, front_right, side_left, side_right;
    double current_time;

    while (wb_robot_step(TIME_STEP) != -1) {
        detect_wall(ps, &front_left, &front_right, &side_left, &side_right);
        current_time = wb_robot_get_time();

        double left_motor_speed = 0.0;
        double right_motor_speed = 0.0;

        if (front_left || front_right) {
            left_motor_speed = -0.3 * MAX_SPEED;
            right_motor_speed = 0.3 * MAX_SPEED;
        } else if (side_left && !front_left) {
            left_motor_speed = 0.5 * MAX_SPEED;
            right_motor_speed = 0.5 * MAX_SPEED;
        } else if (!side_left && side_right) {
            left_motor_speed = 0.3 * MAX_SPEED;
            right_motor_speed = 0.5 * MAX_SPEED;
        } else if (side_left && side_right) {
            left_motor_speed = 0.4 * MAX_SPEED;
            right_motor_speed = 0.4 * MAX_SPEED;
        } else {
            left_motor_speed = 0.5 * MAX_SPEED;
            right_motor_speed = 0.3 * MAX_SPEED;
        }

        wb_motor_set_velocity(left_motor, left_motor_speed);
        wb_motor_set_velocity(right_motor, right_motor_speed);

        if (current_time >= 3372.2) {
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            break;
        }
    }

    wb_robot_cleanup();
    return 0;
}
