#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define SIMULATION_DURATION 375.4
#define RECORD_INTERVAL 1
#define NUM_LIGHT_SENSORS 8

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

    WbDeviceTag led_top = wb_robot_get_device("led7");
    wb_led_set(led_top, 1);
    wb_led_set(led_top, 0);

    WbDeviceTag ps[8];
    char ps_names[8][4] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    for (int i = 0; i < 8; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    WbDeviceTag ls[NUM_LIGHT_SENSORS];
    char ls_names[NUM_LIGHT_SENSORS][5];
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        sprintf(ls_names[i], "ls%d", i);
        ls[i] = wb_robot_get_device(ls_names[i]);
        wb_light_sensor_enable(ls[i], TIME_STEP);
    }

    int front_left, front_right, side_left, side_right;
    double total_distance = 0.0;
    double previous_time = wb_robot_get_time();
    double current_time;

    double light_values[NUM_LIGHT_SENSORS];
    double recorded_light_avg[360];
    double recorded_times[360];
    double recorded_distances[360];
    int record_count = 0;

    double left_motor_speed = 0.0;
    double right_motor_speed = 0.0;

    while (wb_robot_step(TIME_STEP) != -1) {
        detect_wall(ps, &front_left, &front_right, &side_left, &side_right);
        current_time = wb_robot_get_time();
        previous_time = current_time;

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

        double avg_speed = (left_motor_speed + right_motor_speed) / 2.0;
        double distance_step = avg_speed * (TIME_STEP / 1000.0);
        total_distance += distance_step;

        if (current_time >= RECORD_INTERVAL * (record_count + 1)) {
            double total_light_value = 0.0;
            for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
                light_values[i] = wb_light_sensor_get_value(ls[i]);
                total_light_value += light_values[i];
            }
            double avg_light_value = total_light_value / NUM_LIGHT_SENSORS;
            recorded_light_avg[record_count] = avg_light_value;
            recorded_times[record_count] = current_time;
            recorded_distances[record_count] = total_distance;
            record_count++;
            printf("Recorded data at %.2f seconds: Avg Light Value = %.2f, Total Distance = %.2f\n",
                   current_time, avg_light_value, total_distance);
        }

        if (current_time >= SIMULATION_DURATION) {
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            break;
        }
    }

    double min_light_value = 1e6;
    double min_time = 0.0;
    double min_distance = 0.0;

    for (int i = 0; i < record_count; i++) {
        if (recorded_light_avg[i] < min_light_value) {
            min_light_value = recorded_light_avg[i];
            min_time = recorded_times[i];
            min_distance = recorded_distances[i];
        }
    }

    printf("Max Light Value: %.2f at %.2f seconds, Distance traveled: %.2f meters\n",
           min_light_value, min_time, min_distance);

    printf("Navigating back to the high-intensity area...\n");
    double target_time = min_time;

    previous_time = wb_robot_get_time();

    while (wb_robot_step(TIME_STEP) != -1 && (wb_robot_get_time() - previous_time) < target_time) {
        detect_wall(ps, &front_left, &front_right, &side_left, &side_right);

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
    }

    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);

    printf("Reached high light intensity area, blinking LED red ...mission done...\n");
    wb_led_set(led_top, 1);

    wb_robot_cleanup();
    return 0;
}
