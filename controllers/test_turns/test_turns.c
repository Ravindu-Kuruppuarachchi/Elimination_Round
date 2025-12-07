#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>
// --- Constants from your Main Code ---
#define TIME_STEP 10
#define WHEEL_RADIUS 0.0201
#define AXLE_LENGTH 0.056     // Distance between wheels

// --- Globals ---
WbDeviceTag left_ps, right_ps, left_motor, right_motor;

// --- Helper Functions ---

void wait_seconds(double seconds) {
    double start_time = wb_robot_get_time();
    while (wb_robot_get_time() < start_time + seconds) {
        wb_robot_step(TIME_STEP);
    }
}

void API_turnLeft() {
    printf("Debug: Turning Left 90 degrees...\n");
    
    const double angle = M_PI / 2;  // 90 degrees in radians
    const double speed = 3.0;       // Slower speed for more accurate turning
    const double turnerror = 0;     // Add extra rotation if it under-turns
    
    // Compute required wheel rotation in radians
    // Formula: (Target_Angle * Axle_Length) / (2 * Wheel_Radius)
    double wheel_rotation = ((angle * AXLE_LENGTH) + turnerror) / (2 * WHEEL_RADIUS);

    // Get initial encoder values
    double initial_left_pos = wb_position_sensor_get_value(left_ps);
    double initial_right_pos = wb_position_sensor_get_value(right_ps);

    // Set wheel speeds for turning Left (Left back, Right forward)
    wb_motor_set_velocity(left_motor, -speed);
    wb_motor_set_velocity(right_motor, speed);

    // Monitor sensor values
    while (wb_robot_step(TIME_STEP) != -1) {
        double left_pos = wb_position_sensor_get_value(left_ps);
        double right_pos = wb_position_sensor_get_value(right_ps);

        double left_moved = left_pos - initial_left_pos;
        double right_moved = initial_right_pos - right_pos;  

        // Check if the robot has turned the desired amount
        // We use fabs() because moving backwards produces negative encoder changes
        if (fabs(left_moved) >= wheel_rotation && fabs(right_moved) >= wheel_rotation) {
            break; 
        }
    }

    // Stop the motors
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void API_turnRight() {
    printf("Debug: Turning Right 90 degrees...\n");

    const double angle = M_PI / 2;  
    const double speed = 3.0;
    const double turnerror = 0;
    
    double wheel_rotation = ((angle * AXLE_LENGTH) + turnerror) / (2 * WHEEL_RADIUS);

    double initial_left_pos = wb_position_sensor_get_value(left_ps);
    double initial_right_pos = wb_position_sensor_get_value(right_ps);

    // Set wheel speeds for turning Right (Left forward, Right back)
    wb_motor_set_velocity(left_motor, speed);
    wb_motor_set_velocity(right_motor, -speed);

    while (wb_robot_step(TIME_STEP) != -1) {
        double left_pos = wb_position_sensor_get_value(left_ps);
        double right_pos = wb_position_sensor_get_value(right_ps);

        double left_moved = left_pos - initial_left_pos;
        double right_moved = initial_right_pos - right_pos;  
        
        if (fabs(left_moved) >= wheel_rotation && fabs(right_moved) >= wheel_rotation) {
            break; 
        }
    }

    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

// --- Main Test Loop ---
int main(int argc, char **argv) {
    wb_robot_init();

    printf("--- Initializing Turn Test ---\n");

    // Initialize Motors (Using your specific names)
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    // Initialize Encoders (Using your specific names)
    right_ps = wb_robot_get_device("right wheel sensor");
    left_ps = wb_robot_get_device("left wheel sensor");

    wb_position_sensor_enable(right_ps, TIME_STEP);
    wb_position_sensor_enable(left_ps, TIME_STEP);

    // Warm up delay
    wait_seconds(0.5);

    while (wb_robot_step(TIME_STEP) != -1) {
        
        // 1. Turn Left
        API_turnLeft();
        
        printf("Turn Left Complete. Waiting...\n");
        wait_seconds(1.0);

        // 2. Turn Right
        API_turnRight();

        printf("Turn Right Complete. Waiting...\n");
        wait_seconds(1.0);
    }

    wb_robot_cleanup();
    return 0;
}