#include "mcc_generated_files/mcc.h"
#include <stdbool.h>

// Steering Servo PWM
#define servo_center 70  // 1.5ms pulse width (neutral)
#define servo_left 88     // 1.0ms pulse width (full left)
#define servo_right 52     // 2.0ms pulse width (full right)

// PID Control Gains
#define Kp 0.75
#define Ki 1
#define Kd 1
int integral = 0;
int prev_error = 0;

// Global variables
uint16_t sensor_left = 0;
uint16_t sensor_right = 0;
uint16_t battery_voltage = 0;
uint16_t motor_current = 0;

// Function to Read ADC Sensor Values
void Read_Sensors() {
    sensor_left = ADC_GetConversion(channel_ANC2);  // Left Sensor (RC2)
    sensor_right = ADC_GetConversion(channel_ANC3); // Right Sensor (RC3)
    battery_voltage = ADC_GetConversion(channel_ANC4); // Battery Voltage (RC4)
    motor_current = ADC_GetConversion(channel_ANC5);   // Motor Current (RC5)
}



void Set_Motor_Direction(bool forward, int speed) {
    if (forward) {
        PWM5_LoadDutyValue(0);  // Forward (RA4 = LOW)
        PWM6_LoadDutyValue(speed);  // (RA5 = 0-100)
    } else {
        PWM5_LoadDutyValue(speed); //(RA4 = 0-100)
        PWM6_LoadDutyValue(0);  // Reverse (RA5 = low)
    }
   
}
// Function to Adjust Steering Position via PWM1
void Set_Steering_Angle(uint16_t duty) {
    PWM1_LoadDutyValue(duty);  
}

void Test_Servo(){
    PWM1_LoadDutyValue(servo_left);  // Should move fully left
    __delay_ms(1000);
    PWM1_LoadDutyValue(servo_center);  // Should center
    __delay_ms(1000);
    PWM1_LoadDutyValue(servo_right);  // Should move fully right
    __delay_ms(1000);
}

void Simulate_Sensors() {
    static int test_value = 0;

    //Simulate sensor data changing over time
    if (test_value < 200) {
        sensor_left = 300 + test_value;  // Increasing left sensor value
        sensor_right = 300;  // Fixed right sensor
    } else if (test_value < 400) {
        sensor_left = 300;  // Fixed left sensor
        sensor_right = 300 + (test_value - 200);  // Increasing right sensor value
    } else {
        test_value = 0;  // Reset test cycle
    }

    test_value += 10;  // Increment test values gradually
}
// Basic Steering Logic (without PID)
void Basic_Steering() {
    if (sensor_left > sensor_right ) {
        Set_Steering_Angle(servo_left);  // Turn Left
    } else if (sensor_right > sensor_left) {
        Set_Steering_Angle(servo_right);  // Turn Right
    } else {
        Set_Steering_Angle(servo_center);  // Stay Centered
    }
}

// Proportional Control (P Controller)
void P_Controller() {
    int error = sensor_left - sensor_right;
    int pwm_value = servo_center + (Kp * error);

    // Constrain PWM within valid range
    if (pwm_value < servo_right) pwm_value = servo_right;
    if (pwm_value > servo_left) pwm_value = servo_left;

    Set_Steering_Angle(pwm_value);
}

// Proportional-Integral Control (PI Controller)
void PI_Controller() {
    int error = sensor_left - sensor_right;
    integral += error;  // add all errors

    int pwm_value = servo_center + (Kp * error) + (Ki * integral);

    // Constrain PWM within valid range
    if (pwm_value < servo_right) pwm_value = servo_right;
    if (pwm_value > servo_left) pwm_value = servo_left;

    Set_Steering_Angle(pwm_value);
}

// Proportional-Integral-Derivative Control (PID Controller)
void PID_Controller() {
    int error = sensor_left - sensor_right;
    integral += error;
    int derivative = error - prev_error;

    int pwm_value = servo_center + (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Constrain PWM within valid range
    if (pwm_value < servo_right) pwm_value = servo_right;
    if (pwm_value > servo_left) pwm_value = servo_left;

    Set_Steering_Angle(pwm_value);
   
    prev_error = error;  // Store error for next cycle
}


// Timer0 Interrupt - Runs Every 50ms
void Timer0_Interrupt() {
    Read_Sensors();
    //Test_Servo();    
    //Set_Steering_Angle(sensor_left/6);
    //Simulate_Sensors();
    //Basic_Steering();
    P_Controller();
    //PI_Controller();
    //PID_Controller();
    Set_Motor_Direction(true, 100);
}


// Main Loop
void main(void) {
    SYSTEM_Initialize();  // Initialise PIC peripherals

    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();


    // Set Timer0 Interrupt Handler
    TMR0_SetInterruptHandler(Timer0_Interrupt);


    while (1) {
        // Main loop handled by Timer0 Interrupt
    }
}
