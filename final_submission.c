//// ================================= With wheel encoder , ultrasonic, PiD also =================================
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include "hardware/adc.h"
#include <math.h>
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "message_buffer.h"
#include "lwip/udp.h"

int adc_result = 0;

// // // =============================== For wifi ===================================
#define UDP_PORT 8080           // udp port, can use any port but this is comtaasn udp port
struct udp_pcb *udp_server_pcb; // pcb for udp connection

#define WIFI_SSID "jana"
#define WIFI_PASSWORD "horan1993"
#define TARGET_IP "172.20.10.2" // DASHBOARD IP ADDRESS
TaskHandle_t wifi_task_handle = NULL;

float speed = 0.0f;
char *direction = NULL;
float angle = 0.0f;

void split_buffer(char *buffer)
{
    char *token;
    // char *direction = NULL;
    char *speed_token = NULL;
    char *angle_token = NULL;
    // float speed = 0.0f;
    // float angle = 0.0f;

    // Get first token
    token = strtok(buffer, " ");
    if (token != NULL)
    {
        direction = token; // Store the first token

        // Get second token
        token = strtok(NULL, " ");
        if (token != NULL)
        {
            speed_token = token; // This is speed for both types of commands

            // Get third token if it exists
            while ((token = strtok(NULL, " ")) != NULL)
            {
                angle_token = token; // This will be angle for turns
            }
        }
    }

    if (direction && speed_token)
    {
        // Check if this is a turning command (Left/Right)
        if (strcmp(direction, "Left") == 0 || strcmp(direction, "Right") == 0)
        {
            if (angle_token)
            { // We have three tokens (turn command)
                speed = atof(speed_token);
                angle = atof(angle_token);
                printf("Direction: %s\n", direction);
                printf("Speed: %.2f\n", speed);
                printf("Angle: %.2f\n", angle);
            }
        }
        else
        {
            // For Forward/Backwards (two tokens)
            speed = atof(speed_token);
            printf("Direction: %s\n", direction);
            printf("Speed: %.2f\n", speed);
        }
    }
    else if (direction)
    {
        printf("Direction: %s\n", direction);
    }
    else
    {
        printf("Invalid message format\n");
    }
}

// New function for other tasks to use
void notify_udp_message(const char *message)
{
    if (wifi_task_handle != NULL)
    {
        // Send pointer as notification value
        xTaskNotify(wifi_task_handle, (uint32_t)message, eSetValueWithOverwrite);
    }
}

void udp_send_data(const char *data)
{
    if (data == NULL || strlen(data) == 0)
    {
        return;
    }

    size_t data_length = strlen(data);
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_length + 1, PBUF_RAM);
    if (!p)
    {
        printf("Failed to allocate UDP buffer\n");
        return;
    }

    memcpy(p->payload, data, data_length);
    ((char *)p->payload)[data_length] = '\0';

    ip_addr_t target_addr;
    ipaddr_aton(TARGET_IP, &target_addr);

    err_t err = udp_sendto(udp_server_pcb, p, &target_addr, UDP_PORT);
    if (err != ERR_OK)
    {
        printf("Failed to send UDP packet\n");
    }
    else
    {
        printf("Sent data: %s\n", data);
    }

    pbuf_free(p);
}

// Function to handle data received from the accelerometer
static void udp_server_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p == NULL)
    { // when no data is received
        printf("Received empty packet.\n");
        return;
    }

    // buffer to store received data from client
    char buffer[p->len + 1]; // +1 for null-termination
    memcpy(buffer, p->payload, p->len);
    buffer[p->len] = '\0'; // Null-terminate the string

    // remove "received data" in future implementation
    printf("Received data: %s\n", buffer);
    split_buffer(buffer);

    pbuf_free(p); // free buffer so it doesnt overflow
}

// Set up the UDP server to listen on the specified port
void setup_udp_server(void)
{
    udp_server_pcb = udp_new(); // new pcb for server
    if (!udp_server_pcb)
    {
        printf("Failed to create UDP server PCB.\n");
        return;
    }

    // Bind the server to any available IP address (aka the wifi its connected to) and port 8080
    if (udp_bind(udp_server_pcb, IP_ADDR_ANY, UDP_PORT) != ERR_OK)
    {
        printf("Failed to bind UDP server.\n");
        return;
    }

    printf("initializing udp server\n");
    // register callback func when data is received frm accelerometer
    udp_recv(udp_server_pcb, udp_server_receive_callback, NULL);

    printf("UDP server listening on Port: %d\n", UDP_PORT);
}

void wifi_task(__unused void *params)
{
    wifi_task_handle = xTaskGetCurrentTaskHandle();
    if (cyw43_arch_init())
    {
        printf("Failed to initialize Wi-Fi.\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Connecting to wifi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("Failed to connect.\n");
        exit(1);
    }
    else
    {
        printf("Connected to wifi.\n");
    }

    // Set up UDP server
    setup_udp_server();
    uint32_t msg_ptr;

    while (true)
    {
        // vTaskDelay(100);
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &msg_ptr, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Convert notification value back to string pointer
            const char *message = (const char *)msg_ptr;
            udp_send_data(message);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        tight_loop_contents();
    }

    if (udp_server_pcb)
    {
        udp_remove(udp_server_pcb);
    }

    cyw43_arch_deinit();
}
// ================================== others ==================================
volatile bool line_following_enabled = false;
#define LINE_DETECTION_THRESHOLD_MS 3000
#define DETECTION_CHECK_INTERVAL_MS 10
// ================================= For PID ==================================

typedef struct
{
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
    uint32_t last_time;
} PIDController;

PIDController right_motor_pid;
PIDController turn_pid;

void pid_init(PIDController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->last_time = to_ms_since_boot(get_absolute_time());
}

void pids()
{
    pid_init(&right_motor_pid, 0.05, 0.005, 0.0);
    pid_init(&turn_pid, 0.05, 0.005, 0.0);
}

void pid_reset(PIDController *pid)
{
    pid->previous_error = 0;
    pid->integral = 0;
    pid->last_time = to_ms_since_boot(get_absolute_time());
}

float pid_update(PIDController *pid, float setpoint, float measured_value)
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    float dt = (current_time - pid->last_time) / 1000.0f; // Convert to seconds
    pid->last_time = current_time;

    float error = setpoint - measured_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    // printf("error: %f, p : %f,integral: %f, derivative: %f\n", error, pid->kp * error, pid->ki * pid->integral, pid->kd * derivative);

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

//================================== For IR ===================================
#define DIGI_PIN 27
#define ADC_PIN 26
#define MAX_ELEMENTS 30
#define SAMPLE_SIZE 6000
#define ACTIVE_DURATION 150
#define LINE_SENSOR_PIN 16

bool scanning_completed = false;
int reverse_count = 0;
bool line_following_mode = false;
// Element Structure for barcode
typedef struct
{
    int width;    // Width of the element
    bool is_wide; // Wide (1) or Narrow (0)
} Element;

// Arrays to store elements
Element elements[MAX_ELEMENTS];
Element reversed_elements[MAX_ELEMENTS];
int element_count = 0;

uint16_t DIGI_values[SAMPLE_SIZE];
uint16_t HIGH_THRESHOLD, LOW_THRESHOLD;

// Code 39 Patterns and Characters
typedef struct
{
    const int pattern[9];
    char character;
} Code39Mapping;

const Code39Mapping code39_mappings[] = {
    {{0, 1, 0, 0, 1, 0, 1, 0, 0}, '*'},
    {{1, 0, 0, 0, 0, 1, 0, 0, 1}, 'A'},
    {{0, 0, 1, 0, 0, 1, 0, 0, 1}, 'B'},
    {{1, 0, 1, 0, 0, 1, 0, 0, 0}, 'C'},
    {{0, 0, 0, 0, 1, 1, 0, 0, 1}, 'D'},
    {{1, 0, 0, 0, 1, 1, 0, 0, 0}, 'E'},
    {{0, 0, 1, 0, 1, 1, 0, 0, 0}, 'F'},
    {{0, 0, 0, 0, 0, 1, 1, 0, 1}, 'G'},
    {{1, 0, 0, 0, 0, 1, 1, 0, 0}, 'H'},
    {{0, 0, 1, 0, 0, 1, 1, 0, 0}, 'I'},
    {{0, 0, 0, 0, 1, 1, 1, 0, 0}, 'J'},
    {{1, 0, 0, 0, 0, 0, 0, 1, 1}, 'K'},
    {{0, 0, 1, 0, 0, 0, 0, 1, 1}, 'L'},
    {{1, 0, 1, 0, 0, 0, 0, 1, 0}, 'M'},
    {{0, 0, 0, 0, 1, 0, 0, 1, 1}, 'N'},
    {{1, 0, 0, 0, 1, 0, 0, 1, 0}, 'O'},
    {{0, 0, 1, 0, 1, 0, 0, 1, 0}, 'P'},
    {{0, 0, 0, 0, 0, 0, 1, 1, 1}, 'Q'},
    {{1, 0, 0, 0, 0, 0, 1, 1, 0}, 'R'},
    {{0, 0, 1, 0, 0, 0, 1, 1, 0}, 'S'},
    {{0, 0, 0, 0, 1, 0, 1, 1, 0}, 'T'},
    {{1, 1, 0, 0, 0, 0, 0, 0, 1}, 'U'},
    {{0, 1, 1, 0, 0, 0, 0, 0, 1}, 'V'},
    {{1, 1, 1, 0, 0, 0, 0, 0, 0}, 'W'},
    {{0, 1, 0, 0, 1, 0, 0, 0, 1}, 'X'},
    {{1, 1, 0, 0, 1, 0, 0, 0, 0}, 'Y'},
    {{0, 1, 1, 0, 1, 0, 0, 0, 0}, 'Z'}};
const int num_mappings = sizeof(code39_mappings) / sizeof(code39_mappings[0]);

// function prototypes
void collect_data();
void find_width();
void classify_elements(Element *elements, int count);
char decode_character(Element *elements_group);
void decode_elements(Element *elements, int total_elements);
void reverse_array(Element *original, int total_elements);
int hamming_distance(const int *pattern1, const int *pattern2, int length);

// ================================= For motor =================================
const uint IN1_MOTOR1 = 0;
const uint IN2_MOTOR1 = 1;
const uint IN1_MOTOR2 = 4;
const uint IN2_MOTOR2 = 5;

const uint PWM_PIN_MOTOR1 = 10;        // For Motor 1
const uint PWM_PIN_MOTOR2 = 12;        // For Motor 2
const uint MAX_PWM = 65535;            // Max PWM value for full speed (16-bit)
volatile bool object_detected = false; // Shared flag for object detection

uint slice_num1;
uint slice_num2;

// Function prototypes
// void setMotorSpeed( uint left_duty_cycle, uint right_duty_cycle);
void setMotorSpeed(float left_duty_cycle, float right_duty_cycle);
void stopMotors();
void moveForward(float left_duty_cycle, float right_duty_cycle);
void moveBackward(float left_duty_cycle, float right_duty_cycle);
void steerLeft(float base_speed);
void steerRight(float base_speed);
void linefollowRight(float left_duty_cycle, float right_duty_cycle);
void linefollowLeft(float left_duty_cycle, float right_duty_cycle);
void sharpturn(float angle, float right_duty_cycle);
#define MAX_CORRECTION 0.2 // Adjust this value as needed
// ================================= For ultrasonic =================================
#define TRIG_PIN 6
#define ECHO_PIN 7
#define SPEED_OF_SOUND 343  // Speed of sound in meters per second
#define STOP_DISTANCE_CM 10 // Distance threshold
#define WINDOW_SIZE 5

int timeout = 26100; // Maximum pulse width time for no object detected

// ================================= For wheel encoder =================================
#define ENCODER_PIN_LEFT 8    // Left encoder pin
#define ENCODER_PIN_RIGHT 2   // Right encoder pin
#define ENCODER_NOTCHES 20.0f // Number of notches on the encoder wheel
#define WHEEL_CIRCUMFERENCE_CM 20.42f
#define WHEEL_BASE_CM 15.0f
#define PI 3.14159f
#define WHEEL_DIAMETER_CM 6.5f
#define STEERING_RATIO 0.5f

volatile uint32_t left_encoder_count = 0;
volatile uint32_t right_encoder_count = 0;
volatile uint32_t last_time_left = 0;
volatile uint32_t last_time_right = 0;
volatile uint32_t pulse_width_left = 0;
volatile uint32_t pulse_width_right = 0;
volatile uint32_t pulse_width_left_prev = 0;
volatile uint32_t pulse_width_right_prev = 0;
volatile bool pulse_detected_left = false;
volatile bool pulse_detected_right = false;
volatile bool left_encoder_initialized = false;
volatile bool right_encoder_initialized = false;
volatile float current_angle = 0.0f;

// Function prototypes for encoder
void gpio_callback(uint gpio, uint32_t events);
// float calculate_speed_cm_per_s(uint32_t pulse_width_us);

// Variables for the left wheel encoder
int leftReadings[WINDOW_SIZE] = {0};
int leftIndex = 0;
int leftTotal = 0;
int leftCount = 0;

// Variables for the right wheel encoder
int rightReadings[WINDOW_SIZE] = {0};
int rightIndex = 0;
int rightTotal = 0;
int rightCount = 0;

// typedef enum
// {
//     moveForward,
//     moveBackward,
//     turnLeft,
//     turnRight,
//     stopMotors
// }carState;

// ================================= Kalman filter state =================================
typedef struct
{
    float estimate;
    float error;
    float process_noise;
    float measurement_noise;
    float gain;
} KalmanState;

void kalman_init(KalmanState *state, float initial_estimate, float initial_error, float process_noise, float measurement_noise)
{
    state->estimate = initial_estimate;
    state->error = initial_error;
    state->process_noise = process_noise;
    state->measurement_noise = measurement_noise;
}

float kalman_update(KalmanState *state, float measurement)
{
    state->error += state->process_noise;
    state->gain = state->error / (state->error + state->measurement_noise);
    state->estimate = state->estimate + state->gain * (measurement - state->estimate);
    state->error = (1 - state->gain) * state->error;
    return state->estimate;
}

void setupUltrasonicPins()
{
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

// Function to calculate the moving average for the left encoder
uint movingAverageLeft(int newReading)
{
    leftTotal -= leftReadings[leftIndex];
    leftReadings[leftIndex] = newReading;
    leftTotal += newReading;
    leftIndex = (leftIndex + 1) % WINDOW_SIZE;
    if (leftCount < WINDOW_SIZE)
        leftCount++;
    return leftTotal / leftCount;
}

// Function to calculate the moving average for the right encoder
uint movingAverageRight(int newReading)
{
    rightTotal -= rightReadings[rightIndex];
    rightReadings[rightIndex] = newReading;
    rightTotal += newReading;
    rightIndex = (rightIndex + 1) % WINDOW_SIZE;
    if (rightCount < WINDOW_SIZE)
        rightCount++;
    return rightTotal / rightCount;
}

uint64_t getPulse()
{
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t width = 0;
    while (gpio_get(ECHO_PIN) == 0)
        tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();

    while (gpio_get(ECHO_PIN) == 1)
    {
        width++;
        sleep_us(1);
        if (width > timeout)
            return 0;
    }

    absolute_time_t endTime = get_absolute_time();
    return absolute_time_diff_us(startTime, endTime);
}

float getCm(KalmanState *kalman_state)
{
    uint64_t pulseLength = getPulse();
    if (pulseLength == 0)
        return 0;                                 // No object detected
    float distance = (pulseLength * 0.0343) / 2;  // Distance in cm
    return kalman_update(kalman_state, distance); // Filtered distance
}

// void setMotorSpeed(uint left_duty_cycle, uint right_duty_cycle)
// {
//     pwm_set_chan_level(slice_num1, PWM_CHAN_A, left_duty_cycle);
//     pwm_set_chan_level(slice_num2, PWM_CHAN_A, right_duty_cycle);
// }
void setMotorSpeed(float left_duty_cycle, float right_duty_cycle)
{
    right_duty_cycle = right_duty_cycle < 0.0 ? 0.0 : (right_duty_cycle > 1.0 ? 1.0 : right_duty_cycle);
    left_duty_cycle = left_duty_cycle < 0.0 ? 0.0 : (left_duty_cycle > 1.0 ? 1.0 : left_duty_cycle);

    uint left_pwm = (uint)(left_duty_cycle * MAX_PWM);
    uint right_pwm = (uint)(right_duty_cycle * MAX_PWM);

    pwm_set_chan_level(slice_num1, PWM_CHAN_A, left_pwm);
    pwm_set_chan_level(slice_num2, PWM_CHAN_A, right_pwm);
}

void moveForward(float left_duty_cycle, float right_duty_cycle)
{
    printf("Motor moving forward");
    gpio_put(IN1_MOTOR1, false);
    gpio_put(IN2_MOTOR1, true);
    gpio_put(IN1_MOTOR2, true);
    gpio_put(IN2_MOTOR2, false);
    // setMotorSpeed( left_duty_cycle, right_duty_cycle);
    setMotorSpeed(right_duty_cycle, left_duty_cycle);
}

void moveBackward(float left_duty_cycle, float right_duty_cycle)
{
    gpio_put(IN1_MOTOR1, true);
    gpio_put(IN2_MOTOR1, false);
    gpio_put(IN1_MOTOR2, false);
    gpio_put(IN2_MOTOR2, true);
    // setMotorSpeed(left_duty_cycle, right_duty_cycle);
    setMotorSpeed(right_duty_cycle, left_duty_cycle);
}

// Dynamic angle calculation
float calculate_turn_angle()
{
    // Get current encoder counts
    float left_distance = (left_encoder_count * PI * WHEEL_DIAMETER_CM) / ENCODER_NOTCHES;
    float right_distance = (right_encoder_count * PI * WHEEL_DIAMETER_CM) / ENCODER_NOTCHES;

    // Calculate angle based on differential wheel movement
    float angle = (right_distance - left_distance) / WHEEL_BASE_CM;
    return (angle * 180.0f) / PI; // Convert radians to degrees
}

// // Then update steering functions to match
// void steerRight(float angle, float base_speed) {
//     // Ensure angle is positive and capped at 90 degrees
//     angle = fabs(angle);
//     if (angle > 90.0f) angle = 90.0f;

//     // Calculate speeds (right wheel faster for right turn)
//     float slow_speed = base_speed * 0.5f;  // Inner wheel at 50% speed
//     float fast_speed = base_speed;         // Outer wheel at full speed

//     // For right turn: right_duty_cycle = fast_speed, left_duty_cycle = slow_speed
//     moveForward(fast_speed, slow_speed);  // Match moveForward parameter order
//     printf("Steering right - right: %.2f, left: %.2f\n", fast_speed, slow_speed);
// }

// void steerLeft(float angle, float base_speed) {
//     // Ensure angle is positive and capped at 90 degrees
//     angle = fabs(angle);
//     if (angle > 90.0f) angle = 90.0f;

//     // Calculate speeds (left wheel faster for left turn)
//     float slow_speed = base_speed * 0.5f;  // Inner wheel at 50% speed
//     float fast_speed = base_speed;         // Outer wheel at full speed

//     // For left turn: right_duty_cycle = slow_speed, left_duty_cycle = fast_speed
//     moveForward(slow_speed, fast_speed);  // Match moveForward parameter order
//     printf("Steering left - right: %.2f, left: %.2f\n", slow_speed, fast_speed);
// }

void steerRight(float base_speed)
{
    float inner_speed = base_speed * 0.4; // Left wheel slower
    float outer_speed = base_speed;       // Right wheel at full speed
    moveForward(inner_speed, outer_speed);
}

void steerLeft(float base_speed)
{
    float outer_speed = base_speed;       // Left wheel at full speed
    float inner_speed = base_speed * 0.4; // Right wheel slower
    moveForward(outer_speed, inner_speed);
}

// Encoder callback
void gpio_callback(uint gpio, uint32_t events)
{
    // printf("GPIO %d event: %d\n", gpio, events);
    uint32_t current_time = time_us_32();
    if (gpio == ENCODER_PIN_LEFT)
    {
        left_encoder_count++;
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            pulse_width_left_prev = current_time - last_time_left;
            uint leftreading = movingAverageLeft(pulse_width_left_prev);
            pulse_detected_left = true;
            last_time_left = current_time;
            left_encoder_initialized = true;
            pulse_width_left = leftreading;
        }
    }
    else if (gpio == ENCODER_PIN_RIGHT)
    {
        right_encoder_count++;
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            pulse_width_right_prev = current_time - last_time_right;
            uint rightreading = movingAverageRight(pulse_width_right_prev);
            pulse_detected_right = true;
            last_time_right = current_time;
            right_encoder_initialized = true;
            pulse_width_right = rightreading;
        }
    }
}

uint32_t calculate_encoder_counts_for_distance(float distance_cm)
{
    float rotations = distance_cm / WHEEL_CIRCUMFERENCE_CM;
    return (uint32_t)(rotations * ENCODER_NOTCHES);
}

// float calculate_speed_cm_per_s(uint32_t pulse_width_us)
// {
//     if (pulse_width_us == 0)
//         return 0.0;
//     float time_per_rotation_s = (pulse_width_us * ENCODER_NOTCHES) / 1e6;
//     return time_per_rotation_s > 0 ? WHEEL_CIRCUMFERENCE_CM / time_per_rotation_s : 0.0;
// }

// Task for monitoring wheel speed
void wheel_encoder_task(void *params)
{
    gpio_init(ENCODER_PIN_LEFT);
    gpio_set_dir(ENCODER_PIN_LEFT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_LEFT);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_LEFT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_init(ENCODER_PIN_RIGHT);
    gpio_set_dir(ENCODER_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_RIGHT);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN_RIGHT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    printf("wheel encoder start\n");
    while (1)
    {
        if (pulse_detected_left)
        {
            // float speed_left_cm_per_s = calculate_speed_cm_per_s(pulse_width_left);
            movingAverageLeft(pulse_width_left);
            // printf("Left Wheel Speed: %u \n", leftreading);
            pulse_detected_left = false;
        }
        if (pulse_detected_right)
        {
            // float speed_right_cm_per_s = calculate_speed_cm_per_s(pulse_width_right);
            movingAverageRight(pulse_width_right);
            // printf("Right Wheel Speed: %u \n", rightreading);
            pulse_detected_right = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("wheel encoder end\n");
}

void stopMotors()
{
    gpio_put(IN1_MOTOR1, false);
    gpio_put(IN2_MOTOR1, false);
    gpio_put(IN1_MOTOR2, false);
    gpio_put(IN2_MOTOR2, false);
    setMotorSpeed(0, 0);
}
// Task for reading distance from ultrasonic sensor
void ultrasonic_task(void *params)
{
    KalmanState kalman_state;
    kalman_init(&kalman_state, 0, 1, 0.3, 0.2); // Initialize Kalman filter

    setupUltrasonicPins();

    while (1)
    {
        float distance = getCm(&kalman_state);
        printf("Distance: %.2f cm\n", distance);
        object_detected = (distance > 0 && distance < STOP_DISTANCE_CM);
        vTaskDelay(pdMS_TO_TICKS(100)); // Sample every 100ms
    }
}

// Task for controlling motor based on distance
void motor_task(void *params)
{
    gpio_set_function(PWM_PIN_MOTOR1, GPIO_FUNC_PWM);
    slice_num1 = pwm_gpio_to_slice_num(PWM_PIN_MOTOR1);
    pwm_set_clkdiv(slice_num1, 0.5);
    pwm_set_wrap(slice_num1, MAX_PWM);
    pwm_set_enabled(slice_num1, true);

    gpio_set_function(PWM_PIN_MOTOR2, GPIO_FUNC_PWM);
    slice_num2 = pwm_gpio_to_slice_num(PWM_PIN_MOTOR2);
    pwm_set_clkdiv(slice_num2, 0.5);
    pwm_set_wrap(slice_num2, MAX_PWM);
    pwm_set_enabled(slice_num2, true);

    gpio_init(IN1_MOTOR1);
    gpio_set_dir(IN1_MOTOR1, GPIO_OUT);
    gpio_init(IN2_MOTOR1);
    gpio_set_dir(IN2_MOTOR1, GPIO_OUT);
    gpio_init(IN1_MOTOR2);
    gpio_set_dir(IN1_MOTOR2, GPIO_OUT);
    gpio_init(IN2_MOTOR2);
    gpio_set_dir(IN2_MOTOR2, GPIO_OUT);

    pids();

    float right_duty_cycle;
    // float base_duty_cycle = 0.8;

    while (1)
    {
        if (object_detected)
        {
            stopMotors();
            if (strcmp(direction, "Backwards") == 0)
            {
                moveBackward(speed, speed);
            }
        }
        else if (!object_detected)
        {

            // Read the encoder speeds for the left and right wheels
            // float speed_left_cm_per_s = calculate_speed_cm_per_s(pulse_width_left);
            // float speed_right_cm_per_s = calculate_speed_cm_per_s(pulse_width_right);

            // // right_duty_cycle = right_duty_cycle < 0.0 ? 0.0 : (right_duty_cycle > 1.0 ? 1.0 : right_duty_cycle);
            float correction_right = pid_update(&right_motor_pid, pulse_width_left, pulse_width_right);

            // right_duty_cycle = base_duty_cycle + (correction_right * MAX_CORRECTION);//line follow
            right_duty_cycle = speed + (correction_right * MAX_CORRECTION); // remote control
            right_duty_cycle = right_duty_cycle > 1.0 ? 1.0 : (right_duty_cycle < 0.1 ? 0.1 : right_duty_cycle);

            // ==============the accelerometer ==============
            if (!line_following_enabled)
            {
                if (strcmp(direction, "Forward") == 0)
                {
                    moveForward(speed * 0.98, speed);
                    printf("Moving forward...\n");
                }
                else if (strcmp(direction, "Backwards") == 0)
                {
                    moveBackward(speed, speed);
                    printf("Moving backwards...\n");
                }
                else if (strcmp(direction, "Right") == 0)
                {
                    steerRight(speed);
                    printf("Turning right...\n");
                }
                else if (strcmp(direction, "Left") == 0)
                {
                    steerLeft(speed);
                    printf("Turning left...\n");
                }
                else if (strcmp(direction, "Stop") == 0)
                {
                    stopMotors();
                    printf("Stopping...\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 100ms
    }
}

void line_task(void *params)
{
    gpio_init(LINE_SENSOR_PIN);
    gpio_set_dir(LINE_SENSOR_PIN, GPIO_IN);

    float base_speed = 0.8f;
    uint32_t line_detection_start = 0;
    bool line_detected = false;

    while (1)
    {
        // on white paper
        if (!line_following_enabled)
        {
            if (adc_result > 3800)
            {
                if (!line_detected)
                {
                    line_detected = true;
                    line_detection_start = get_absolute_time();
                }
                else
                {
                    uint32_t current_time = get_absolute_time();
                    if ((current_time - line_detection_start) >= LINE_DETECTION_THRESHOLD_MS)
                    {
                        line_following_enabled = true;
                    }
                }
            }
            else
            {
                line_detected = false;
            }
        }
        else
        {
            // Line following mode active
            float correction = pid_update(&right_motor_pid, pulse_width_left, pulse_width_right);
            correction = correction > MAX_CORRECTION ? MAX_CORRECTION : (correction < -MAX_CORRECTION ? -MAX_CORRECTION : correction);

            if (gpio_get(LINE_SENSOR_PIN) == 1)
            {
                moveForward(0.8, 1.0);
                printf("Line detected - adjusting with PID\n");
            }
            else
            {
                gpio_put(IN1_MOTOR1, false);
                gpio_put(IN2_MOTOR1, false); // Left motor forward
                gpio_put(IN1_MOTOR2, true);
                gpio_put(IN2_MOTOR2, false); // Right motor forward
                setMotorSpeed(base_speed + (correction * MAX_CORRECTION), base_speed);
                printf("No line - adjusting with PID\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Function to calculate the Hamming distance between two patterns
int hamming_distance(const int *pattern1, const int *pattern2, int length)
{
    int distance = 0;
    for (int i = 0; i < length; i++)
    {
        if (pattern1[i] != pattern2[i])
        {
            distance++;
        }
    }
    return distance;
}

void reverse_array(Element *original, int total_elements)
{
    for (int i = 0; i < total_elements; i++)
    {
        reversed_elements[i] = original[total_elements - i - 1];
    }
}

void find_width()
{
    int high_count = 0; // To count width of high pulse
    int low_count = 0;  // To count width of low pulse
    bool is_high = 1;   // To alternate between high and low pulses

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        if (DIGI_values[i] && is_high) // Start of a high pulse
        {
            high_count = 1; // Initialize high pulse count

            // Loop to keep counting as long as ADC values remain above threshold
            for (int j = i + 1; j < SAMPLE_SIZE; j++)
            {
                if (!DIGI_values[j]) // High pulse ends
                {
                    elements[element_count++].width = high_count; // Store high pulse width
                    i = j - 1;                                    // Continue from this position in outer loop
                    is_high = false;                              // Switch to expect a low pulse next
                    high_count = 0;                               // Reset high pulse counter
                    break;
                }
                high_count++; // Increment high pulse width count
            }
        }
        else if (!DIGI_values[i] && !is_high) // Start of a low pulse
        {
            low_count = 1; // Initialize low pulse count

            // Loop to keep counting as long as ADC values remain below threshold
            for (int j = i + 1; j < SAMPLE_SIZE; j++)
            {
                if (DIGI_values[j]) // Low pulse ends
                {
                    elements[element_count++].width = low_count; // Store low pulse width
                    i = j - 1;                                   // Continue from this position in outer loop
                    is_high = true;                              // Switch to expect a high pulse next
                    low_count = 0;                               // Reset low pulse counter
                    break;
                }
                low_count++; // Increment low pulse width count
            }
        }
    }

    printf("Find width completed\n");
}

void collect_data()
{
    int consecutive_active_count = 0;
    // int sample_count = 0; // Track actual collected samples
    absolute_time_t timeout = make_timeout_time_ms(10000); // 5 second timeout

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        bool current_state = gpio_get(DIGI_PIN);
        DIGI_values[i] = current_state;
        printf("Digital value: %d\n", current_state);

        if (!current_state)
        {
            consecutive_active_count++;
            if (consecutive_active_count >= ACTIVE_DURATION)
            {
                printf("End of barcode.\n");
                break;
            }
        }
        else
        {
            consecutive_active_count = 0;
        }

        // sleep_us(10); // small delay
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent task from hogging CPU

        // Check for timeout
        if (absolute_time_diff_us(get_absolute_time(), timeout) <= 0)
        {
            printf("Timeout reached in collect_digi()\n");
            break;
        }
    }
}

// Function to classify elements as narrow or wide
void classify_elements(Element *elements, int count)
{
    // Find the shortest
    float min_duration = elements[0].width;
    for (int i = 1; i < count; i++)
    {
        if (elements[i].width < min_duration)
        {
            min_duration = elements[i].width; // shortest duration
        }
    }
    // Classify elements based on the shortest duration
    for (int i = 0; i < count; i++)
    {
        if (elements[i].width > min_duration * 3) // wide element is 3 to 3.5 times longer than the narrow element
        {
            elements[i].is_wide = 1; // Wide element
        }
        else
        {
            elements[i].is_wide = 0; // Narrow element
        }
    }

    printf("Classification complete\n");
}

// Function to decode a single character from a group of 9 elements
char decode_character(Element *elements_group)
{
    // Create a temporary pattern array for the 9-element sequence
    int temp_pattern[10];
    for (int i = 0; i < 9; i++)
    {
        temp_pattern[i] = elements_group[i].is_wide ? 1 : 0;
    }

    int min_distance = INT_MAX;
    char closest_char = '?';

    // Compare with each mapping in code39_mappings
    // Find the closest matching character based on Hamming distance
    for (int i = 0; i < num_mappings; i++)
    {
        int distance = hamming_distance(temp_pattern, code39_mappings[i].pattern, 9);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_char = code39_mappings[i].character;
        }
    }

    return closest_char;
}

// grouping them in sets of 9 for decoding of each character.
void decode_elements(Element *elements, int total_elements)
{
    // First, classify all elements as narrow or wide
    classify_elements(elements, total_elements);

    int element_index = 0;
    int char_count = 0;
    char decoded_chars[3] = {0}; // Initialize the array to avoid undefined behavior
    char message[128] = {0};     // Initialize to prevent garbage values

    // Loop through elements and decode each 9-element group
    while (element_index + 9 <= total_elements) // Ensure enough elements remain
    {
        // Collect 9 elements for the current character
        Element elements_group[9];
        for (int i = 0; i < 9; i++)
        {
            elements_group[i] = elements[element_index++];
        }

        // Decode the character using the elements_group and code39_mappings
        decoded_chars[char_count] = decode_character(elements_group);
        printf("Decoded Character %d: %c\n", char_count + 1, decoded_chars[char_count]);
        char_count++;

        // If a separator follows, skip it
        if (element_index < total_elements && elements[element_index].is_wide == 0)
        {
            element_index++;
        }
    }

    // If fewer than 2 characters are decoded, handle it safely
    if (char_count >= 2)
    {
        snprintf(message, sizeof(message), "GHV %c", decoded_chars[1]);
        notify_udp_message(message);
    }
    else
    {
        printf("Error: Not enough characters decoded to access decoded_chars[1].\n");
    }

    // If the first decoded char is not "*", reverse the array and decode again. Make sure to reverse only once.
    if (char_count > 0 && decoded_chars[0] != '*' && reverse_count == 0)
    {
        reverse_count++;
        reverse_array(elements, total_elements);
        decode_elements(reversed_elements, total_elements);
    }
}

void barcode_task(__unused void *params)
{
    // Initialize ADC
    // Initialize digital pin
    gpio_init(DIGI_PIN);
    gpio_set_dir(DIGI_PIN, GPIO_IN);
    gpio_pull_up(DIGI_PIN); // D0 is active LOW

    printf("Scanning for barcodes...\n");
    bool scanning_started = false;
    bool scanning_complete = false;

    while (1)
    {
        bool current_state = gpio_get(DIGI_PIN);

        if (current_state && !scanning_started) // Barcode detected
        {
            scanning_started = true;
            collect_data();
            scanning_complete = true;
        }

        if (scanning_complete)
        {
            find_width();

            decode_elements(elements, element_count);

            printf("==== Original Array ====\n");
            for (int i = 0; i < element_count; i++)
            {
                printf("Element %d: %d | %d width\n", i + 1, elements[i].is_wide, elements[i].width);
            }

            if (reverse_count > 0)
            {
                printf("==== Reversed Array ====\n");
                for (int i = 0; i < element_count; i++)
                {
                    printf("Element %d: %d | %d width\n", i + 1, reversed_elements[i].is_wide, reversed_elements[i].width);
                }
            }

            scanning_complete = false; // Reset after completing processing
            scanning_started = false;  // Prepare for a new scan
            element_count = 0;         // Reset element count
            reverse_count = 0;         // Reset reverse count

            vTaskDelay(3000); // Delay before checking for the next barcode
        }

        // Small delay to prevent task from hogging CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void adc_task(void *params)
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    while (1)
    {
        adc_result = adc_read();
        printf("ADC Value: %d\n", adc_result);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Main and task launch
void vLaunch()
{
    xTaskCreate(wifi_task, "Wifi Task", 2048, NULL, 1, NULL);
    xTaskCreate(motor_task, "Motor Task", 1024, NULL, 1, NULL);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, NULL, 1, NULL);
    xTaskCreate(wheel_encoder_task, "Wheel Encoder Task", 1024, NULL, 1, NULL);
    xTaskCreate(barcode_task, "Barcode Task", 1024, NULL, 6, NULL);
    xTaskCreate(line_task, "Line Task", 1024, NULL, 1, NULL);
    xTaskCreate(adc_task, "ADC Task", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
}

int main()
{
    stdio_init_all();
    printf("Starting FreeRTOS tasks...\n");
    vLaunch();
    return 0;
}