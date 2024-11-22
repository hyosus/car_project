#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "pico/multicore.h"

#define RIGHT_SENSOR_PIN 1 // digital pin (RIGHT)()
#define LINE_SENSOR_PIN 0  // digital pin (LEFT)
#define ADC_RIGHT 27       // analog pin (RIGHT)
#define ADC_LEFT 26        // analog pin (LEFT)
#define THRESHOLD 1500     // threshold for surface contrast detection

#define LEFT_MOTOR_PIN 3
#define RIGHT_MOTOR_PIN 4

void setup()
{
    stdio_init_all();
    gpio_init(LINE_SENSOR_PIN);
    gpio_set_dir(LINE_SENSOR_PIN, GPIO_IN);

    gpio_init(LEFT_MOTOR_PIN);
    gpio_set_dir(LEFT_MOTOR_PIN, GPIO_OUT);

    gpio_init(RIGHT_MOTOR_PIN);
    gpio_set_dir(RIGHT_MOTOR_PIN, GPIO_OUT);
}

void move_forward()
{
    gpio_put(LEFT_MOTOR_PIN, 1);
    gpio_put(RIGHT_MOTOR_PIN, 1);
}

void turn_left()
{
    gpio_put(LEFT_MOTOR_PIN, 0);
    gpio_put(RIGHT_MOTOR_PIN, 1);
}

void turn_right()
{
    gpio_put(LEFT_MOTOR_PIN, 1);
    gpio_put(RIGHT_MOTOR_PIN, 0);
}

void stop()
{
    gpio_put(LEFT_MOTOR_PIN, 0);
    gpio_put(RIGHT_MOTOR_PIN, 0);
}

// Function to measure pulse widths for both active and inactive states
void measure_pulse_width(uint gpio_pin)
{
    uint32_t start_time, end_time;
    uint32_t active_pulse_width = 0, inactive_pulse_width = 0;

    // Detect initial state (active or inactive) at startup
    bool initial_state = gpio_get(gpio_pin); // 1 for HIGH (dark), 0 for LOW (white)

    // Start timer when we see a transition from inactive to active
    if (!initial_state)
    {
        start_time = time_us_32();
    }

    // Keep track of the number of transitions
    int transitions = 0;

    while (1)
    {
        bool current_state = gpio_get(gpio_pin);

        if (current_state != initial_state)
        {
            // Transition detected
            if (start_time != 0)
            {
                end_time = time_us_32();
                uint32_t pulse_width = end_time - start_time;

                if (transitions & 1)
                { // Odd number of transitions means it's an active pulse
                    printf("Active pulse: %f ms\n", pulse_width / 1000.0f);
                    active_pulse_width += pulse_width; // Accumulate active pulse width
                }
                else
                { // Even number of transitions means it's an inactive pulse
                    printf("Inactive pulse: %f ms\n", pulse_width / 1000.0f);
                    inactive_pulse_width += pulse_width; // Accumulate inactive pulse width
                }
            }

            // Reset timer and initial state
            start_time = time_us_32();
            transitions++;
            initial_state = !initial_state;
        }
    }
}

// void line_detection(void)
// {
//     uint16_t left_sensor, right_sensor;

//     while (1)
//     {
//         // Read left sensor
//         adc_select_input(0);
//         left_sensor = adc_read();

//         // Read right sensor
//         adc_select_input(1);
//         right_sensor = adc_read();

//         printf("Left sensor: %d,  Right sensor: %d\n", left_sensor, right_sensor);

//         // Detect surface contrast (simple threshold logic)
//         if (left_sensor > THRESHOLD && right_sensor > THRESHOLD)
//         {
//             printf("Both sensors on DARK\n");
//         }
//         else if (left_sensor < THRESHOLD && right_sensor < THRESHOLD)
//         {
//             printf("Both sensor on WHITE\n");
//         }
//         else if (left_sensor < THRESHOLD && right_sensor > THRESHOLD)
//         {
//             printf("Left sensor on WHITE, Right sensor on DARK\n");
//         }
//         else if (left_sensor > THRESHOLD && right_sensor < THRESHOLD)
//         {
//             printf("Left sensors on DARK, Right sensor ON WHITE\n");
//         }
//         else
//         {
//             printf("???\n");
//         }

//         sleep_ms(1000);
//     }
// }

void line_detection()
{
    while (1)
    {

        bool on_line = gpio_get(LINE_SENSOR_PIN);

        if (on_line)
        {
            printf("On line\n");
            move_forward();
            sleep_ms(1000);

            // Move left to check if there is a left turn
            move_left();

            if (gpio_get(LINE_SENSOR_PIN))
            {
                printf("Left turn detected\n");
                turn_left();
                sleep_ms(1000);
            }
            else
            {
                printf("No left turn detected\n");
                move_forward();
                sleep_ms(1000);
            }
        }
        else
        {
            printf("Off line\n");
        }
    }
}

// Function to be run on core 1
// void core1_entry()
// {
//     // measure_pulse_width(RIGHT_SENSOR_PIN);
//     measure_pulse_width(LEFT_SENSOR_PIN);
// }

int main()
{
    stdio_init_all();
    adc_init();

    adc_gpio_init(ADC_RIGHT);
    adc_gpio_init(ADC_LEFT);

    // Initialize digital pins for pulse width measurement
    gpio_init(LINE_SENSOR_PIN);
    gpio_set_dir(LINE_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(LINE_SENSOR_PIN); // D0 is active LOW

    // Detect line using analog sensors
    line_detection();

    return 0;
}