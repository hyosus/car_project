
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include <stdbool.h>
#include <limits.h> // For INT_MAX

// GPIO Definitions
#define ADC_PIN 26 // GPIO 26 (ADC0)
#define DIGI_PIN 27
// Thresholds and Timing
#define THRESHOLD 1000        // ADC midpoint (0-4095 for 12-bit ADC)
#define MAX_ELEMENTS 30       // Each character in the barcode has 9 elements (5 bars and 4 spaces), 9 + 1 for extra space element after each character
#define MAX_BARCODE_LENGTH 90 // Maximum barcode length (10 characters)
#define DEBOUNCE_TIME_US 500  // Debounce time in us
#define SAMPLE_SIZE 500
#define ACTIVE_DURATION 35 // Number of consecutive active readings to consider as end of barcode

bool scanning_completed = false;

// Element Structure. Each scanned element is classified as either a bar or a space.
typedef struct
{
    int width;    // Width of the element calculated in PWM cycles
    bool is_wide; // Wide (1) or Narrow (0)
} Element;

// Array to store elements
Element elements[MAX_ELEMENTS];
Element reversed_elements[MAX_ELEMENTS];
int element_count = 0;

uint16_t ADC_values[SAMPLE_SIZE];
uint16_t HIGH_THRESHOLD, LOW_THRESHOLD;

// Code 39 Patterns and Characters
typedef struct
{
    const int pattern[9]; // Array of 0s and 1s representing narrow and wide elements
    char character;       // Corresponding character
} Code39Mapping;

/*
Complete Code 39 Mapping with provided patterns.
0 = Narrow, 1 = Wide
*/
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
    {{0, 1, 1, 0, 1, 0, 0, 0, 0}, 'Z'},
    {{0, 1, 0, 0, 0, 0, 1, 0, 1}, '-'},
    {{1, 1, 0, 0, 0, 0, 1, 0, 0}, '.'},
    {{0, 1, 0, 1, 0, 1, 0, 0, 0}, '$'},
    {{0, 1, 0, 1, 0, 0, 0, 1, 0}, '/'},
    {{0, 1, 0, 0, 0, 1, 0, 1, 0}, '+'},
    {{0, 0, 0, 1, 0, 1, 0, 1, 0}, '%'},
    {{0, 0, 0, 1, 1, 0, 1, 0, 0}, '0'},
    {{1, 0, 0, 1, 0, 0, 0, 0, 1}, '1'},
    {{0, 0, 1, 1, 0, 0, 0, 0, 1}, '2'},
    {{1, 0, 1, 1, 0, 0, 0, 0, 0}, '3'},
    {{0, 0, 0, 1, 1, 0, 0, 0, 1}, '4'},
    {{1, 0, 0, 1, 1, 0, 0, 0, 0}, '5'},
    {{0, 0, 1, 1, 1, 0, 0, 0, 0}, '6'},
    {{0, 0, 0, 1, 0, 0, 1, 0, 1}, '7'},
    {{1, 0, 0, 1, 0, 0, 1, 0, 0}, '8'},
    {{0, 0, 1, 1, 0, 0, 1, 0, 0}, '9'}};

const int num_mappings = sizeof(code39_mappings) / sizeof(code39_mappings[0]);

// Function to initialize ADC
void init_adc_sensor()
{
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0); // ADC0
}

// void measure_pulse_width_adc(uint adc_pin)
// {
//     uint32_t start_time, end_time;
//     uint32_t active_pulse_width = 0, inactive_pulse_width = 0;
//     // Initialize ADC
//     adc_init();
//     adc_gpio_init(adc_pin);
//     adc_select_input(0);
//     // Detect initial state (active or inactive) at startup
//     uint16_t initial_value = adc_read();
//     bool initial_state = initial_value > THRESHOLD; // 1 for HIGH (dark), 0 for LOW (white)
//     // Start timer when we see a transition from inactive to active
//     if (!initial_state)
//     {
//         start_time = time_us_32();
//     }
//     while (element_count < MAX_ELEMENTS) // Wait for 9 elements
//     {
//         uint16_t current_value = adc_read();
//         bool current_state = current_value > THRESHOLD;
//         // Debounce logic: check if there’s a transition
//         if (current_state != initial_state)
//         {
//             sleep_us(DEBOUNCE_TIME_US); // Apply debounce delay
//             // Re-check after debounce delay to confirm the state
//             current_value = adc_read();
//             current_state = current_value > THRESHOLD;
//             // Confirm transition if the state is still different
//             if (current_state != initial_state)
//             {
//                 if (start_time != 0)
//                 {
//                     end_time = time_us_32();
//                     uint32_t pulse_width = end_time - start_time;
//                     if (current_state == false)
//                     {
//                         printf("Active pulse: %f ms\n", pulse_width / 1000.0f);
//                         active_pulse_width += pulse_width;
//                         elements[element_count].duration_ms = pulse_width / 1000.0f;
//                         element_count++;
//                     }
//                     else // Even number of transitions means it's an inactive pulse
//                     {
//                         printf("Inactive pulse: %f ms\n", pulse_width / 1000.0f);
//                         inactive_pulse_width += pulse_width;
//                         elements[element_count].duration_ms = pulse_width / 1000.0f;
//                         element_count++;
//                     }
//                 }
//                 start_time = time_us_32();
//                 initial_state = current_state;
//             }
//         }
//     }
// }

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

void collect_data()
{
    int consecutive_active_count = 0;

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        adc_select_input(0);
        bool current_state = gpio_get(DIGI_PIN);

        uint16_t sensor_value = adc_read();
        ADC_values[i] = sensor_value;
        printf("Sensor value: %d\n", sensor_value);

        sleep_ms(100);
    }
}

void find_threshold()
{
    uint16_t max = 0;
    uint16_t min = 4095;

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        if (ADC_values[i] > max)
        {
            max = ADC_values[i];
        }
        if (ADC_values[i] < min)
        {
            min = ADC_values[i];
        }
    }

    HIGH_THRESHOLD = max * 0.48; // 78% of max value
    LOW_THRESHOLD = min * 3;     // 120% of min value
}

void find_width()
{
    int high_count = 0;  // To count width of high pulse
    int low_count = 0;   // To count width of low pulse
    bool is_high = true; // To alternate between high and low pulses

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        if (ADC_values[i] >= HIGH_THRESHOLD && is_high) // Start of a high pulse
        {
            high_count = 1; // Initialize high pulse count

            // Loop to keep counting as long as ADC values remain above threshold
            for (int j = i + 1; j < SAMPLE_SIZE; j++)
            {
                if (ADC_values[j] < HIGH_THRESHOLD) // High pulse ends
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
        else if (ADC_values[i] <= LOW_THRESHOLD && !is_high) // Start of a low pulse
        {
            low_count = 1; // Initialize low pulse count

            // Loop to keep counting as long as ADC values remain below threshold
            for (int j = i + 1; j < SAMPLE_SIZE; j++)
            {
                if (ADC_values[j] > LOW_THRESHOLD) // Low pulse ends
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
    char decoded_chars[3];

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

    // If first decoded char is not "*", reverse the array and decode again
    if (decoded_chars[0] != '*')
    {
        reverse_array(elements, total_elements);
        decode_elements(reversed_elements, total_elements);
    }
}

int main()
{
    // Initialize standard IO
    stdio_init_all();
    // Initialize ADC
    init_adc_sensor();
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
            find_threshold();
            printf("High threshold: %d, Low threshold: %d\n", HIGH_THRESHOLD, LOW_THRESHOLD);

            find_width();

            decode_elements(elements, element_count);

            printf("==== Original Array ====\n");
            for (int i = 0; i < element_count; i++)
            {
                printf("Element %d: %d | %d width\n", i + 1, elements[i].is_wide, elements[i].width);
            }
            printf("==== Reversed Array ====\n");
            for (int i = 0; i < element_count; i++)
            {
                printf("Element %d: %d | %d width\n", i + 1, reversed_elements[i].is_wide, reversed_elements[i].width);
            }

            scanning_complete = false; // Reset after completing processing
            scanning_started = false;  // Prepare for a new scan
            element_count = 0;         // Reset element count
            sleep_ms(5000);            // Delay before checking for the next barcode
        }

        // if (!scanning_started && sensor_value > THRESHOLD)
        // {
        //     scanning_started = true;
        //     printf("Barcode scanning started!\n");
        //     scanning_completed = false;
        //     element_count = 0;
        // }
        // if (scanning_started)
        // {
        //     // Measure pulse widths
        //     measure_pulse_width_adc(ADC_PIN);
        //     scanning_completed = true;
        //     // Wait until scanning is completed
        //     while (!scanning_completed)
        //     {
        //         tight_loop_contents();
        //     }
        //     printf("Scanning completed.");
        //     // Classify elements
        //     classify_elements(elements, element_count);
        //     // Decode character
        //     char decoded_char = decode_character(elements);
        //     printf("Decoded character: %c\n", decoded_char);
        //     // Reset for next character
        //     element_count = 0;
        //     scanning_completed = false;
        //     // Check if we've reached the end of the barcode
        //     if (decoded_char == '*')
        //     { //'*' marks the end of the barcode
        //         printf("Barcode scan completed.\n");
        //         scanning_started = false;
        //     }
        // }
    }
    return 0;
}
