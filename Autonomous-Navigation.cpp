#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "boards/pico.h" 

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Motor pins
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENA 6  // PWM pin for motor A
#define ENB 7  // PWM pin for motor B

//Ultrasonic Sensor pins
#define TRIGGER_PIN 12
#define ECHO_PIN 13

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

// Initialize USB serial
void usb_serial_init() {
    stdio_usb_init();
}

void init_ultrasonic() {
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    // gpio_put(ECHO_PIN, 1);
}

float measure_distance() {
    //sleep_ms(2000);
    //printf("LED, ON1!\r\n");

    gpio_put(TRIGGER_PIN, 1);
    sleep_us(10);
    gpio_put(TRIGGER_PIN, 0);

    absolute_time_t start = get_absolute_time();
    printf("Waiting for ECHO!");
    while (gpio_get(ECHO_PIN) == 0) {
        // sleep_ms(2000);
        // printf(".");
        start = get_absolute_time();
    }
    // sleep_ms(2000);
    // printf("LED, ON3!\r\n");

    absolute_time_t end = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1) {
        end = get_absolute_time();
    }

    //sleep_ms(2000);
    //printf("LED, ON4!\r\n");

    int64_t pulse_width_us = absolute_time_diff_us(start, end);
    float distance_cm = (pulse_width_us / 2.0) / 29.1;
    
    //sleep_ms(2000);
    //printf("LED, ON5!\r\n");

    return distance_cm;
}

void init_motor_pins() {
    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_init(IN3);
    gpio_set_dir(IN3, GPIO_OUT);
    gpio_init(IN4);
    gpio_set_dir(IN4, GPIO_OUT);
    
    gpio_set_function(ENA, GPIO_FUNC_PWM);
    gpio_set_function(ENB, GPIO_FUNC_PWM);
}

void set_motor_forward(bool forward) {
    gpio_put(IN1, forward ? 1 : 0);
    gpio_put(IN2, forward ? 0 : 1);
    gpio_put(IN3, forward ? 1 : 0);
    gpio_put(IN4, forward ? 0 : 1);
}

void set_motor_rotate(bool rotate_rigth){
    gpio_put(IN1, rotate_rigth ? 1 : 0);
    gpio_put(IN2, rotate_rigth ? 0 : 1);
    gpio_put(IN3, rotate_rigth ? 0 : 1);
    gpio_put(IN4, rotate_rigth ? 1 : 0);
}

void motor_stop(bool stop) {
    gpio_put(IN1, stop ? 0 : 1);
    gpio_put(IN2, stop ? 0 : 1);
    gpio_put(IN3, stop ? 0 : 1);
    gpio_put(IN4, stop ? 0 : 1);
}

void set_motor_angular_speed(uint slice_num, uint16_t angular_speed) {
    pwm_set_gpio_level(ENA, angular_speed);
    pwm_set_gpio_level(ENB, angular_speed);
}

void set_motor_A_speed(uint slice_num, uint16_t speed) {
    pwm_set_gpio_level(ENA, speed);
    pwm_set_gpio_level(ENB, speed);    
}

void set_motor_B_speed(uint slice_num, uint16_t speed) {
    pwm_set_gpio_level(ENA, speed);
    pwm_set_gpio_level(ENB, speed);
}




int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

     // Initialize All pins
    stdio_init_all();

    // Initialize Motor
    init_motor_pins();

    // Initialize the USB serial
    usb_serial_init();

    // Initialize Ultrasonic
    init_ultrasonic();

    // Onboard LED
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    uint slice_num_A = pwm_gpio_to_slice_num(ENA);
    uint slice_num_B = pwm_gpio_to_slice_num(ENB);

    pwm_set_wrap(slice_num_A, 255);
    pwm_set_wrap(slice_num_B, 255);

    pwm_set_chan_level(slice_num_A, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_B, PWM_CHAN_B, 0);

    pwm_set_enabled(slice_num_A, true);
    pwm_set_enabled(slice_num_B, true);

    printf("SET MOTOR SPEED!\r\n");

    uint speed_motor_A = 250;  // Set speed (0-255)
    uint angular_speed = 0;  // Set speed (0-255)
    
    while (true) {
        
        uint speed_motor_B = speed_motor_A - angular_speed;
        float distance = measure_distance();
        gpio_put(LED_PIN, 1);

        if (speed_motor_B < 0);
            speed_motor_A = speed_motor_B;
            speed_motor_B = 0;

        set_motor_A_speed(slice_num_A, speed_motor_A);  // duty cycle for motor A
        set_motor_B_speed(slice_num_B, speed_motor_B);  // duty cycle for motor B

        // Send a message every second
        printf("SET, MOTOR STATE AND SPEED!\r\n");
        printf("DISTANCE = %2f \n", distance);

        while (distance >= 30)
        {
            set_motor_forward(true);
            distance = measure_distance();
            printf("FORWARD \r\n");
        }

        while (distance < 30)
        {
            set_motor_rotate(true);
            distance = measure_distance();
            printf("ROTATE  \r\n");
        }
    }

}



/*
// SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
*/