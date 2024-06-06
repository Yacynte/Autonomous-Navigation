

#include "functions.cpp"


int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("Failed to connect to Wi-Fi\n");
        return -1;
    }

    printf("Connected to Wi-Fi\n");

    setup_tcp_server();

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

    //uint speed_motor_A = 250;  // Set speed (0-255)
    //uint angular_speed = 0;  // Set speed (0-255)
    set_motor_speed(slice_num_A, speed, angular_speed);

    
    while (true) {
        
        //uint speed_motor_B = speed_motor_A - angular_speed;
        float distance = measure_distance();
        gpio_put(LED_PIN, 1);


        printf("SET, MOTOR STATE AND SPEED!\r\n");
        printf("DISTANCE = %2f \n", distance);

        if (received_command) {
            printf("Main loop received command: %s\n", received_command);
            control_vehicle(received_command);
            free(received_command); // Free the memory
            received_command = NULL; // Reset the pointer
        }

        /*while (distance >= 30)
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
        } */
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