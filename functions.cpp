// Header guard

#include "common_headers.h"
//#include "functions.h"
#include "ports.h"


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
    gpio_init(Left_Motor_IN1);
    gpio_set_dir(Left_Motor_IN1, GPIO_OUT);
    gpio_init(Left_Motor_IN2);
    gpio_set_dir(Left_Motor_IN2, GPIO_OUT);
    gpio_init(Right_Motor_IN3);
    gpio_set_dir(Right_Motor_IN3, GPIO_OUT);
    gpio_init(Right_Motor_IN4);
    gpio_set_dir(Right_Motor_IN4, GPIO_OUT);
    
    gpio_set_function(ENA, GPIO_FUNC_PWM);
    gpio_set_function(ENB, GPIO_FUNC_PWM);
}

void set_motor_forward(bool forward) {
    gpio_put(Left_Motor_IN1, forward ? 1 : 0);
    gpio_put(Left_Motor_IN2, forward ? 0 : 1);
    gpio_put(Right_Motor_IN3, forward ? 1 : 0);
    gpio_put(Right_Motor_IN4, forward ? 0 : 1);
}

void set_motor_rotate(bool rotate_rigth){
    gpio_put(Left_Motor_IN1, rotate_rigth ? 1 : 0);
    gpio_put(Left_Motor_IN2, rotate_rigth ? 0 : 1);
    gpio_put(Right_Motor_IN3, rotate_rigth ? 0 : 1);
    gpio_put(Right_Motor_IN4, rotate_rigth ? 1 : 0);
}

void motor_stop(bool stop) {
    gpio_put(Left_Motor_IN1, stop ? 0 : 1);
    gpio_put(Left_Motor_IN2, stop ? 0 : 1);
    gpio_put(Right_Motor_IN3, stop ? 0 : 1);
    gpio_put(Right_Motor_IN4, stop ? 0 : 1);
}

uint16_t speed = 0; uint16_t angular_speed = 0;

void control_vehicle(const char *command) {
    if (strcmp(command, "forward") == 0) {
        gpio_put(Left_Motor_IN1, 1);
        gpio_put(Left_Motor_IN2, 0);
        gpio_put(Right_Motor_IN3, 1);
        gpio_put(Right_Motor_IN4, 0);
    } else if (strcmp(command, "backward") == 0) {
        gpio_put(Left_Motor_IN1, 0);
        gpio_put(Left_Motor_IN2, 1);
        gpio_put(Right_Motor_IN3, 0);
        gpio_put(Right_Motor_IN4, 1);
    } else if (strcmp(command, "linear_speed_up") == 0) {
        speed =+ 10;
    } else if (strcmp(command, "linear_speed_down") == 0) {
        speed =- 10;
    } else if (strcmp(command, "rot_speed_up") == 0) {
        angular_speed =+ 10;
    } else if (strcmp(command, "rot_speed_down") == 0) {
        angular_speed =- 10;
    } else {
        // Stop the vehicle
        gpio_put(Left_Motor_IN1, 0);
        gpio_put(Left_Motor_IN2, 0);
        gpio_put(Right_Motor_IN3, 0);
        gpio_put(Right_Motor_IN4, 0);
    }
}

void set_motor_angular_speed(uint slice_num, uint16_t angular_speed) {
    pwm_set_gpio_level(ENA, angular_speed);
    pwm_set_gpio_level(ENB, angular_speed);
}

void set_motor_speed(uint slice_num, uint16_t speed, uint16_t angular_speed) {
    pwm_set_gpio_level(ENA, abs(speed));
    pwm_set_gpio_level(ENB, abs(speed+angular_speed));    
}

void set_motor_B_speed(uint slice_num, uint16_t speed) {
    pwm_set_gpio_level(ENA, speed);
    pwm_set_gpio_level(ENB, speed);
}

// Function to handle received data
// char received_command[MAX_COMMAND_LEN];
char *received_command = NULL;
err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p != NULL) {
        // Process received data
        memcpy(received_command, p->payload, p->len);
        received_command[p->len] = '\0'; // Null-terminate the string
        printf("Received command: %s\n", received_command);

        // Pass the received command to the main loop
        //*((char **)arg) = strdup(command);

        // Free the pbuf
        pbuf_free(p);
    } else {
        // Connection closed by client
        tcp_close(tpcb);
    }
    return ERR_ABRT; // Abort the connection
}

err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, recv_callback);
    return ERR_OK;
}

static struct tcp_pcb *server_pcb;
void setup_tcp_server() {
    server_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!server_pcb) {
        printf("Failed to create PCB\n");
        return;
    }

    err_t err = tcp_bind(server_pcb, IP_ANY_TYPE, SERVER_PORT);
    if (err != ERR_OK) {
        printf("Failed to bind PCB: %d\n", err);
        return;
    }

    server_pcb = tcp_listen(server_pcb);
    tcp_accept(server_pcb, accept_callback);

    printf("Server listening on port %d\n", SERVER_PORT);
}

