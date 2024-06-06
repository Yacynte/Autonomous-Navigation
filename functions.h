// Header guard
#ifndef FUNCTIONS_H
#define FUNCTIONS_H


void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

// Initialize USB serial
void usb_serial_init();

void init_ultrasonic();

float measure_distance();

void init_motor_pins() ;

void set_motor_forward(bool forward);

void set_motor_rotate(bool rotate_rigth);

void motor_stop(bool stop);

void control_vehicle(const char *command);

void set_motor_angular_speed(uint slice_num, uint16_t angular_speed);

void set_motor_speed(uint slice_num, uint16_t speed, uint16_t angular_speed);

void set_motor_B_speed(uint slice_num, uint16_t speed);


#endif 