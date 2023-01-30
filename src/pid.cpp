#include "pid.hpp"
#include "Arduino.h"


PID::PID() {
    this->t_lasts[0] = micros()-2000000;
    this->t_lasts[1] = micros()-1000000;
    this->current_state = true;
    this->t_last_update = micros();
    this->target_speed = 0.0; 
}   


float PID::update(bool encoder_state) {
    unsigned long now = micros();
    float dt = (1e-6)*(now-this->t_last_update);
    this->t_last_update = now;
    if (encoder_state &&  !this->current_state) {
        this->t_lasts[0] = this->t_lasts[1];
        this->t_lasts[1] = now;
    }
    this->current_state = encoder_state;
    unsigned long delta1 = this->t_lasts[1]-this->t_lasts[0];
    unsigned long delta2 = now - this->t_lasts[1];
    unsigned long delta = max(delta1, delta2);

    float ticks_per_second = 1e6/((float)(delta));
    if (ticks_per_second < 1) {
        ticks_per_second = 0;
    }

    float error = this->target_speed-ticks_per_second;
    this->current_voltage += error*PID_P*dt;
    if (this->current_voltage > 240.0) {
        this->current_voltage = 240.0;
    }
    if (this->current_voltage < -240.0) {
        this->current_voltage = -240.0;
    }

    // Serial.println("");
    // Serial.print(this->current_voltage);
    // Serial.print(" ");
    // Serial.print(ticks_per_second);
    // Serial.print(" ");
    // Serial.println(this->target_speed);
    
    return this->current_voltage;
}

void PID::set_target_speed(float target_speed) {
    this->target_speed = target_speed;
}