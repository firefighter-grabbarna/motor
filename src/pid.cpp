#include "pid.hpp"
#include "Arduino.h"


PID::PID() {
    this->t_lasts[0] = micros()-2000000;
    this->t_lasts[1] = micros()-1000000;
    this->current_state = true;
    this->t_last_update = micros();
    this->target_speed = 0.0; 
    this->tot_err = 0.0;
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
    if (this->current_voltage < 0) {
        ticks_per_second = -ticks_per_second;
    }

    float error = this->target_speed-ticks_per_second;

    this->current_voltage += error*PID_P*dt + this->tot_err*PID_I*dt;

    if (this->current_voltage > 240.0) {
        this->current_voltage = 240.0;
    }
    if (this->current_voltage < -240.0) {
        this->current_voltage = -240.0;
    }

    if (this->tot_err > 50.0) {
        this->tot_err = 50.0;
    }
    if (this->tot_err < -50.0) {
        this->tot_err = -50.0;
    }

    this->tot_err += error*dt;
    if (abs(this->target_speed) < 1) {
        this->current_voltage = 0.0;
    }

    float output_voltage = this->current_voltage;
    if (abs(this->current_voltage) > 1.0) {
        if (this-> current_voltage < 0) {
            output_voltage -= MIN_REQ_VOLTAGE;
        } else {
            output_voltage += MIN_REQ_VOLTAGE;
        }
    }

    if (output_voltage > 240.0) {
        output_voltage = 240.0;
    }
    if (output_voltage < -240.0) {
        output_voltage = -240.0;
    }
    

    return output_voltage;
}

void PID::set_target_speed(float target_speed) {
    this->target_speed = target_speed;
}

void PID::reset_integral() {
    this->tot_err = 0.0;
}