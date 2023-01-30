#ifndef PID_HPP
#define PID_HPP

#define PID_P 4.0
#define PID_I 1.0
#define PID_D
#define MIN_REQ_VOLTAGE 60

class PID {
    private:
        unsigned long t_lasts[2]; // first element happened first.
        bool current_state;
        unsigned long t_last_update;
        
        float current_voltage;
        float target_speed; // Ticks per second.
        float tot_err;
    public:
        PID();
        float update(bool encoder_state);
        void set_target_speed(float target_speed);
        void reset_integral();
};

#endif