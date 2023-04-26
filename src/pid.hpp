#ifndef PID_HPP
#define PID_HPP

#define PID_P 4.0
#define SLOW_P 2.0
#define PID_I 1.0
#define SLOW_I 1.0
#define PID_D
#define MIN_REQ_VOLTAGE 50
#define SLOW_MIN_REQ_VOLTAGE 60

class PID {
    private:
    public:
        unsigned long t_lasts[2]; // first element happened first.
        bool current_state;
        unsigned long t_last_update;
        float tps;
        
        float current_voltage;
        float target_speed; // Ticks per second.
        float tot_err;
        PID();
        float update(bool encoder_state, bool is_slow_state);
        void set_target_speed(float target_speed);
        void reset_integral();
};

#endif