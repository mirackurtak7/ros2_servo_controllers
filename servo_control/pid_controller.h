#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    double kp;          // Proportional gain
    double ki;          // Integral gain
    double kd;          // Derivative gain
    double setpoint;    // Target value
    double output;      // Controller output
    double error;       // Current error
    double prev_error;  // Previous error
    double integral;    // Integral term
    double derivative;  // Derivative term
    double output_min;  // Minimum output limit
    double output_max;  // Maximum output limit
    double integral_min; // Minimum integral limit
    double integral_max; // Maximum integral limit
} pid_controller_t;

// PID controller başlatma
void pid_init(pid_controller_t *pid, double kp, double ki, double kd, 
              double output_min, double output_max);

// PID controller güncelleme
double pid_update(pid_controller_t *pid, double measurement);

// PID controller sıfırlama
void pid_reset(pid_controller_t *pid);

// PID parametrelerini ayarlama
void pid_set_gains(pid_controller_t *pid, double kp, double ki, double kd);

// PID setpoint ayarlama
void pid_set_setpoint(pid_controller_t *pid, double setpoint);

#endif // PID_CONTROLLER_H 