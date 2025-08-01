#include "pid_controller.h"
#include <math.h>

void pid_init(pid_controller_t *pid, double kp, double ki, double kd, 
              double output_min, double output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0;
    pid->output = 0.0;
    pid->error = 0.0;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = -output_max;
    pid->integral_max = output_max;
}

double pid_update(pid_controller_t *pid, double measurement)
{
    // Error hesaplama
    pid->error = pid->setpoint - measurement;
    
    // Proportional term
    double proportional = pid->kp * pid->error;
    
    // Integral term
    pid->integral += pid->error;
    
    // Integral windup koruması
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    
    double integral = pid->ki * pid->integral;
    
    // Derivative term
    pid->derivative = pid->error - pid->prev_error;
    double derivative = pid->kd * pid->derivative;
    
    // Output hesaplama
    pid->output = proportional + integral + derivative;
    
    // Output sınırlama
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // Previous error güncelleme
    pid->prev_error = pid->error;
    
    return pid->output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->error = 0.0;
    pid->prev_error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->output = 0.0;
}

void pid_set_gains(pid_controller_t *pid, double kp, double ki, double kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_setpoint(pid_controller_t *pid, double setpoint)
{
    pid->setpoint = setpoint;
} 