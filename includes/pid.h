#ifndef PID_H
#define PID_H

typedef struct pid_data {
    float p,i,d;
    float integral, derivative, previous_error, error, setpoint;
} pid_data;

void pid_init( pid_data *data, float p, float i, float d);

float pid_update( pid_data *data, float currentValue, float elapsedSeconds);
#endif
