#include "pid.h"

void pid_init( pid_data *data, float p, float i, float d) {
  data -> p = p;
  data -> i = i;
  data -> d = 0;

  data->integral = 0;
  data->derivative = 0;
  data->previous_error = 0;
  data->error = 0;
  data->setpoint = 0;
}

float pid_update( pid_data *data, float currentValue, float elapsedSeconds) {
    data->error = data->setpoint - currentValue;
    data->integral += (data->error * elapsedSeconds);
    data->derivative = (data->error - data->previous_error) / elapsedSeconds;
    return data->p * data->error + data->i * data->integral + data->d * data->derivative;
}
