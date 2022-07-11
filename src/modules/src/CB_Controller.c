#include "controller.h"
#include "CB_Controller.h"

#define desire_roll 0
#define desire_pitch 0
#define desire_yaw 0


void CB_Controller(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{
  
CB_AttitudeControl(&control,&sensors,&state);
}


void CB_AttitudeControl(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{
control->roll=Kq_Roll*()
}

void CB_Motor()
{

}