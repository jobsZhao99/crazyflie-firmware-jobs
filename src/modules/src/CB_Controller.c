#include "controller.h"
#include "CB_Controller.h"
#include "math3d.h"
#define desire_roll 0
#define desire_pitch 0
#define desire_yaw 0


void CB_Controller(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{
  
CB_AttitudeControl(&control,&sensors,&state);
}


void CB_AttitudeControl(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{
    
    // dq=qqmul( state->attitude,
//control->roll=Kq_Roll*state->attitudeQuaternion.q1
}

quaternion_t qqmul(quaternion_t q, quaternion_t p) 
{
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
   
	return (quaternion_t){.x=x, .y=y, .z=z, .w=w};
}



void CB_Motor()
{

}