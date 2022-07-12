#include "controller.h"
#include "CB_Controller.h"
#include "stabilizer_types.h"
#include "math3d.h"
#define desire_roll 0
#define desire_pitch 0
#define desire_yaw 0

void CB_AttitudeControl(CB_control_t *control, const sensorData_t *sensors, const state_t *state) ;
quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p);

void CB_Controller(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{

	CB_AttitudeControl(&control,&sensors,&state);
}


void CB_AttitudeControl(CB_control_t *control, const sensorData_t *sensors, const state_t *state) 
{
	quaternion_t desire_q0=(quaternion_t){.x=1,.y=0,.z=0,.w=0};
    quaternion_t delta_q=quaternionMultiply(state->attitudeQuaternion,desire_q0);
	control->roll=Kq_Roll*delta_q.q1+Kw_Roll*(radians(sensors->gyro.x)-0);
	control->pitch=Kq_Pitch*delta_q.q2+Kw_Pitch*(-radians(sensors->gyro.y)-0);// the same as r_pithc in controller_pid
	control->yaw=Kq_Yaw*delta_q.q3+Kw_Yaw*(radians(sensors->gyro.z)-0);
}

quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p) 
{
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
   
	return (quaternion_t){.x=x, .y=y, .z=z, .w=w};
}



void CB_Motor(motors_thrust_t *motorPower,CB_control_t *control )
{
	int16_t r = control->roll / 2.0f;
  	int16_t p = control->pitch / 2.0f;
  	motorPower->m1 = limitThrust(control->thrust - r + p + control->yaw);
  	motorPower->m2 = limitThrust(control->thrust - r - p - control->yaw);
  	motorPower->m3 =  limitThrust(control->thrust + r - p + control->yaw);
  	motorPower->m4 =  limitThrust(control->thrust + r + p - control->yaw);
}