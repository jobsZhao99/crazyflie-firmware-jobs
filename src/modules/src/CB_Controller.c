#include "controller.h"
#include "CB_Controller.h"
#include "stabilizer_types.h"
#include "math3d.h"
#include "num.h"
#include "debug.h"



static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
double Vector3Norm(CB_Vector3_t input)
{
	return sqrt(input.x*input.x+input.y*input.y+input.z*input.z);
}

quaternion_t quaternionMultiply(quaternion_t q, quaternion_t p)
{
	float w = -q.x * p.x - q.y * p.y - q.z * p.z + q.w * p.w;
	float x = q.w * p.x + q.z * p.y - q.y * p.z + q.x * p.w;
	float y = -q.z * p.x + q.w * p.y + q.x * p.z + q.y * p.w;
	float z = q.y * p.x - q.x * p.y + q.w * p.z + q.z * p.w;
	return (quaternion_t){.w = w,.x = x, .y = y, .z = z};
}


quaternion_t quaternionDivide(quaternion_t q, quaternion_t p)
{
	p.x*=-1;
	p.y*=-1;
	p.z*=-1;
	return quaternionMultiply(q,p);
}


quaternion_t DCM2UnitQuat(CB_DCM DCM)
{
	quaternion_t output;
	output.w = (float)sqrt(DCM.u1.x+DCM.u1.y+DCM.u1.z+1.0)/2.0f;
	output.x=(float)(DCM.u2.z-DCM.u3.y)/output.w/4.0f;
	output.y=(float)(DCM.u3.x-DCM.u1.z)/output.w/4.0f;
	output.z=(float)(DCM.u1.y-DCM.u2.x)/output.w/4.0f;
	float norm =sqrt(output.x*output.x+output.y*output.y+output.z*output.z+output.w*output.w);
	if(norm!=0)
	{
		output.w/=norm;
		output.x/=norm;
		output.y/=norm;
		output.z/=norm;
	}
	return output;
}

// output=VA*VB
CB_Vector3_t Vector3Cross(CB_Vector3_t VA,CB_Vector3_t VB)
{
	CB_Vector3_t output;
	output.x=VA.y*VB.z-VA.x*VB.y;
	output.y=VA.z*VB.x-VA.x*VB.z;
	output.z=VA.x*VB.y-VA.y*VB.x;
	return output;
}

CB_Vector3_t Vector3Normalize(CB_Vector3_t input)
{

	double norm=Vector3Norm(input);
	input.x/=norm;
	input.y/=norm;
	input.z/=norm;
	return input;
}





uint16_t Thrust2PMW(double Thrust)
{
	return limitThrust((uint32_t)(-47934.626+sqrt(19.4898745+163.33408*Thrust)*10000));
}

void CB_Motor(motors_thrust_t *motorPower, CB_control_t *CB_control)
{
	double a=0.25,b=54.3478,c=41.9143;
	double term1=a*CB_control->thrust;
	double term2=b*CB_control->torque_roll;
	double term3=b*CB_control->torque_pitch;
	double term4=c*CB_control->torque_yaw;

	motorPower->m1=Thrust2PMW(term1-term2-term3-term4);
	motorPower->m2=Thrust2PMW(term1-term2+term3+term4);
	motorPower->m3=Thrust2PMW(term1+term2+term3-term4);
	motorPower->m4=Thrust2PMW(term1+term2-term3+term4);
	if (motorPower->m1 < idleThrust)
	{
		motorPower->m1 = idleThrust;
	}
	if (motorPower->m2 < idleThrust)
	{
		motorPower->m2 = idleThrust;
	}
	if (motorPower->m3 < idleThrust)
	{
		motorPower->m3 = idleThrust;
	}
	if (motorPower->m4 < idleThrust)
	{
		motorPower->m4 = idleThrust;
	}
}


void CB_YawControl(CB_control_t *CB_control,CB_Vector3_t *directionF, const sensorData_t *sensors, const state_t *state,const uint32_t tick)
{
	CB_DCM R;
	R.u3=*directionF;
	R.u2=(CB_Vector3_t){.x=-sin(desire_yaw),.y=cos(desire_yaw),.z=0};
	R.u1=Vector3Cross(R.u2,R.u3);
	R.u1=Vector3Normalize(R.u1);
	R.u2=Vector3Cross(R.u3,R.u1);
	R.u2=Vector3Normalize(R.u2);
	quaternion_t desire_q= DCM2UnitQuat(R);

	quaternion_t delta_q = quaternionDivide(desire_q,state->attitudeQuaternion);
	CB_control->torque_roll = Kq_Roll * (double)delta_q.x + Kw_Roll * (0 - (double)sensors->gyro.x*M_PI/180.0);
	CB_control->torque_pitch = Kq_Pitch * (double)delta_q.y + Kw_Pitch * (0 - (double)sensors->gyro.y*M_PI/180.0); // the same as r_pithc in CB_controller_pid
	CB_control->torque_yaw= Kq_Yaw* (double)delta_q.z + Kw_Yaw * (0 - (double)sensors->gyro.z*M_PI/180.0);

	if (MaxRotate !=0.0	)
	{
		if (fabs(CB_control->torque_roll) > MaxRotate)
		{
			CB_control->torque_roll/=(fabs(CB_control->torque_roll)/MaxRotate);
		}
		if (fabs(CB_control->torque_pitch) > MaxRotate)
		{
			CB_control->torque_pitch/=(fabs(CB_control->torque_pitch)/MaxRotate);
		}
		if (fabs(CB_control->torque_yaw) > MaxRotate)
		{
			CB_control->torque_yaw/=(fabs(CB_control->torque_yaw)/MaxRotate);
		}

	}
	

	//Tau=I*alpha
	CB_control->torque_roll*=5.5*1e-7;
	CB_control->torque_pitch*=5.5*1e-7;
	CB_control->torque_yaw*=11*1e-7;
}


void CB_AttitudeControl(CB_control_t *CB_control, const sensorData_t *sensors, const state_t *state,const uint32_t tick)
{
	

	quaternion_t delta_q = quaternionDivide((quaternion_t){.w=1,.x=0,.y=0,.z=0},state->attitudeQuaternion);
	CB_control->torque_roll = Kq_Roll * (double)delta_q.x + Kw_Roll * (0 - (double)sensors->gyro.x*M_PI/180.0);
	CB_control->torque_pitch = Kq_Pitch * (double)delta_q.y + Kw_Pitch * (0 - (double)sensors->gyro.y*M_PI/180.0); // the same as r_pithc in CB_controller_pid
	CB_control->torque_yaw= Kq_Yaw* (double)delta_q.z + Kw_Yaw * (0 - (double)sensors->gyro.z*M_PI/180.0);

	if (MaxRotate !=0.0	)
	{
		if (fabs(CB_control->torque_roll) > MaxRotate)
		{
			CB_control->torque_roll/=(fabs(CB_control->torque_roll)/MaxRotate);
		}
		if (fabs(CB_control->torque_pitch) > MaxRotate)
		{
			CB_control->torque_pitch/=(fabs(CB_control->torque_pitch)/MaxRotate);
		}
		if (fabs(CB_control->torque_yaw) > MaxRotate)
		{
			CB_control->torque_yaw/=(fabs(CB_control->torque_yaw)/MaxRotate);
		}

	}
	

	//Tau=I*alpha
	CB_control->torque_pitch*=5.5*1e-7;
	CB_control->torque_pitch*=5.5*1e-7;
	CB_control->torque_yaw*=11*1e-7;
}


void CB_PositionControl(CB_control_t *CB_control,CB_Vector3_t *directionF, const state_t *state,const uint32_t tick)
{
	directionF->x=Kp_X*(0.0-(double)state->position.x)+Kd_X*(0-(double)state->velocity.x);
	directionF->y=Kp_Y*(0.0-(double)state->position.y)+Kd_Y*(0-(double)state->velocity.y);
	directionF->z=Kp_Z*(0.0-(double)state->position.z)+Kd_Z*(0-(double)state->velocity.z)+gravity;
	if(fabs(directionF->x)>MaxAcc)
	{
		directionF->x/=(fabs(directionF->x)/MaxAcc);
	}
	if(fabs(directionF->y)>MaxAcc)
	{
		directionF->y/=(fabs(directionF->y)/MaxAcc);
	}

	// this two step can not be exchange
	CB_control->thrust=(double)Vector3Norm(*directionF)*mass;
	*directionF=Vector3Normalize(*directionF);
}


void CB_Controller(CB_control_t *CB_control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,const uint32_t tick)
{
	if(setpoint->thrust>2000)
	{

		CB_Vector3_t DirectionF;
		CB_PositionControl(CB_control,&DirectionF,state,tick);
		CB_YawControl(CB_control,&DirectionF, sensors, state,tick);
		// double x=setpoint->thrust;
		//CB_control->thrust = (double)(6.241*1e-9*x*x+5.488*1e-4*x-0.099)*1e-3*9.81*4;
		//CB_AttitudeControl(CB_control,&DirectionF, sensors, state,tick);
	}
	else
	{
		CB_control->thrust=0;
		CB_control->torque_pitch=0;
		CB_control->torque_pitch=0;
		CB_control->torque_yaw=0;
	}
	
}
