#include "stabilizer_types.h"


#define kp 0.0
//30
#define kd 9.0
#define kq 10000.0
#define kw 300.0

// position
#define Kp_X kp
#define Kp_Y kp
#define Kp_Z kp
//velocity
#define Kd_X kd
#define Kd_Y kd
#define Kd_Z kd
//attitude
#define Kq_Roll kq
#define Kq_Pitch kq
#define Kq_Yaw kq
//angular velocity in body frame
#define Kw_Roll kw
#define Kw_Pitch  kw
#define Kw_Yaw kw

#define mass 0.027
#define gravity 9.81
#define DEFAULT_IDLE_THRUST 0
#define limitThrust(VAL) limitUint16(VAL)
#define MaxRotate 0.0
#define MaxAcc 0.1*gravity


// #define desire_roll 0
// #define desire_pitch 0
#define desire_yaw 0

typedef struct CB_control_s {
  double torque_roll;
  double torque_pitch;
  double torque_yaw;
  double thrust;
}CB_control_t;


typedef struct CB_Vector3_s 
{
  double x;
  double y;
  double z;
} CB_Vector3_t;

typedef struct CB_DirectionCosinMatirx
{
  CB_Vector3_t u1;
  CB_Vector3_t u2;
  CB_Vector3_t u3;
}CB_DCM;


void CB_Controller(CB_control_t *CB_control,setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,const uint32_t tick);
void CB_Motor(motors_thrust_t *motorPower,CB_control_t *CB_Control);