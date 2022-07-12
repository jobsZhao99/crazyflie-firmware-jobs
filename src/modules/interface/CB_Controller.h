#include "stabilizer_types.h"

// position
#define Kp_X 30
#define Kp_Y 30
#define Kp_Z 30
//velocity
#define Kd_X 9
#define Kd_Y 9
#define Kd_Z 9
//attitude
#define Kq_Roll 250
#define Kq_Pitch 250
#define Kq_Yaw 250
//angular velocity in body frame
#define Kw_Roll 6
#define Kw_Pitch  6
#define Kw_Yaw 6


typedef struct CB_control_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
}CB_control_t;

void CB_Controller(CB_control_t *CB_control, const sensorData_t *sensors, const state_t *state);
void CB_Motor(motors_thrust_t *motorPower,CB_control_t *CB_Control);