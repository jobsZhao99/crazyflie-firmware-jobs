#include "stabilizer_types.h"

// position
#define Kp_X 1
#define Kp_Y 1
#define Kp_Z 1
//velocity
#define Kd_X 1
#define Kd_Y 1
#define Kd_Z 1
//attitude
#define Kq_Roll 1
#define Kq_Pitch 1
#define Kq_Yaw 1
//angular velocity in body frame
#define Kw_Roll 1
#define Kw_Pitch  1
#define Kw_Yaw 1


typedef struct CB_ontrol_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
}CB_control_t;

void CB_Controller(CB_control_t *control, const sensorData_t *sensors, const state_t *state);
void CB_Motor();