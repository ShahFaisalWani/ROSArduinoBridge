/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define LEFT_MOTOR_FORWARD   11
  #define LEFT_MOTOR_BACKWARD  12
  #define RIGHT_MOTOR_FORWARD  6
  #define RIGHT_MOTOR_BACKWARD 7
  #define LEFT_MOTOR_ENABLE 9
  #define RIGHT_MOTOR_ENABLE 10
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
