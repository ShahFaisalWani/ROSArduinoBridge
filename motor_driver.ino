/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE

#ifdef L298_MOTOR_DRIVER
  void initMotorController() {
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

    // Initially set motor enable pins to LOW to stop motors
    digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
    digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0) {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255) {
      spd = 255;  // Ensure speed doesn't exceed the maximum PWM value
    }

    // Fix the direction logic for each motor
    if (i == LEFT) { 
      if (reverse == 1) {
        digitalWrite(LEFT_MOTOR_FORWARD, HIGH);  // Fix: Forward and backward are swapped
        digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      } else {
        digitalWrite(LEFT_MOTOR_FORWARD, LOW);
        digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);  // Fix: Forward and backward are swapped
      }
      
      // Send the PWM signal to the LEFT_MOTOR_ENABLE pin for speed control
      analogWrite(LEFT_MOTOR_ENABLE, spd);
    } else { /* if (i == RIGHT) */
      if (reverse == 1) {
        digitalWrite(RIGHT_MOTOR_FORWARD, LOW);  // Fix: Forward and backward are swapped
        digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
      } else {
        digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
        digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);  // Fix: Forward and backward are swapped
      }

      // Send the PWM signal to the RIGHT_MOTOR_ENABLE pin for speed control
      analogWrite(RIGHT_MOTOR_ENABLE, spd);
    }
  }

  // Set motor speeds for both motors
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
