/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */

#ifdef USE_BASE

#if defined(ARDUINO_ENC_COUNTER)

// Pin definitions for encoder (update these based on your setup)
#define LEFT_ENC_PIN_A  2  // Must be an interrupt pin on Arduino Nano
#define LEFT_ENC_PIN_B  4
#define RIGHT_ENC_PIN_A 3  // Must be an interrupt pin on Arduino Nano
#define RIGHT_ENC_PIN_B 5

// Variables to store encoder positions
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

// Encoder lookup table for AB phase encoders
static const int8_t ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// Flags to trigger debug messages
volatile bool left_debug_flag = false;
volatile bool right_debug_flag = false;

// Variables to store encoder states for debugging
volatile uint8_t left_last_state = 0;
volatile uint8_t left_current_state = 0;
volatile uint8_t right_last_state = 0;
volatile uint8_t right_current_state = 0;


/* Interrupt routine for LEFT encoder, taking care of actual counting */
void leftEncoderISR() {
    if  (digitalRead(LEFT_ENC_PIN_B) == HIGH) {
      // clockwise rotation
      left_enc_pos++;
    } else {
      //counter-clockwise rotation
      left_enc_pos--;    
    } 
}

void rightEncoderISR() {
    if  (digitalRead(RIGHT_ENC_PIN_B) == HIGH) {
      // clockwise rotation
      right_enc_pos++;
    } else {
      //counter-clockwise rotation
      right_enc_pos--;    
    } 
}


/* Wrap the encoder reading function */
long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
    if (i == LEFT) {
        left_enc_pos = 0L;
    } else {
        right_enc_pos = 0L;
    }
}

/* Wrap the encoder reset function for both encoders */
void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
}

#else
    #error An encoder driver must be selected!
#endif

#endif
