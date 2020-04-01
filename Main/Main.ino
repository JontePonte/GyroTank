

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID:s
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_z = 0.8;               // Gain setting for the turn P-controller
float pid_i_gain_z = 0.02;              // Gain setting for the turn I-controller
float pid_d_gain_z = 1.5;               // Gain setting for the turn D-controller
int pid_max_z = 400;                    // Maximum output of the PID-controller (+/-)

float pid_p_gain_y = 0.0;               // Gain setting for the pitch P-controller
float pid_i_gain_y = 0.0;               // Gain setting for the pitch I-controller
float pid_d_gain_y = 0.0;               // Gain setting for the pitch D-controller
int pid_max_y = 200;                    // Maximum output of the PID-controller (+/-)
int pid_y_throttle_deadband = 100;      // Deadband on throttle (channel 3) for PID-y
int pid_y_gyro_deadband = 20;           // Deadband on gyro pitch for PID-y

float pid_i_term_relax = 0.9;           // Softening I-term while pid_error is close to zero

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tank varables
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float max_turn_rate = 360;              // Max turn rate
int esc_twitch_compensation = 23;       // Compensate for positiv twitch in esc signals


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "Wire.h" // This library allows you to communicate with I2C devices.


////////// Declare receiver variables //////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time; 


////////// Declare gyro variables //////////
const int MPU_ADDR = 0x68;                                                  // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t accelerometer_x, accelerometer_y, accelerometer_z;                  // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                                             // variables for gyro raw data
float accelerometer_input_x, accelerometer_input_y, accelerometer_input_z;  // vatiables for filtered accelerometer data
float gyro_input_x, gyro_input_y, gyro_input_z;                             // variables for gyro filtered data in deg/s
int16_t temperature;                                                        // variables for temperature data
char tmp_str[7];                                                            // temporary variable used in convert function
double gyro_axis_cal_x, gyro_axis_cal_y, gyro_axis_cal_z;                   // Gyro calibration value
double acc_axis_cal_x, acc_axis_cal_y, acc_axis_cal_z;                      // Accelerometer calibration value
int cal_int;                                                                // Gyro calibration counter


////////// Declare PID variables //////////
float pid_setpoint_z, pid_output_z;                                         // variables for PID in- & output
float pid_i_mem_z, pid_last_z_d_error;                                      // variables for PID-controller

float pid_setpoint_y, pid_output_y;                                         // variables for PID in- & output
float pid_i_mem_y, pid_last_y_d_error;                                      // variables for PID-controller

float pid_error_temp;                                                       // variable that stores PID-error


////////// Declare esc varables //////////
int throttle, esc_1, esc_2, esc_3, esc_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;
unsigned long esc_loop_timer;
unsigned long loop_timer;         


////////// Declare other global varables //////////
int is_setup;       // Variable to keep track if the program is in setup or loop
int loop_mode;      // 0 = avstängd, 1 = normal, 2 = no gyro


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  is_setup = 1;
  Serial.begin(9600);
  TWBR = 12;

  ////////// Reciever Setup //////////
  // Prepare the interupts for input pin 8 tp 11
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change6


  ////////// Gyro Setup //////////
  // Initialice the gyro
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(250); // Give the gyro time to start

  //Let's take multiple gyro and accelerometer data samples so we can determine the average offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    read_gyro_values();                                                     //Read the gyro output. is_setup disable gyro calibration
    gyro_axis_cal_x += gyro_x;                                              //Ad roll value to gyro_roll_cal.
    gyro_axis_cal_y += gyro_y;                                              //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal_z += gyro_z;                                              //Ad yaw value to gyro_yaw_cal.
    
    acc_axis_cal_x += accelerometer_x;
    acc_axis_cal_y += accelerometer_y;
    acc_axis_cal_z += accelerometer_z;
    delay(3);
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal_x /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal_y /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal_z /= 2000;                                                 //Divide the yaw total by 2000.

  acc_axis_cal_x /= 2000;                                                  //Divide the roll total by 2000.
  acc_axis_cal_y /= 2000;                                                  //Divide the pitch total by 2000.
  acc_axis_cal_z /= 2000;                                                  //Divide the yaw total by 2000.


  ////////// ESC Setup //////////
  DDRD |= B11110000;          // Set pin 4,5,6,7 as output (in a faster way than pinmode)
  loop_timer = micros();      // Set loop_timer for the next loop
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;
  
  is_setup = 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  loop_mode = 0;  // base mode is all motors off

  ////////// Reciever Loop //////////

  // print_receiver_signals();

  // The PID set point in degrees per second is determined by the z receiver input.
  pid_setpoint_z = 0;
  // We need a little dead band of 16us for better results.
  if(receiver_input_channel_4 > 1508)pid_setpoint_z = receiver_input_channel_4 - 1508;
  else if(receiver_input_channel_4 < 1492)pid_setpoint_z = receiver_input_channel_4 - 1492;

  // Max turn rate is set to deg by max_turn_rate / (500 - 8)
  pid_setpoint_z *= (max_turn_rate / 492);

  // The tankt does not want to wheeele
  pid_setpoint_y = 0;


  ////////// Gyro Loop //////////
  read_gyro_values();   // This is done while the esc-pulse is created instead

  // Transform the gyro to deg/s and filter it
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_input_x = (gyro_input_x * 0.7) + ((gyro_x / 65.5) * 0.3);       //Gyro pid input is deg/sec.
  gyro_input_y = (gyro_input_y * 0.7) + ((gyro_y / 65.5) * 0.3);       //Gyro pid input is deg/sec.
  gyro_input_z = (gyro_input_z * 0.7) + ((gyro_z / 65.5) * 0.3);       //Gyro pid input is deg/sec.

  // Filter accelerometer data
  accelerometer_input_x = (accelerometer_input_x * 0.8) + (accelerometer_x * 0.2);    
  accelerometer_input_y = (accelerometer_input_y * 0.8) + (accelerometer_y * 0.2);    
  accelerometer_input_z = (accelerometer_input_z * 0.8) + (accelerometer_z * 0.2);    

  //print_gyro_signals();             // print gyro (filtered input values)
  //print_accelerometer_signals();    


  ////////// PID Loop //////////
  calculate_pid();                                                        //PID inputs are known. So we can calculate the pid output.


  ////////// Mode switch //////////

  // The esc:s only run if throttle is up
  if(receiver_input_channel_1 > 1020) loop_mode = 1;

  // The esc:s don't move if the tank is upside down  (gyro z goes crazy)
  if(abs(accelerometer_input_x) > 10000) loop_mode = 0;
  
  
  ////////// ESC Loop //////////
  
  throttle = receiver_input_channel_3;                                    //We need the throttle signal as a base signal.

  esc_1 = receiver_input_channel_1;
  esc_2 = receiver_input_channel_2;
  esc_3 = throttle + pid_output_z - pid_output_y;
  esc_4 = throttle - pid_output_z - pid_output_y;

  if (esc_1 < 1000) esc_1 = 1000;                                           //Minimum esc-1 pulse is 1000us.
  if (esc_2 < 1000) esc_2 = 1000;                                           //Minimum esc-2 pulse is 1000us.
  if (esc_3 < 1000) esc_3 = 1000;                                           //Minimum esc-3 pulse is 1000us.
  if (esc_4 < 1000) esc_4 = 1000;                                           //Minimum esc-4 pulse is 1000us.

  if (esc_1 > 2000) esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
  if (esc_2 > 2000) esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
  if (esc_3 > 2000) esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
  if (esc_4 > 2000) esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  

  // Kill switch with if loop_mode == 0
  if(loop_mode == 0) {
    esc_1 = 1500;
    esc_2 = 1500;
    esc_3 = 1500;
    esc_4 = 1500;
  }

  // Compensate for esc twitch because of positiv noise in esc signals
  if (esc_1 > (1000 + esc_twitch_compensation)) esc_1 -= esc_twitch_compensation;
  if (esc_2 > (1000 + esc_twitch_compensation)) esc_2 -= esc_twitch_compensation;
  if (esc_3 > (1000 + esc_twitch_compensation)) esc_3 -= esc_twitch_compensation;
  if (esc_4 > (1000 + esc_twitch_compensation)) esc_4 -= esc_twitch_compensation;

  //print_esc_values();

  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  //Serial.println(float((micros() - loop_timer)) / float(4000));           // Print the processor load
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.
  
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  //read_gyro_values();   // (It is to slow for now... Check gyro video again)

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                The receiver interupt bit
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(PCINT0_vect){                                                           // Initialise the interupt
  current_time = micros();
  // Channel 1
  if(PINB & B00000001){                                                     // Is input 8 high?
    if(last_channel_1 == 0){                                                // Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   // Remember current input state.
      timer_1 = current_time;                                               // Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             // Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     // Remember current input state.
    receiver_input_channel_1 = current_time - timer_1;                      // Channel 1 is current_time - timer_1.
  }
  // Channel 2
  if(PINB & B00000010 ){                                                    // Is input 9 high?
    if(last_channel_2 == 0){                                                // Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   // Remember current input state.
      timer_2 = current_time;                                               // Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             // Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     // Remember current input state.
    receiver_input_channel_2 = current_time - timer_2;                      // Channel 2 is current_time - timer_2.
  }
  // Channel 3
  if(PINB & B00000100 ){                                                    // Is input 10 high?
    if(last_channel_3 == 0){                                                // Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   // Remember current input state.
      timer_3 = current_time;                                               // Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             // Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     // Remember current input state.
    receiver_input_channel_3 = current_time - timer_3;                      // Channel 3 is current_time - timer_3.

  }
  // Channel 4
  if(PINB & B00001000 ){                                                    // Is input 11 high?
    if(last_channel_4 == 0){                                                // Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   // Remember current input state.
      timer_4 = current_time;                                               // Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             // Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     // Remember current input state.
    receiver_input_channel_4 = current_time - timer_4;                      // Channel 4 is current_time - timer_4.
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                The gyro reading
//                Needs fixing, the gyro is twitching
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_gyro_values() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                         // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);              // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true);    // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_y = Wire.read()<<8 | Wire.read();   // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read();   // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_x = Wire.read()<<8 | Wire.read();   // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read();       // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read();            // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read();            // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read();            // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  // Make sure that the gyro movement follows the standard aircraft rotation
  gyro_x *= 1;                                      // -1 inverts the signal
  gyro_y *= -1;                                     // -1 inverts the signal
  gyro_z *= 1;                                      // -1 inverts the signal
  
  if(is_setup == 0) {                               // Do not center the data during the setup calibration
    gyro_x -= gyro_axis_cal_x;                      // Set the centerpoint to zero
    gyro_y -= gyro_axis_cal_y;
    gyro_z -= gyro_axis_cal_z;

    accelerometer_x -= acc_axis_cal_x;
    accelerometer_y -= acc_axis_cal_y;
    accelerometer_z -= acc_axis_cal_z;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                Calculate PID
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  
  ////////// z (turn) calculations //////////
  pid_error_temp = gyro_input_z - pid_setpoint_z;       // Calculate diff between gyro and receiver input

  // I-term calculation
  pid_i_mem_z += pid_i_gain_z * pid_error_temp;                         // Save I-term memory
  if(pid_i_mem_z > pid_max_z) pid_i_mem_z = pid_max_z;                  // I-term max is pid_max
  else if(pid_i_mem_z < pid_max_z * -1) pid_i_mem_z = pid_max_z * -1;   // I-term min is -pid_max

  // I-term relax if pid_error is zero
  if(abs(pid_error_temp) < 0.5) pid_i_mem_z *= pid_i_term_relax;

  // PID-calculation
  pid_output_z = pid_p_gain_z * pid_error_temp + pid_i_mem_z + pid_d_gain_z * (pid_error_temp - pid_last_z_d_error);
  
  if(pid_output_z > pid_max_z) pid_output_z = pid_max_z;                 // Max PID output is pid_max
  else if(pid_output_z < pid_max_z * -1) pid_output_z = pid_max_z * -1;  // Min PID output is -pid_max

  // Save error so D-term can calculate change next cycle
  pid_last_z_d_error = pid_error_temp;


  ////////// y (pitch) calculations //////////
  pid_error_temp = 0;
  // Add a little deadband on the gyro signal
  if(gyro_input_y >  pid_y_gyro_deadband) pid_error_temp = gyro_input_y - pid_setpoint_y - pid_y_gyro_deadband;
  if(gyro_input_y < -pid_y_gyro_deadband) pid_error_temp = gyro_input_y - pid_setpoint_y + pid_y_gyro_deadband;
  
  // Only activate PID-y if throttle is over 1600 or under 1400 ms
  if(abs(receiver_input_channel_3 - 1500) >= pid_y_throttle_deadband) pid_error_temp = 0;
  
  // I-term calculation
  pid_i_mem_y += pid_i_gain_y * pid_error_temp;                         // Save I-term memory
  if(pid_i_mem_y > pid_max_y) pid_i_mem_z = pid_max_y;                  // I-term max is pid_max
  else if(pid_i_mem_y < pid_max_y * -1) pid_i_mem_y = pid_max_y * -1;   // I-term min is -pid_max

  // I-term relax if pid_error is zero
  if(abs(pid_error_temp) < 0.5) pid_i_mem_y *= pid_i_term_relax;

  // PID-calculation
  pid_output_y = pid_p_gain_y * pid_error_temp + pid_i_mem_y + pid_d_gain_y * (pid_error_temp - pid_last_y_d_error);
  
  if(pid_output_y > pid_max_z) pid_output_y = pid_max_y;                 // Max PID output is pid_max
  else if(pid_output_y < pid_max_y * -1) pid_output_y = pid_max_y * -1;  // Min PID output is -pid_max

  // Save error so D-term can calculate change next cycle
  pid_last_y_d_error = pid_error_temp;

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                Print the receiver input values
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void print_receiver_signals() {
  Serial.print("Thr:     ");
  Serial.print(receiver_input_channel_1);
  
  Serial.print("     Yaw:     ");
  Serial.print(receiver_input_channel_2);

  Serial.print("     Ail:     ");
  Serial.print(receiver_input_channel_3);

  Serial.print("     Ele:     ");
  Serial.println(receiver_input_channel_4);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                Print the gyro values
//                The gyro_input_xyz is the filtered value converted to deg/s   (raw values = gyro_xyz)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void print_gyro_signals() {
  Serial.print("x:     ");
  Serial.print(gyro_input_x);
  
  Serial.print("     y:     ");
  Serial.print(gyro_input_y);

  Serial.print("     z:     ");
  Serial.println(gyro_input_z);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                Print the accelerometer values
//                The accelerometer_input_xyz is filtered
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void print_accelerometer_signals() {
  Serial.print("x:     ");
  Serial.print(accelerometer_input_x);
  
  Serial.print("     y:     ");
  Serial.print(accelerometer_input_y);

  Serial.print("     z:     ");
  Serial.println(accelerometer_input_z);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                Print the esc values
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void print_esc_values() {
  Serial.print("1:     ");
  Serial.print(esc_3);
  
  Serial.print("     2:     ");
  Serial.println(esc_4);
}
