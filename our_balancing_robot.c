#include <Wire.h>

///////////////////////////////////////////////////////////
// Constants
///////////////////////////////////////////////////////////
const int mpu_addr      = 0x68;      // MPU 6050 address
const int calAmount     = 500;      // Amount of times values are taken to calibrate
const int acc_cal_value = 1271;     // Accelerometer standing up value
const float Kp  = 12;               // Gain setting for the P-controller
const float Ki  = 1.1;              // Gain setting for the I-controller
const float Kd  = 25;             // Gain setting for the D-controller

///////////////////////////////////////////////////////////
// Global Variables
///////////////////////////////////////////////////////////
unsigned long loop_timer;
long gyro_Y_cal;
int  gyro_Y_data;
int  acc_data;
int  left_motor, right_motor;
int  left_motor_steps, right_motor_steps;   // how to steps to balance the robot
int  left_motor_cntr, right_motor_cntr;     // keeps track of how many steps it has taken
int  left_motor_mem, right_motor_mem; 		// keeps track of how many steps each motor has left at interrupt
float angle_acc;
float gyro_angle;
float balance_setpoint;
float temp_error, average_mem, setpoint, output, last_error_d; // PID variables
float output_left, output_right;
byte run_robot;

void setup()
{
  Wire.begin();                 // Start I2C bus
  TWBR = 12;                    // Set I2C clock to 400kHz

  //Setup Timer Interrupt TIMER2_COMPA_vect (www.instructables.com/id/Arduino-Timer-Interrupts/)
  TCCR2A = 0;                   // Set register to 0
  TCCR2B = 0;                   // Set register to 0
  TCNT2 = 0;                    // counter value to 0
  OCR2A = 74;                   // The compare register is set to 74 = [16MHz / (26.6kHz * 8)] - 1 = 37.5uS
  TCCR2A |= (1 << WGM21);       // Set counter 2 to CTC (clear timer on compare) mode
  TCCR2B |= (1 << CS21);        // Set prescaler to 8 for frequency calculation
  TIMSK2 |= (1 << OCIE2A);      // Enable timer compare interrupt

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x6B);             // Power Management 1 Register
  Wire.write(0x00);             // Set to 0 to wake-up Gyro
  Wire.endTransmission();

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1A);             // Device Configuration Register
  Wire.write(0x03);             // Set to 3 to set Low Pass Filter to ~43Hz (off of datasheet)
  Wire.endTransmission();		// Keeps the Gyro and Accelerometer signals clean

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1B);             // Gyro Configuartion Register
  Wire.write(0x00);             // Set to 0 to get to +/- 250 degrees per sec
  Wire.endTransmission();

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x1C);             // Accelerometer Configuration Register
  Wire.write(0x08);             // Set to 8 to get to +/- 4g
  Wire.endTransmission();

  pinMode(2,  OUTPUT);          // Digital Pin 2 (Left Motor Control)
  pinMode(3,  OUTPUT);          // Digital Pin 3 (Left Motor Direction)
  pinMode(4,  OUTPUT);          // Digital Pin 4 (Right Motor Control)
  pinMode(5,  OUTPUT);          // Digital Pin 5 (Right Motor Direction)
  pinMode(9,  OUTPUT);          // Digital Pin 9 (Used for eyes)
  pinMode(13, OUTPUT);          // Digital Pin 13 (LED)

  digitalWrite(9, LOW);   // Make sure LEDs are off till done calibrating     
  digitalWrite(13, HIGH); // Turn on LED while calibrating

  // Loop to calibrate the Gyro value "calAmount" times
  for (int calGrab = 0; calGrab < calAmount; calGrab++)
  {
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x45);                             // Look at register 0x45 (GYRO_YOUT_H)
    Wire.endTransmission();
    Wire.requestFrom(mpu_addr, 2);                // Reguest 2 register values starting from previously sent spot
    gyro_Y_cal += Wire.read() << 8 | Wire.read(); // Grab Gyro pitch value
    delayMicroseconds(4000);                      // Used to give time for value to change
  }
  gyro_Y_cal /= calAmount;

  digitalWrite(9, HIGH); // LED on for Eyes while running                   
  digitalWrite(13, LOW); // Turn off LED when calibration is done
  loop_timer = micros() + 4000;
}

void loop()
{
  //Calculate Robot Angle
  Wire.beginTransmission(mpu_addr);
  Wire.write(0x3F);                         // Go to Accelerometer Z-Axis Register
  Wire.endTransmission();
  Wire.requestFrom(mpu_addr, 2);            // Read back Accelerometer Z-axis value

  acc_data  = Wire.read() << 8 | Wire.read();
  acc_data += acc_cal_value;

  // Create a min and max to create bounds for angle calculation
  if (acc_data > 8200)
    acc_data = 8200;
  if (acc_data < -8200)
    acc_data = -8200;

  angle_acc = asin((float)acc_data / 8200.0) * (180 / 3.14159); // Calculate radians to degrees

  if (run_robot == 0 && angle_acc > -0.5 && angle_acc < 0.5) // Starting robot running once stood up
  {
    gyro_angle = angle_acc;
    run_robot = 1;
  }

  Wire.beginTransmission(mpu_addr);
  Wire.write(0x45);                         // Look at Gyro_Y values
  Wire.endTransmission();
  Wire.requestFrom(mpu_addr, 2);
  gyro_Y_data = Wire.read() << 8 | Wire.read();

  gyro_Y_data -= gyro_Y_cal;                // Set to 0 stable Gyro/Robot.
  gyro_angle  += gyro_Y_data * 0.0000305;   // Adjustment point for accurate angle

  ///////////////////////////////////////////////////////////
  // PID Controller Start
  ///////////////////////////////////////////////////////////
  temp_error = gyro_angle - balance_setpoint - setpoint;

  if (output > 10 || output < -10)
    temp_error += output * 0.015 ;

  average_mem += Ki * temp_error;

  if (average_mem > 350)
    average_mem = 350;
  else if (average_mem < -350)
    average_mem = -350;

  // Calculate output value
  output = Kp * temp_error + average_mem + Kd * (temp_error - last_error_d);

  if (output > 350)
    output = 350;
  else if (output < -350)
    output = -350;

  last_error_d = temp_error;

  if (output < 25 && output > -25) // Stop center stepper motor jitter
    output = 0;

  // If robot falls over stop motors from spinning
  if (gyro_angle > 30 || gyro_angle < -30 || run_robot == 0)
  {
    output = 0;
    average_mem = 0;
    run_robot = 0;
    balance_setpoint = 0;
  }

  // Control Calculations & set controller output for both motors
  output_left  = output;
  output_right = output;

  if (setpoint == 0)
  {
    if (output < 0)
      balance_setpoint += 0.0015;                   // Stabilize if moving forwards
    if (output > 0)
      balance_setpoint -= 0.0015;                   // Stablize if moving backwards
  }

  // Compensate for the non-linear behaviour of the stepper motors
  if (output_left > 0)
    output_left =  355 - (1 / (output_left + 9)) * 5500;
  else if (output_left < 0)
    output_left = -355 - (1 / (output_left - 9)) * 5500;

  if (output_right > 0)
    output_right =  355 - (1 / (output_right + 9)) * 5500;
  else if (output_right < 0)
    output_right = -355 - (1 / (output_right - 9)) * 5500;

  // Calculate the needed pulse time
  if (output_left > 0)
    left_motor =  350 - output_left;
  else if (output_left < 0)
    left_motor = -350 - output_left;
  else
    left_motor = 0;

  if (output_right > 0)
    right_motor =  350 - output_right;
  else if (output_right < 0)
    right_motor = -350 - output_right;
  else
    right_motor = 0;

  left_motor_steps  = left_motor;
  right_motor_steps = right_motor;

  // Loop time timer
  while (loop_timer > micros());
  loop_timer += 4000;
}

ISR(TIMER2_COMPA_vect)
{
  left_motor_cntr ++;
  right_motor_cntr ++;

  // right motor pulse calculations
  if (right_motor_cntr > right_motor_mem)
  {
    right_motor_cntr = 0;
    right_motor_mem = right_motor_steps;
    if (right_motor_mem < 0)
    {
      PORTD |= 0b00100000;              // Pin 5 low mask for right motor backwards (wanted to be digitalwrite() but it caused problems)
      right_motor_mem *= -1;
    }
    else
      PORTD &= 0b11011111;              // Pin 5 high mask for right motor forward
  }
  else if (right_motor_cntr == 1)
    digitalWrite(4, HIGH);              // PIN 4 HIGH to pulse right motor
  else if (right_motor_cntr == 2)
    digitalWrite(4, LOW);               // Pin 4 LOW to reset pulse for right motor

  // Left motor pulse calculations
  if (left_motor_cntr > left_motor_mem)
  {
    left_motor_cntr = 0;
    left_motor_mem = left_motor_steps;
    if (left_motor_mem < 0)
    {
      digitalWrite(3, LOW);             // Pin 3 low for left motor backwards
      left_motor_mem *= -1;
    }
    else
      digitalWrite(3, HIGH);          // Pin 3 high for left motor forward
  }
  else if (left_motor_cntr == 1)
    digitalWrite(2, HIGH);              // PIN 2 HIGH to pulse left motor
  else if (left_motor_cntr == 2)
    digitalWrite(2, LOW);               // Pin 2 LOW to reset pulse for left motor
}}