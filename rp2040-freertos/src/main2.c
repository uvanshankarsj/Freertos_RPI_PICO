// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Servo.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"


// Adafruit_MPU6050 mpu;
// Servo standservo1;
// Servo standservo2;
// Servo gripper;
// Servo pan;
// Servo tilt;

#define D0 2
#define D1 3
#define D2 4
#define D3 5
#define LED_PIN 25
#define GPIO_ON     1
#define GPIO_OFF    0
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1


int pan_pos = 90;

int tilt_pos = 90;

int self_count = 0;

int motorDirection = 0;
double setpoint = 0;
double Kp = 10.532;
double Ki = 18.338;
double Kd = 1.077;

double error, lastError = 0;
double P, I, D;
double output;

// MPU6050 offsets
int accelXOffset = 0;
int accelYOffset = 0;
int accelZOffset = 0;
int gyroXOffset = 0;
int gyroYOffset = 0;
int gyroZOffset = 0;


// Tolerance range for the error
double errorTolerance = 1.0;

void setup() {
  // Serial.begin(9600);
  uart_init(uart0,9600);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  // while (!Serial)
    // delay(10); // will pause Zero, Leonardo, etc until serial console opens

  uart_puts(UART_ID, "Adafruit MPU6050 test!");

  // Try to initialize!
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
  uart_puts(UART_ID, "MPU6050 Found!");

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   Serial.print("Accelerometer range set to: ");
//   switch (mpu.getAccelerometerRange()) {
//     case MPU6050_RANGE_2_G:
//       Serial.println("+-2G");
//       break;
//     case MPU6050_RANGE_4_G:
//       Serial.println("+-4G");
//       break;
//     case MPU6050_RANGE_8_G:
//       Serial.println("+-8G");
//       break;
//     case MPU6050_RANGE_16_G:
//       Serial.println("+-16G");
//       break;
//   }
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   Serial.print("Gyro range set to: ");
//   switch (mpu.getGyroRange()) {
//     case MPU6050_RANGE_250_DEG:
//       Serial.println("+- 250 deg/s");
//       break;
//     case MPU6050_RANGE_500_DEG:
//       Serial.println("+- 500 deg/s");
//       break;
//     case MPU6050_RANGE_1000_DEG:
//       Serial.println("+- 1000 deg/s");
//       break;
//     case MPU6050_RANGE_2000_DEG:
//       Serial.println("+- 2000 deg/s");
//       break;
//   }

//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//   Serial.print("Filter bandwidth set to: ");
//   switch (mpu.getFilterBandwidth()) {
//     case MPU6050_BAND_260_HZ:
//       Serial.println("260 Hz");
//       break;
//     case MPU6050_BAND_184_HZ:
//       Serial.println("184 Hz");
//       break;
//     case MPU6050_BAND_94_HZ:
//       Serial.println("94 Hz");
//       break;
//     case MPU6050_BAND_44_HZ:
//       Serial.println("44 Hz");
//       break;
//     case MPU6050_BAND_21_HZ:
//       Serial.println("21 Hz");
//       break;
//     case MPU6050_BAND_10_HZ:
//       Serial.println("10 Hz");
//       break;
//     case MPU6050_BAND_5_HZ:
//       Serial.println("5 Hz");
//       break;
//   }
  uart_puts(UART_ID, "");
//   delay(100);
//   pinMode(D0, OUTPUT);
//   pinMode(D1, OUTPUT);
//   pinMode(D2, OUTPUT);
//   pinMode(D3, OUTPUT);
//   standservo1.attach(11);
//   standservo2.attach(12); 
//   gripper.attach(10);
//   pan.attach(8);
//   tilt.attach(9); 
//   pan.write(90);
//   tilt.write(90);
//   standservo1.write(0);
//   standservo2.write(0);
//   Serial.begin(9600);
}

void control(){
if (1) {
    char command = 2;
    // print(command);
    /*
    stand_state = Serial.read();
    gripper_state = Serial.read();
    pan_state = Serial.read();
    tilt_state = Serial.read();
    */
    if (command=='1')
    {
      // Serial.print("motor_state:");
      uart_putc(UART_ID, 'w');
    //   analogWrite(D0, 50);
    //   digitalWrite(D1, LOW);
    //   analogWrite(D2, 50);
    //   digitalWrite(D3, LOW);   
    }
    if (command=='2')
    {
      // Serial.print("motor_state:");
      uart_putc(UART_ID, 's');
    //   digitalWrite(D0, LOW);
    //   analogWrite(D1,50);
    //   digitalWrite(D2, LOW);
    //   analogWrite(D3,50); 
    }
    if (command=='3')
    {
      // Serial.print("motor_state:");
      uart_putc(UART_ID, 'a');
    //   analogWrite(D0,50);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,0);
    //   digitalWrite(D3,LOW);
    }
    if (command=='4')
    {
      // Serial.print("motor_state");
      uart_putc(UART_ID, 'd');
    //   analogWrite(D0,0);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,50);
    //   digitalWrite(D3,LOW);
    }
    if (command=='a')
    {
      // Serial.print("stand_state:");
      uart_puts(UART_ID, "on");
    //   gripper.write(90);
    }
    if (command=='b')
    {
    //   analogWrite(D0,0);
    //   digitalWrite(D1,LOW);
    //   analogWrite(D2,0);
    //   digitalWrite(D3,LOW);

    }
    if (command=='6')
    {
      // Serial.print("gripper_state:");
      uart_puts(UART_ID, "cw");    
    //   gripper.write(45);
    }
    else if(command=='7')
    {
      // Serial.print("gripper_state:");
      uart_puts(UART_ID, "ccw");
    //   gripper.write(135);
    }
    if (command=='8'){
    //   pan_pos=pan_pos+1;
    //   if(pan_pos>180){
    //     pan_pos=180;
    //   }
      uart_puts(UART_ID,"pan_pos:");
    //   Serial.println(pan_pos);
    //   pan.write(pan_pos);
    }
    else if (command=='9')
    {
    //   pan_pos=pan_pos-1;
    //   if(pan_pos<0)
    //   {
    //     pan_pos=0;
    //   }
      uart_puts(UART_ID,"pan_pos:");
    //   Serial.println(pan_pos);
    //   pan.write(pan_pos);
    }
    if (command=='0')
    {
    //   tilt_pos = tilt_pos+1;
    //   if(pan_pos>180){
    //     tilt_pos=180;
    //   }
      uart_puts(UART_ID,"tilt_pos:");
    //   Serial.println(tilt_pos);
    //   tilt.write(tilt_pos);
    }
    if (command=='5')
    {
    //   tilt_pos = tilt_pos-1;
    //   if(tilt_pos<0)
    //   {
    //     tilt_pos=0;
    //   }
      uart_puts(UART_ID,"tilt_pos:");
    //   Serial.println(tilt_pos);
    //   tilt.write(tilt_pos);
    }
    // tilt.write(tilt_pos);
    if (command=='c')
    {
      self_count+=1; 
    }
    if (command=='d')
    {
      self_count=0;
    }
    if(self_count>0)
    {
      //Serial.println("Self balancing enabled");
      if (motorDirection == 1) {
        uart_puts(UART_ID, "forward");
        // analogWrite(D0, 60);
        // analogWrite(D1, 0);
        // analogWrite(D2, 60);
        // analogWrite(D3, 0);
      } else if (motorDirection == -1) {
        uart_puts(UART_ID, "backward");
        // analogWrite(D0, 0);
        // analogWrite(D1, 60);
        // analogWrite(D2, 0);
        // analogWrite(D3, 60);
      }
      else{
        uart_puts(UART_ID, "else");
        // analogWrite(D0, 0);
        // analogWrite(D1, 0);
        // analogWrite(D2, 0);
        // analogWrite(D3, 0);
        }
    }
    if(self_count==0){
      uart_puts(UART_ID, "Self balancing disabled");
    //   analogWrite(D0, 0);
    //   analogWrite(D1, 0);
    //   analogWrite(D2, 0);
    //   analogWrite(D3, 0);
    }
}
}

void balance(){
    gpio_put(LED_PIN, GPIO_ON);
    vTaskDelay(1000);
    gpio_put(LED_PIN, GPIO_OFF);
    vTaskDelay(1000);
// int accelX = a.acceleration.x - accelXOffset;
// int accelY = a.acceleration.y - accelYOffset;
// int accelZ = a.acceleration.z - accelZOffset;
// int gyroX = g.gyro.x - gyroXOffset;
// int gyroY = g.gyro.y - gyroYOffset;
// int gyroZ = g.gyro.z - gyroZOffset;

// double angle = atan2(accelY, accelZ) * 180 / PI;
// double gyroAngle = angle + (gyroX / 131.0) * 0.98;
// error = setpoint - gyroAngle;
//   P = Kp * error;
//   I += Ki * error;
//   D = Kd * (error - lastError);
//   output = P + I + D;
//   lastError = error;
  
//   standservo1.write(0);
//   standservo2.write(0);
//   pan.write(80);
  
  // Check if the error is within the tolerance range
//   if (fabs(error) < errorTolerance) {
//     output = 0.0;  // Set the output to zero
//     I = 0.0;      // Reset the integral term
//   }

//   if (output > 255.00) {
//     output = 255.00;
//   } else if (output < -255.00) {
//     output = -255.00;
//   }
//   int motorSpeed = abs(output);  // Map the absolute output to the motor speed range
//   if (output < 0.00) {
//     motorDirection = -1;
//   } else if(output==0){
//     motorDirection = 0;
//   }
//   else{
//     motorDirection = 1;
//   }
//    if(self_count>0)
//     {
//       //Serial.println("Self balancing enabled");
//       if (motorDirection == 1) {
//         analogWrite(D0, 60);
//         analogWrite(D1, 0);
//         analogWrite(D2, 60);
//         analogWrite(D3, 0);
//       } else if (motorDirection == -1) {
//         analogWrite(D0, 0);
//         analogWrite(D1, 60);
//         analogWrite(D2, 0);
//         analogWrite(D3, 60);
//       }
//       else{
//         analogWrite(D0, 0);
//         analogWrite(D1, 0);
//         analogWrite(D2, 0);
//         analogWrite(D3, 0);
//         }
//     }
//     if(self_count==0){
//       //Serial.println("Self balancing disabled");
//       analogWrite(D0, 0);
//       analogWrite(D1, 0);
//       analogWrite(D2, 0);
//       analogWrite(D3, 0);
//     }

  }

void loop(){
    //xTaskCreate(callfunction,ourfunctionname,no of varialbe,pointer which will be used as the parameter for the task created,priority,handel by which the created task cn be reffered)
    
    uint32_t status =xTaskCreate(control, "LED_0", 128, NULL, 0, NULL);
	uint32_t status2=xTaskCreate(balance, "LED_1", 128, NULL, 0, NULL);
    
    vTaskStartScheduler();
}