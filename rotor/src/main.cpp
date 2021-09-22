/* Author: Rik Starmans
   Organization: Hexastorm
   License: GPL3
   About: polygon is pulsed for PANASONIC (AN44000A chip) scheme, midle pin, pin 3 AKA enable pin
          polygon is rotated and accelerometer data and IR
          sensor data is collected
*/

#include <Wire.h>
// modified fork
#include "SparkFun_MMA8452Q.h" 


// rotor pulsed via pin A3, i.e. pin 0.29 on microcontroller
// #define PWM_PIN 29UL
// not sure about LCK and EN pin, they should be kept fixed!
// #define LCK_PIN A2
// #define EN_PIN A1
// #define irsensorAPin A6
// #define irsensorDPin 2

#define ACCEL_ADDR 0x1C

TwoWire wire1(1, I2C_FAST_MODE);
TwoWire wire2(2, I2C_FAST_MODE);

const int samples = 952;
// pulse frequency != rotor frequency
// 20 hz approx 100 hz for panasonic
// WARNING: if frequency too small COUNTERTOP out of range!!
int frequency = 20;    // Hertz 


MMA8452Q accel1(ACCEL_ADDR); 
MMA8452Q accel2(ACCEL_ADDR);

// TODO: move sample_freqs to LSM9DS1 class
const int sample_freqs[6] = {10, 50, 119, 238, 476, 952};

unsigned int sample_freq;
uint8_t ir_data[samples];
int16_t accel_data[samples];
// used for duty cycle, must be global
uint16_t buf[1]; 


void scan_wire(TwoWire &wire){
          for (byte i = 8; i < 127; i++)
        {
            wire.beginTransmission(i);
            if (wire.endTransmission() == 0)
            {
                Serial.println(String("Found address: " + String(i)));
                // check address
                if (i == 64)
                {
                    // found device, do something..
                }

                delay(1);
            }
        }
        wire.endTransmission();

        Serial.println("I2C device check DONE.");
}  

void accel_setup(TwoWire &wire) {

  // Start I2C Transmission
  wire.beginTransmission(ACCEL_ADDR);
  // Select control register
  wire.write(0x2A);
  // StandBy mode
  wire.write(0x00);
  // Stop I2C Transmission
  wire.endTransmission();
 
  // Start I2C Transmission
  wire.beginTransmission(ACCEL_ADDR);
  // Select control register
  wire.write(0x2A);
  // Active mode
  wire.write(0x01);
  // Stop I2C Transmission
  wire.endTransmission();
 
  // Start I2C Transmission
  wire.beginTransmission(ACCEL_ADDR);
  // Select control register
  wire.write(0x0E);
  // Set range to +/- 2g
  wire.write(0x00);
  // Stop I2C Transmission
  wire.endTransmission();
  delay(300);
}

void setup(){
  

  // pinMode(A3, INPUT);
  // pinMode(LCK_PIN, OUTPUT);
  // pinMode(EN_PIN, OUTPUT);
  // pinMode(irsensorDPin, INPUT);
  // pinMode(irsensorAPin, INPUT);


  Serial.begin(115200);  
  wire1.begin();
  wire2.begin();

  // while (!accel1.begin(wire1, ACCEL_ADDR)) {
  //   Serial.println("Failed to communicate with wire1.");
  //   delay(1000);
  // }


  //  while (!accel2.begin(wire2, ACCEL_ADDR)) {
  //   Serial.println("Failed to communicate with wire2.");

  //   delay(1000);
  // }



  // // sample_freq = sample_freqs[imu.settings.accel.sampleRate-1];
  // // imu.calibrate();

  // accel1.init();
  // accel2.init();

  accel_setup(wire2);

}


void accel_loop(TwoWire &wire){
    unsigned int data[7];
 
  // Request 7 bytes of data
  wire.requestFrom(ACCEL_ADDR, 7);
 
  // Read 7 bytes of data
  // staus, xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
  if(wire.available() == 7) 
  {
    data[0] = wire.read();
    data[1] = wire.read();
    data[2] = wire.read();
    data[3] = wire.read();
    data[4] = wire.read();
    data[5] = wire.read();
    data[6] = wire.read();
  }
 
  // Convert the data to 12-bits
  int xAccl = ((data[1] * 256) + data[2]) / 16;
  if (xAccl > 2047)
  {
    xAccl -= 4096;
  }
 
  int yAccl = ((data[3] * 256) + data[4]) / 16;
  if (yAccl > 2047)
  {
    yAccl -= 4096;
  }
 
  int zAccl = ((data[5] * 256) + data[6]) / 16;
  if (zAccl > 2047)
  {
    zAccl -= 4096;
  }
 
  // Output data to serial monitor
  Serial.print("Acceleration in X-Axis : ");
  Serial.println(xAccl);
  Serial.print("Acceleration in Y-Axis : ");
  Serial.println(yAccl);
  Serial.print("Acceleration in Z-Axis : ");
  Serial.println(zAccl);
  delay(500);
}


void loop() {

  accel_loop(wire2);
  
  //   if(accel1.available()) {
  //     Serial.print("A1: ");
  //     Serial.println(accel1.getZ());
  //   } else {  accel1.init();
  // accel2.init();
  //     Serial.println("accel1 not aviable!");
  //   }


  //   if(accel2.available()) {
  //     Serial.print("A2: ");
  //     Serial.println(accel2.getZ());
  //   } else {
  //     Serial.println("accel 2 not aviable");
  //   }
  //   delay(500);

    // Serial.println("Press 1 to start samples.");
    // Serial.println("Press 2 to calibrate IR sensor.");
    // Serial.println("Press 3 to spin polygon.");
    // Serial.println("Press 4 to check pulse frequency.");
    // Serial.println("Press 5 to set pulse frequency.");
    // Serial.setTimeout(2000);
    // int int_received = Serial.parseInt();
    // switch(int_received) {
    //   // parseInt polls and returns 0 if nothing is received so ignored
    //   case 0 : 
    //   break;
    //   case 1 : {
    //     Serial.setTimeout(500);
    //     Serial.print("Process time ");
    //     Serial.print(round(startup_time+samples/sample_freq));
    //     Serial.println(" seconds.");
    //     // wait for polygon to stabilize
    //     unsigned int iteration = 0;
    //     int tijd = millis();
    //     while(iteration < startup_time*sample_freq){
    //         //  if (imu.accelAvailable())
    //         //     {
    //         //     imu.readAccel();
    //         //     iteration = iteration + 1;
    //         //     }
    //     }
    //     int verschil = millis()-tijd;
    //     if(1.1*verschil<startup_time*1000||startup_time*1000<0.9*verschil){
    //       // frequency accelerometer is probably wrong
    //       Serial.println("Error: time difference out of bounds");
    //     }
    //     // execute measurements
    //     iteration = 0;
    //     int digitalout = 1;
    //     while(iteration<samples){
    //       if(digitalRead(irsensorDPin) == 0) digitalout = 0;
    //       // if (imu.accelAvailable()){
    //       //   //ir_data[iteration] = analogRead(irsensorAPin);
    //       //   ir_data[iteration] = digitalout;
    //       //   imu.readAccel();
    //       //   accel_data[iteration] = imu.ax;
    //       //   digitalout = 1;
    //       //   ++iteration;
    //       // }
    //     }
    //     Serial.print("Pulse frequency ");
    //     Serial.print(frequency);
    //     Serial.println(" Hz.");
    //     Serial.print("Sample frequency ");
    //     Serial.print(sample_freq);
    //     Serial.println(" Hz.");
    //     Serial.print("Samples collected ");
    //     Serial.println(samples);
    //     for(int sample = 0; sample<samples; sample++){
    //       Serial.println(ir_data[sample]);
    //       Serial.println(accel_data[sample]);
    //     }
    //     Serial.println("Measurement completed");
    //   }
    //     break;
    //   case 2 :
    //     Serial.setTimeout(500);
    //     Serial.println("Starting IR calibration, press 1 to exit.");
    //     while(true)
    //     {
    //       if (Serial.parseInt() == 1) break;
    //       Serial.println(analogRead(irsensorAPin));
    //       Serial.println(digitalRead(irsensorDPin));
    //     }
    //     break;
    //   case 3 :
    //     while(true){
    //       Serial.setTimeout(500);
    //       Serial.println("Spinning polygon, press 1 to exit.");
    //       if (Serial.parseInt() == 1){
    //         break;
    //       }
    //     }
    //     break; 
    //   case 4 :{
    //     Serial.setTimeout(1000);
    //     Serial.println("Measuring pulse frequency.");
    //     int old = 0;
    //     int counter = 0;
    //     int current = 1;
    //     float tijd = millis();
    //     while (counter<100)
    //     {
    //      current = digitalRead(A3);
    //      if((current==1)&&(old==0)) counter += 1;
    //      old = current;
    //     }
    //     float frequency = (float)counter/((millis()-tijd)/1000);
    //     Serial.print(frequency);
    //     Serial.println(" Hz");
    //     Serial.println("Pulse test completed.");
    //     break;
    //     }
    //   case 5 :
    //     while (true){
    //       Serial.setTimeout(1000);
    //       frequency = Serial.parseInt();
    //       Serial.println("Set pulse frequency in Hz.");
    //       if (frequency>4){
    //         Serial.println("Accepted.");
    //         break;
    //       }
    //       else if(0<frequency && frequency<4){
    //         Serial.println("Invalid, countertop will overflow.");
    //       }
    //       // ignore 0, default return value of parseInt
    //     }
    //     break;
    //   default:
    //     Serial.println(int_received); 
    //     Serial.println("Invalid command");
    //     break;
    // };
}
