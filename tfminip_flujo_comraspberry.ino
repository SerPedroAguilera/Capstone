#include <SoftwareSerial.h>  //header file of software serial port

SoftwareSerial Serial1(6,7); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX

int dist;  //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
int check;  //save check value
int i;
int uart[9];  //save data measured by LiDAR
const int HEADER=0x59;  //frame header of data package

volatile int flow_frequency; // Measures flow sensor pulses
unsigned int l_hour; // Calculated litres/hour
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;

int motor1pin1 = 11;
int motor1pin2 = 12;

void flow () // Interrupt function
{
flow_frequency++;
}

void setup() {
  Serial.begin(19200); //set bit rate of serial port connecting Arduino with computer
  Serial1.begin(115200);  //set bit rate of serial port connecting LiDAR with Arduino

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() { 
  if (Serial1.available()) {  //check if serial port has data input
    if(Serial1.read() == HEADER) {  //assess data package frame header 0x59
      uart[0]=HEADER;
      if (Serial1.read() == HEADER) { //assess data package frame header 0x59
        uart[1] = HEADER;
        for (i = 2; i < 9; i++) { //save data in array
          uart[i] = Serial1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
          dist = uart[2] + uart[3] * 256;     //calculate distance value
          strength = uart[4] + uart[5] * 256; //calculate signal strength value

          currentTime = millis();
          // Every second, calculate and print litres/hour
          if(currentTime >= (cloopTime + 1000))
          {
          cloopTime = currentTime; // Updates cloopTime

          // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
          l_hour = (flow_frequency * 60 / 7.5);
          // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour

          flow_frequency = 0; // Reset Counter
          Serial.print(l_hour, DEC); // Print litres/hour
          Serial.print(" L/hour,");
          Serial.print("dist=");
          Serial.print(dist); //output measure distance value of LiDAR
          Serial.print(',');
          Serial.print("strength=");
          Serial.print(strength); //output signal strength value
          Serial.print(',');
          if (Serial.available() > 0) {
           String data = Serial.readStringUntil('\n');
           digitalWrite(motor1pin1, HIGH);
           digitalWrite(motor1pin2, LOW);
           analogWrite(9,data.toInt());
           Serial.print("Velocidad Bomba: ");
           Serial.println(data);
          }
          }
        }
      }
    }
  }
}
