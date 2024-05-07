#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100]; //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100]; //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
const int buzzerPin = 4; // Pin connected to the buzzer
const int trigPin = 10; // Trigger pin of ultrasonic sensor
const int echoPin = 8; // Echo pin of ultrasonic sensor
const int sampleTime_ms = 100; // Sampling time in milliseconds
const int numSamples = 10; // Number of samples to average heart rate
uint32_t lastTime = 0;
float irValue; // Variable to store raw IR sensor reading
float redValue; // Variable to store raw red sensor reading
const int heartbeatThreshold = 100; // Threshold for heartbeat detection
const int heartbeatmin = 30;

void setup() {
  Serial.begin(115200);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  pinMode(trigPin, OUTPUT); // Set trigger pin as output
  pinMode(echoPin, INPUT); // Set echo pin as input
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeIR(0x0A); // Turn IR LED to low to indicate sensor is running

  Serial.println(F("Attach sensor to finger and press any key to start conversion"));
  while (Serial.available() == 0) ;
  Serial.read();
}

void loop() {
  
  String heartRateOutput = printHeartRate(); // Get heart rate output
  String postureOutput = checkPosture(); // Get posture output
  SpO2();
  // Concatenate SpO2, heart rate, and posture information
  String dataOutput = "SpO2: " + String(spo2) + "\tHeart Rate: " + heartRateOutput + "\tPosture: " + postureOutput;

  // Print all data on the same line
  Serial.println(dataOutput);

  delay(10); // Add delay to avoid rapid serial output
}

void SpO2() {
  bufferLength = 100;

  for (byte i = 0 ; i < bufferLength ; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  while (1) {
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false)
        particleSensor.check();

      digitalWrite(readLED, !digitalRead(readLED));

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();

      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

String printHeartRate() {
  if (millis() - lastTime > sampleTime_ms) {
    lastTime = millis();
    irValue = particleSensor.getIR();
    redValue = particleSensor.getRed();
    float avgValue = 0;
    for (int i = 0; i < numSamples; i++) {
      avgValue += irValue;
      delay(2);
    }
    avgValue /= numSamples;
    float heartBeat = (avgValue * 5.0) / 1024.0;
    heartRate = 60000 / heartBeat * 0.18;
    return String(heartRate); // Return heart rate as a string
  }
  return ""; // Return empty string if not ready to print heart rate
}

String checkPosture() {
  long duration, distance;
  String postureStatus;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  if (distance > 100)
    return "";  // Return empty string when distance is too large
  
  if (distance >= 15) {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    postureStatus = "False";
  } else {
    postureStatus = "True";
  }
  
  return postureStatus;
}