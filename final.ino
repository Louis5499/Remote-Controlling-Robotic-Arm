#include <Servo.h>
#include <SoftwareSerial.h>

// Global
unsigned long currentMillis = 0;
unsigned long stopMillis = 0;
const int PER_DELAY_TIME = 60;
const int debounceDelay = 50;
// ---

// MSP430 Definition
SoftwareSerial mspSerial(16,17);
// ---


// Bluetooth Definition
const int RX_PIN = 10;
const int TX_PIN = 11;
SoftwareSerial BTSerial(RX_PIN, TX_PIN); // RX | TX
// ---

// Servo Definition
Servo myservo[4];
/// motor
// 0 -> 底座向前
// 1 -> 爪
// 2 -> 上座向前
// 3 -> 底部旋轉
const int servoPort[4] = { 6, 7, 8, 9 };
int currentAngle[4] = { 0 };
const int UPPER_BOUND = 2400;
const int LOWER_BOUND = 500;
int perMovement = 100;
int moveForwardRunningMotor = 0; // 確定向前伸展的馬達，目前呼叫是哪一個
// ---

// JoySticks Definition
int joyStickButton = 5;
int xAxis = A0, yAxis = A1;
int joyButtonState;
int joyLastButtonState = LOW;
unsigned long joyLastDebounceTime = 0;
// ---

// Ultra Definition
const int trigPin = 13, echoPin = 12;
const int AVAILABLE_DISTANCE_LOW = 0;
const int AVAILABLE_DISTANCE_HIGH = 40;
long duration, distance, lastDistance;
// ---

// Button Definition
// 0 -> Ultra Open
// 1 > MSP / BT Serial switch
int buttonPins[2] = {4, 18};
bool isUltraOpen = false;
bool isBluetoothOpen = false;
int buttonStates[2] = { LOW };
int lastButtonStates[2] = { LOW };
unsigned long lastDebounceTimes[2] = { 0 };
// ---

// LED Definition
// 0 -> move forward status
// 1 -> ultra open
// 2 -> serial switch
const int ledPins[3] = { 2, 3, 19 };
// ---

void setup() {
  Serial.begin(9600);
  pinMode(joyStickButton, INPUT_PULLUP); //return LOW when down
  for (int i=0;i<2;i++){
    pinMode(buttonPins[i], INPUT);
  }
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  for (int i=0;i<4;i++) {
    myservo[i].attach(servoPort[i], 500, 2400); // 修正脈衝寬度範圍
    myservo[i].write(90); // 一開始先置中90度
    currentAngle[i]= (UPPER_BOUND + LOWER_BOUND) / 2;
  }

  for (int i=0;i<3;i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  BTSerial.begin(9600);  // HC-06 current bound rate (default 9600)
  mspSerial.begin(9600);
  mspSerial.print("o");
  mspSerial.print("c");
  mspSerial.println();
  delay(1000);
}

void increaseMotor(int motorNum) {
    if (currentAngle[motorNum] + perMovement <= UPPER_BOUND) {
      currentAngle[motorNum] += perMovement;
    } else {
      currentAngle[motorNum] = UPPER_BOUND;
    }
    myservo[motorNum].writeMicroseconds(currentAngle[motorNum]);
}

void decreaseMotor(int motorNum) {
    if (currentAngle[motorNum] - perMovement >= LOWER_BOUND) {
      currentAngle[motorNum] -= perMovement;
    } else {
      currentAngle[motorNum] = LOWER_BOUND;
    }
     myservo[motorNum].writeMicroseconds(currentAngle[motorNum]);
}

void moveFrontBack(int pos) {
  // pos: 0 -> forward, 1 -> backward
  // Note: motor 2 need to reverse direction
  bool shouldIncrease = (moveForwardRunningMotor == 0 && pos == 0) || (moveForwardRunningMotor == 2 && pos == 1);
  if (shouldIncrease) increaseMotor(moveForwardRunningMotor);
  else decreaseMotor(moveForwardRunningMotor);
}

void ultraSensor() {
   digitalWrite(trigPin, LOW); // Clears the trigPin
   delayMicroseconds(2);
   /* Sets the trigPin on HIGH state for 10 ms */
   digitalWrite(trigPin, HIGH);    delayMicroseconds(10);
   digitalWrite(trigPin, LOW);
   /* Reads Echo pin, returns sound travel time in ms */
   duration = pulseIn(echoPin, HIGH);
   /* Calculating the distance */
   distance = duration*0.034/2;

   if (distance >= AVAILABLE_DISTANCE_LOW && distance <= AVAILABLE_DISTANCE_HIGH) {
    // Only distance when it's reasonable
    if (lastDistance - distance < 0) {
      // which means become closer
      decreaseMotor(1);
    } else {
      // which means become farer
      increaseMotor(1);
    }
    Serial.println(lastDistance);
    Serial.println(distance);
    Serial.println(lastDistance - distance < 0 ? "positive" : "negtive");
    lastDistance = distance;
   }
   
   Serial.print("Dis: ");
   Serial.println(distance);
}

void ultraButtonDetect() {
  int reading = digitalRead(buttonPins[0]);
  if (reading != lastButtonStates[0]) {
    lastDebounceTimes[0] = millis();
  }
  if ((millis() - lastDebounceTimes[0]) > debounceDelay) {
    if (reading != buttonStates[0]) {
      buttonStates[0] = reading;

      if (buttonStates[0]) {
        isUltraOpen = !isUltraOpen;
      }
    }
  }
  lastButtonStates[0] = reading;
  analogWrite(ledPins[1], isUltraOpen ? 100 : 0);
}


void serialButtonDetect() {
  int reading = digitalRead(buttonPins[1]);
  if (reading != lastButtonStates[1]) {
    lastDebounceTimes[1] = millis();
  }
  if ((millis() - lastDebounceTimes[1]) > debounceDelay) {
    if (reading != buttonStates[1]) {
      buttonStates[1] = reading;

      if (buttonStates[1]) {
        if (isBluetoothOpen) {
          mspSerial.listen();
          isBluetoothOpen = false;
        } else {
          BTSerial.listen();
          isBluetoothOpen = true;
        }
      }
    }
  }
  lastButtonStates[1] = reading;
  analogWrite(ledPins[2], isBluetoothOpen ? 100 : 0);
}

void changeMovingForwardMotor() {
  if (moveForwardRunningMotor == 0) {
    moveForwardRunningMotor = 2;
    analogWrite(ledPins[0], 100);  
  } else {
    moveForwardRunningMotor = 0;
    analogWrite(ledPins[0], 0);  
  }
}

void bluetoothControl() {
  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTSerial.available()) {
    int value = BTSerial.read();
    switch(value) {
      case 49:
        // left
        increaseMotor(3);
        break;
      case 50:
        // up
        moveFrontBack(0);
        break;
      case 51:
        // right
        decreaseMotor(3);
        break;
      case 52:
        // down
        moveFrontBack(1);
        break;
      case 53:
        increaseMotor(1);
        break;
      case 54:
        changeMovingForwardMotor();
        break;
      case 55:
        decreaseMotor(1);
        break;
      case 56:
        break;
    }
    Serial.print(value);
    
  }
   // Keep reading from Arduino Serial Monitor and send to HC-06
//  if (Serial.available()) {
//    BTSerial.print(Serial.read());
//  }
}

void joystickDetect() {
  int xVal = analogRead(xAxis);
  int yVal = analogRead(yAxis);

  if (yVal <= 10) {
    moveFrontBack(0);
  } else if (yVal >= 1020) {
    moveFrontBack(1);
  }

  if (xVal <= 10) {
    increaseMotor(3);
  } else if (xVal >= 1020) {
    decreaseMotor(3);
  }

  int isPress = digitalRead(joyStickButton);
  if (isPress != joyLastButtonState) {
    joyLastDebounceTime = millis();
  }
  if ((millis() - joyLastDebounceTime) > debounceDelay) {
    if (isPress != joyButtonState) {
      joyButtonState = isPress;
      if (joyButtonState == HIGH) {
        changeMovingForwardMotor();
      }
    }
  }
  joyLastButtonState = isPress;
}

void mspProcess() {
  if (mspSerial.available()) {
    char serialData = mspSerial.read();
    Serial.print(serialData);
    if (serialData == 'b') {
      perMovement = (perMovement == 100) ? 200 : 100;
      mspSerial.print("o");
      mspSerial.println();

      if (perMovement == 100) {
        mspSerial.print("c");
        mspSerial.println();
      } else {
        mspSerial.print("w");
        mspSerial.println();
      }
    }
  }
  
}

void loop() {
  currentMillis = millis();
  if (stopMillis < currentMillis) {
    // Execute per 100 millis
    ultraButtonDetect();
    serialButtonDetect();
    bluetoothControl();
    if (isUltraOpen) ultraSensor();
    joystickDetect();
    mspProcess();
    stopMillis = currentMillis + PER_DELAY_TIME;
  }
}
