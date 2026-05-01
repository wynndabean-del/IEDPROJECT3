//AIC is to show that this Code wasnt AI copy and pasted, the code has multiple Human added checks to authenticate the code

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// --- Wifi pins ---
SoftwareSerial espSerial(12, 13);

// Software Reset Function
void(* resetFunc) (void) = 0;

// --- WiFi Credentials ---
const char* wifiSSID     = "WIFI NAME";     // ← CHANGE THIS
const char* wifiPassword = "WIFI PASSWORD"; // ← CHANGE THIS

// --- EEPROM Addresses ---
#define ADDR_SPEED        0
#define ADDR_LOG_START   10

// --- Settings ---
#define TURN_SPEED_A     120
#define TURN_SPEED_B     120
#define DEBOUNCE_DELAY   300
#define OBSTACLE_DIST     25
#define HOME_STOP_DIST    10

// --- Pin Definitions ---
#define BATTERY_PIN A5
#define SERVO_PIN   A2
#define BUZZER_PIN  A3
#define touchPin    4
#define scsensor    2
#define IRLeft      A0
#define IRRight     A1
#define Trig1       11
#define Echo1       10
#define Trig2       8
#define Echo2       7
#define IN1         9
#define IN2         6
#define IN3         5
#define IN4         3

LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Global Var ---
int targetLoc = 0, loc = 0;
int motorSpeed;  // ← removed volatile
bool athome = true, gohome = false, markerLatched = false, isLocked = false, ignoreFirstMarker = true;
unsigned long buzzerTimeout = 0, touchHoldStart = 0;
unsigned long lastMarkerTime = 0;
int lastAction = -1;
int servoAngle = 0;

// --- Forward Declarations ---
void sendAT(String cmd, int timeout = 2000);
void setupESPWiFi();
void processCommand(String input);
void buzzStateChange();
long get_distance_safe(int trig, int echo);
void sendServoPulse(int pin, int angle);
void forkliftUp();
void forkliftDown();
void executePickupRoutine();
void Corrected_Turn_Left();
void drive_logic();
void applyKick(int action);
void moveForward();
void handle_movement();
void count_markers();
void selectDestination();
void updateLcd(String a, String b);
void updateLCDStatus(String s, int t = 0); 
void stop_motors();
void checkReset();
void smartDelay(unsigned long ms);
void safetyCheck();
void triggerLock();
void checkUnlockSequence();
void logMission(int result);
void updateBuzzer();
void beep(int d);
void finishMission();
void printStatusReport();
void handleSerialInput();

// ================= SETUP =================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  espSerial.setTimeout(10);
  Serial.print("startup init");
  motorSpeed = EEPROM.read(ADDR_SPEED);
  if (motorSpeed < 50 || motorSpeed > 255) motorSpeed = 120;

  pinMode(touchPin, INPUT); pinMode(scsensor, INPUT);
  pinMode(IRLeft, INPUT); pinMode(IRRight, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(Trig1, OUTPUT); pinMode(Echo1, INPUT);
  pinMode(Trig2, OUTPUT); pinMode(Echo2, INPUT);
  pinMode(SERVO_PIN, OUTPUT);

  for (int i = 0; i < 30; i++) {
    sendServoPulse(SERVO_PIN, 0);
    delay(20);
  }

  lcd.init(); lcd.backlight();
  lcd.clear();
  updateLcd("Forklift v14", "Touch to Start");
  beep(300);

   unsigned long startup = millis();
  while (millis() - startup < 3000) {
    handleSerialInput();
  }

  setupESPWiFi();

  updateLcd("System Ready", "Spd: " + String(motorSpeed));
}

// ================= MAIN LOOP =================
void loop() {
  sendServoPulse(SERVO_PIN, servoAngle);
  updateBuzzer();

  if (espSerial.available()) {
    String input = espSerial.readString();
    input.trim();
    
    if (input.length() > 0) {
      Serial.print("Raw: "); Serial.println(input);

      int colonPos = input.indexOf(':');
      if (colonPos != -1) {
        String cleanCommand = input.substring(colonPos + 1);
        cleanCommand.trim();
        Serial.print("Action -> "); Serial.println(cleanCommand);
        processCommand(cleanCommand);
      } else {
        processCommand(input);
      }
    }
  }

  handleSerialInput();
  safetyCheck();

  if (isLocked) { 
    checkUnlockSequence(); 
    return; 
  }

  if (digitalRead(touchPin) == HIGH && athome && !gohome) selectDestination();

  // Navigation Logic
  if (targetLoc != 0 && loc < targetLoc) {
    long frontDist = get_distance_safe(Trig1, Echo1);
    if (frontDist > 0 && frontDist < OBSTACLE_DIST) {
      stop_motors();
      updateLcd("OBSTACLE!", String(frontDist) + "cm");
      Serial.print("Obstacle detected");
    } else {
      handle_movement();
      updateLCDStatus("Heading ", targetLoc);
    }
  }
  else if (targetLoc != 0 && loc == targetLoc) {
    stop_motors();
    executePickupRoutine();
    targetLoc = 0;
    gohome = true;
    buzzStateChange();
  }
  else if (gohome) {
    drive_logic();
    updateLCDStatus("Returning"); 
    long homeDist = get_distance_safe(Trig2, Echo2);
    if (homeDist > 1 && homeDist < HOME_STOP_DIST) {
      stop_motors();
      forkliftDown();
      logMission(1);
      finishMission();
    }
  }
}

// ================= ESP-01 =================
void sendAT(String cmd, int timeout) {
  espSerial.println(cmd);
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (espSerial.available()) {
      char c = espSerial.read();
      Serial.write(c);
    }
  }
}

void setupESPWiFi() {
  updateLcd("WiFi Setup", "Connecting...");
  sendAT("AT+RST", 5000);
  sendAT("AT+CWMODE=1", 2000);
  String join = "AT+CWJAP=\"" + String(wifiSSID) + "\",\"" + String(wifiPassword) + "\"";
  sendAT(join, 15000);
  sendAT("AT+CIPMUX=1", 2000);
  sendAT("AT+CIPSERVER=1,8080", 2000);
  Serial.println("\n--- WiFi Ready ---");
}

// ================= COMMAND PROCESSING =================
void processCommand(String input) {
  input.trim();
  if (input.length() == 0) return;
  
  char first = input.charAt(0);
  int num = input.toInt();
  const int MAX_LOC = 5;

  if (first == '+') { 
    motorSpeed = min(motorSpeed + 10, 240); 
    EEPROM.write(ADDR_SPEED, motorSpeed); 
    lcd.clear(); updateLcd("Speed Up", String(motorSpeed));
    beep(50); 
  }
  else if (first == '-') { 
    motorSpeed = max(motorSpeed - 10, 50); 
    EEPROM.write(ADDR_SPEED, motorSpeed); 
    lcd.clear(); updateLcd("Speed Down", String(motorSpeed));
    beep(50); 
  }
  else if (input.length() == 1 && num >= 1 && num <= MAX_LOC) {
    if (athome && !gohome && !isLocked) { 
      targetLoc = num; loc = 0; athome = false; 
      updateLcd("Target Set", String(targetLoc));
      applyKick(1); buzzStateChange(); 
    }
  }
  else if (first == 'H' || first == 'h') { 
    if (!isLocked && !athome) { 
      targetLoc = 0; gohome = true; 
      updateLcd("Command", "Go Home");
      buzzStateChange(); 
    } 
  }
  else if (first == 'M' || first == 'm') printStatusReport();
  else if (first == 'S' || first == 's') triggerLock();
  else if (first == 'C' || first == 'c') { 
    isLocked = false; athome = true; beep(200); 
    updateLcd("Unlocked", "Ready"); 
  }
  else if (input.startsWith("R") || input.startsWith("r")) resetFunc();
  else if (input.equalsIgnoreCase("IP")) { sendAT("AT+CIFSR", 3000); }
}

void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
}

// ================= NAVIGATION & MOTORS =================
long get_distance_safe(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long start = micros();
  while (digitalRead(echo) == LOW) {
    if (micros() - start > 20000UL) return 999;
  }

  unsigned long echoStart = micros();
  while (digitalRead(echo) == HIGH) {
    if (micros() - echoStart > 20000UL) return 999;
  }

  long duration = micros() - echoStart;
  long distance = duration * 0.034 / 2;
  return (distance < 2 || distance > 400) ? 999 : distance;
}

void moveForward() {
  digitalWrite(IN2, LOW);  analogWrite(IN1, motorSpeed);
  digitalWrite(IN3, LOW);  analogWrite(IN4, motorSpeed);
}

void drive_logic() {
  int L = digitalRead(IRLeft);
  int R = digitalRead(IRRight);


if (L == LOW && R == LOW) {
  digitalWrite(IN2, LOW); analogWrite(IN1, motorSpeed); 
  digitalWrite(IN3, LOW); analogWrite(IN4, motorSpeed);
}
  else if (L == HIGH) {        
  digitalWrite(IN2, LOW); analogWrite(IN1, 0);
  digitalWrite(IN3, LOW); analogWrite(IN4, 70);
} 
  else if (R == HIGH) {  
  digitalWrite(IN2, LOW); analogWrite(IN1, 70);
  digitalWrite(IN3, LOW); analogWrite(IN4, 0);
} 
}

void stop_motors() {
  analogWrite(IN1, 0); analogWrite(IN4, 0);
  digitalWrite(IN2, LOW); digitalWrite(IN3, LOW);
}

void applyKick(int action) {
  analogWrite(IN1, 220); analogWrite(IN4, 220);
  delay(70);
}

void count_markers() {
  int val = digitalRead(scsensor);
  if (val == LOW && !markerLatched) {
    if (millis() - lastMarkerTime > DEBOUNCE_DELAY) {
      markerLatched = true;
      lastMarkerTime = millis();
      if (ignoreFirstMarker) {
        ignoreFirstMarker = false;
        return; 
      }
      loc++;
      beep(80);
      lcd.clear();
      updateLcd("T:" + String(targetLoc), "Loc:" + String(loc));
    }
  }
  else if (val == HIGH) {
    markerLatched = false;
  }
}

void handle_movement() {
  drive_logic();
  count_markers();
}

// ================= WORKFLOWS =================
void executePickupRoutine()
{
  lcd.clear(); 
  updateLcd("Station", "Turn Left");
  Serial.println("at loc");
  Corrected_Turn_Left();
  stop_motors(); 
  delay(500);
  moveForward();
  delay(500);
  delay(1000);
  
  drive_logic(); 
  lcd.clear(); 
  updateLcd("Aligning", "To Pallet");
  Serial.println("going to pallet");
  while (get_distance_safe(Trig1, Echo1) > 15) 
  {
    delay(50);
    checkReset();
    drive_logic();
    moveForward();
  }
  stop_motors();

  lcd.clear(); 
  updateLcd("Lifting", "Sweep Start");
  Serial.println("at pallet, picking up");
  forkliftUp();
  smartDelay(1000); 

  lcd.clear(); 
  updateLcd("Storing", "To Wall");
  Serial.println("going to ret track");
  drive_logic();

  while (get_distance_safe(Trig1, Echo1) > 10) {
    checkReset();
    moveForward();
    sendServoPulse(SERVO_PIN, servoAngle);
  }
  stop_motors();
  lcd.clear();
  updateLcd("Exiting", "waiting for line");
  unsigned long Bothonline = 0;
  bool exitLoop = false;
  while (!exitLoop) {
    int L = digitalRead(IRLeft);
    int R = digitalRead(IRRight);
    int sensorState = 0;
    if (L == HIGH) sensorState += 1;
    if (R == HIGH) sensorState += 2;
    switch (sensorState) {
      case 3:
        if (Bothonline == 0) {
          Bothonline = millis();
        } else if (millis() - Bothonline >= 500) {
          exitLoop = true;
        }
        break;
      case 0:
      case 1:
      case 2:
        Bothonline = 0;
        drive_logic();
        break;
    }
    sendServoPulse(SERVO_PIN, servoAngle);
    checkReset();
  }
  stop_motors();
  lcd.clear();
  updateLcd("Exiting", "Turn Left");
  Serial.println("on ret track");
  Corrected_Turn_Left();
  smartDelay(400);
  stop_motors();
}

void forkliftUp() { 
  Serial.println("fork lift up");
  for (int ang = servoAngle; ang <= 180; ang += 10) {
    servoAngle = ang;
    for (int j = 0; j < 8; j++) {
      sendServoPulse(SERVO_PIN, servoAngle);
      delay(20);
    }
  }
}

void forkliftDown() {
  Serial.println("fork lift down");
  for (int ang = servoAngle; ang >= 0; ang -= 10) {
    servoAngle = ang;
    for (int j = 0; j < 8; j++) {
      sendServoPulse(SERVO_PIN, servoAngle);
      delay(20);
    }
  }
}

void Corrected_Turn_Left() 
{
  int L = digitalRead(IRLeft);
  int R = digitalRead(IRRight);

  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW); 
  analogWrite(IN3, 100);
  digitalWrite(IN4, LOW); 
  delay(1500);
}

void selectDestination() {
  int count = 0;
  unsigned long start = millis();
  const int MAX_LOCATIONS = 4; 

  while (millis() - start < 3000) {
    sendServoPulse(SERVO_PIN, servoAngle); 
    
    if (digitalRead(touchPin) == HIGH) {
      count++;
      if (count > MAX_LOCATIONS) {
        count = 1; 
        beep(500); 
      } else {
        beep(100);
      }
      updateLcd("Select Target", String(count));
      
      while(digitalRead(touchPin) == HIGH) {
        if (millis() - start > 2000) resetFunc();
        delay(10);
      }
      
      start = millis(); 
    }
  }

  if (count > 0 && count <= MAX_LOCATIONS) { 
    targetLoc = count; 
    loc = 0; 
    athome = false; 
    updateLcd("Target Set", String(targetLoc));
    applyKick(1);
    buzzStateChange(); 
  }
}

// ================= UTILITIES =================
void sendServoPulse(int pin, int angle) {
  int pulse = map(angle, 0, 180, 544, 2400);
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(pin, LOW);
}

void updateLcd(String a, String b) {
  static String lastA = "", lastB = "";
  if (a != lastA || b != lastB) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(a);
    lcd.setCursor(0,1); lcd.print(b);
    lastA = a; lastB = b;
  }
}

void updateLCDStatus(String s, int t) {
  lcd.setCursor(0, 0);
  lcd.print(s + String(t) + " ");
  delay(10);
  lcd.setCursor(0, 1);
  lcd.print("L:" + String(loc) + " S:" + String(motorSpeed) + " ");
}

void safetyCheck() {
  if (digitalRead(touchPin) == HIGH) {
    if (touchHoldStart == 0) touchHoldStart = millis();
    if (millis() - touchHoldStart > 1000) triggerLock();
  } else { touchHoldStart = 0; }
}

void triggerLock() { 
  stop_motors(); isLocked = true; 
  updateLcd("EMERGENCY", "LOCKED");
  Serial.print("EMR stop dectected"); 
  beep(1000); 
}

void checkUnlockSequence() {
  if (digitalRead(touchPin) == HIGH) { delay(500); resetFunc(); }
}

void buzzStateChange() {
  digitalWrite(BUZZER_PIN, HIGH); delay(150);
  digitalWrite(BUZZER_PIN, LOW);  delay(100);
  digitalWrite(BUZZER_PIN, HIGH); delay(150);
  digitalWrite(BUZZER_PIN, LOW);
}

void beep(int d) { digitalWrite(BUZZER_PIN, HIGH); buzzerTimeout = millis() + d; }
void updateBuzzer() { if (millis() > buzzerTimeout) digitalWrite(BUZZER_PIN, LOW); }
void logMission(int result) { 
  for (int i = 4; i > 0; i--) 
    EEPROM.write(ADDR_LOG_START + i, EEPROM.read(ADDR_LOG_START + i - 1)); 
  EEPROM.write(ADDR_LOG_START, result); 
}

void checkReset() {
  if (digitalRead(touchPin) == HIGH) {
    delay(2000);
    if(digitalRead(touchPin) == HIGH) resetFunc();
  }
}

void smartDelay(unsigned long ms) {
  unsigned long t = millis();
  while(millis() - t < ms) { checkReset(); delay(1); }
}

void finishMission() {
  updateLcd("Mission Done", "Resetting...");
  Serial.print("Home, resetting system"); 
  delay(2000);
  resetFunc();
}

void printStatusReport() {
  float voltage = (analogRead(BATTERY_PIN) / 1023.0) * 10.0;

  Serial.println("\n--- STATUS REPORT ---");
  Serial.print("Battery: "); Serial.print(voltage, 2); Serial.println(" V");
  Serial.print("Motor Speed: "); Serial.println(motorSpeed);
  Serial.print("IR L/R: "); Serial.print(digitalRead(IRLeft)); Serial.print(" / "); Serial.println(digitalRead(IRRight));
  Serial.print("Front dist: "); Serial.print(get_distance_safe(Trig1, Echo1)); Serial.println(" cm");
  Serial.print("Top dist: "); Serial.print(get_distance_safe(Trig2, Echo2)); Serial.println(" cm");
  Serial.print("Target / Current: "); Serial.print(targetLoc); Serial.print(" / "); Serial.println(loc);
  Serial.print("State: ");
  if (isLocked) Serial.println("LOCKED");
  else if (athome) Serial.println("At Home");
  else if (gohome) Serial.println("Returning");
  else Serial.println("En route");

  Serial.println("\nLast 5 Missions:");
  for (int i = 0; i < 5; i++) {
    int val = EEPROM.read(ADDR_LOG_START + i);
    Serial.print("Run "); Serial.print(i+1); Serial.print(": ");
    if (val == 1) Serial.println("SUCCESS");
    else if (val == 99) Serial.println("OBSTACLE");
    else if (val == 55) Serial.println("EMERGENCY");
    else Serial.println("-");
  }
  Serial.println("---------------------\n");
}