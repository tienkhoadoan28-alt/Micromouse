#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <EEPROM.h> // Thư viện lưu trữ Flash bền vững của STM32

// ======================== ĐỘ NHẠY XOAY BÁNH XE TRONG MENU ========================
const int MENU_SENSITIVITY = 20;

// ======================== ĐỊA CHỈ Ô NHỚ EEPROM ========================
#define EEPROM_VALID_ADDR   0   
#define EEPROM_SPEED_ADDR   4   
#define EEPROM_PULSE_ADDR   8   
#define EEPROM_TXT_ADDR     12  
#define EEPROM_TY_ADDR      16  

#define EEPROM_KP_YAW_ADDR  20  
#define EEPROM_KD_YAW_ADDR  24  
#define EEPROM_KP_TURN_ADDR 28  
#define EEPROM_KD_TURN_ADDR 32  
#define EEPROM_KP_WALL_ADDR 36  
#define EEPROM_KD_WALL_ADDR 40  

#define EEPROM_MAGIC_NUM    0x1234567A 

// ======================== CẤU HÌNH CƠ BẢN ========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Adafruit_MPU6050 mpu;
const int IN1 = PA8;
const int IN2 = PA9; const int IN3 = PA10; const int IN4 = PA11; 
const int ENCA_L = PA6;
const int ENCB_L = PA7; 
const int ENCA_R = PB3; const int ENCB_R = PB4; 

volatile long encLeftCount = 0;
volatile long encRightCount = 0;

const int XSHUT_FRONT = PA1; const int XSHUT_LEFT  = PA2;
const int XSHUT_RIGHT = PA0; 
Adafruit_VL53L0X tofFront = Adafruit_VL53L0X();
Adafruit_VL53L0X tofLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X tofRight = Adafruit_VL53L0X();
const int BTN_START = PB11; 

// Thông số di chuyển & PID
const int BASE_SPEED = 130;
const int MAX_PWM = 255;                  
float Kp_enc = 0.02;

float Kp_yaw = 12.0;
float Kd_yaw = 2.5;     
float t_kp = 3.5;
float t_kd = 0.5;       
float Kp_wall = 8.0;
float Kd_wall = 3.0;    

float last_wall_err = 0.0f;
float targetAngle = 0.0f;
float last_yaw_err = 0.0f;
long prevEncLeft = 0; long prevEncRight = 0;
float mpu_yaw = 0.0f;
float gyro_z_offset = 0.0f;
uint32_t mpu_last_micros = 0;

float lastDistF = 999.0, lastDistL = 999.0, lastDistR = 999.0;
uint32_t lastControlTime = 0, lastSensorTime = 0, lastOledTime = 0;
#define WALL_MISSING_DIST 16.0  
#define WALL_FRONT_DIST 7.0     
#define TARGET_SIDE_DIST 7.5    
#define WALL_DEADBAND 0.5       

// ======================== BIẾN MÊ CUNG & TỌA ĐỘ ÂM ========================
#define MAZE_SIZE 32      
#define MAZE_OFFSET 16    

#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define WALL_N 1
#define WALL_E 2
#define WALL_S 4
#define WALL_W 8

byte walls[MAZE_SIZE][MAZE_SIZE];
int maze[MAZE_SIZE][MAZE_SIZE];   

int currX = 0, currY = 0;     
int targetX = 0, targetY = 0; 
int heading = DIR_N;
int robotState = 0; 

int customBaseSpeed = 130;
int customPulsesCell = 445;

void SaveDataToFlash();
void ExecutePidMenu();

// ======================== CÁC HÀM CƠ SỞ ========================
void isrLeft() { if (digitalRead(ENCB_L) == HIGH) encLeftCount++; else encLeftCount--; }
void isrRight() { if (digitalRead(ENCB_R) == HIGH) encRightCount++; else encRightCount--; }
float normalizeAngle(float a) { while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }
void stopMotors() { analogWrite(IN1, 0); analogWrite(IN2, 0); analogWrite(IN3, 0); analogWrite(IN4, 0); }

void drive(int targetSL, int targetSR) {
    targetSL = constrain(targetSL, -MAX_PWM, MAX_PWM); targetSR = constrain(targetSR, -MAX_PWM, MAX_PWM);
    if (targetSL >= 0) { analogWrite(IN1, targetSL); analogWrite(IN2, 0); } else { analogWrite(IN1, 0); analogWrite(IN2, -targetSL); }
    if (targetSR >= 0) { analogWrite(IN3, targetSR); analogWrite(IN4, 0); } else { analogWrite(IN3, 0); analogWrite(IN4, -targetSR); }
}

void MPU_Init() {
    if (!mpu.begin()) { 
        display.clearDisplay(); display.setCursor(0, 0);
        display.print("MPU ERROR!"); display.display();
        while (1) delay(10); 
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void MPU_Calibrate() {
    float sum = 0; sensors_event_t a, g, temp;
    display.clearDisplay(); display.setCursor(0, 0); display.print("CALIB MPU..."); display.display();
    for (int i = 0; i < 1000; i++) { mpu.getEvent(&a, &g, &temp); sum += g.gyro.z; delay(2); }
    gyro_z_offset = sum / 1000.0f; mpu_last_micros = micros();
}

void MPU_Update() {
    uint32_t now = micros(); float dt = (now - mpu_last_micros) / 1000000.0f;
    if (dt <= 0.0001f) return; mpu_last_micros = now;
    sensors_event_t a, g, temp; mpu.getEvent(&a, &g, &temp);
    mpu_yaw = normalizeAngle(mpu_yaw - ((g.gyro.z - gyro_z_offset) * (180.0f / PI) * dt));
}

float getTOFDistance(Adafruit_VL53L0X &tof) {
    VL53L0X_RangingMeasurementData_t measure; tof.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) return measure.RangeMilliMeter / 10.0;
    return 999.0; 
}

// ======================== KHỐI ĐỌC GHI FLASH ========================
void LoadSavedData() {
    uint32_t validFlag = 0;
    EEPROM.get(EEPROM_VALID_ADDR, validFlag);
    
    if (validFlag == EEPROM_MAGIC_NUM) {
        EEPROM.get(EEPROM_SPEED_ADDR, customBaseSpeed);
        EEPROM.get(EEPROM_PULSE_ADDR, customPulsesCell);
        EEPROM.get(EEPROM_TXT_ADDR, targetX);
        EEPROM.get(EEPROM_TY_ADDR, targetY);
        
        int tempKpYaw, tempKdYaw, tempKpTurn, tempKdTurn, tempKpWall, tempKdWall;
        EEPROM.get(EEPROM_KP_YAW_ADDR, tempKpYaw);   Kp_yaw = tempKpYaw / 100.0f;
        EEPROM.get(EEPROM_KD_YAW_ADDR, tempKdYaw);   Kd_yaw = tempKdYaw / 100.0f;
        EEPROM.get(EEPROM_KP_TURN_ADDR, tempKpTurn); t_kp = tempKpTurn / 100.0f;
        EEPROM.get(EEPROM_KD_TURN_ADDR, tempKdTurn); t_kd = tempKdTurn / 100.0f;
        EEPROM.get(EEPROM_KP_WALL_ADDR, tempKpWall); Kp_wall = tempKpWall / 100.0f;
        EEPROM.get(EEPROM_KD_WALL_ADDR, tempKdWall); Kd_wall = tempKdWall / 100.0f;

        customBaseSpeed = constrain(customBaseSpeed, 60, 255);
        customPulsesCell = constrain(customPulsesCell, 200, 800);
        targetX = constrain(targetX, -15, 15);
        targetY = constrain(targetY, -15, 15);
    } else {
        customBaseSpeed = 130; customPulsesCell = 445; targetX = 0; targetY = 0;
        Kp_yaw = 12.0; Kd_yaw = 2.5; t_kp = 3.5; t_kd = 0.5; Kp_wall = 8.0; Kd_wall = 3.0;
        SaveDataToFlash();
    }
}

void SaveDataToFlash() {
    EEPROM.put(EEPROM_VALID_ADDR, (uint32_t)EEPROM_MAGIC_NUM);
    EEPROM.put(EEPROM_SPEED_ADDR, customBaseSpeed);
    EEPROM.put(EEPROM_PULSE_ADDR, customPulsesCell);
    EEPROM.put(EEPROM_TXT_ADDR, targetX);
    EEPROM.put(EEPROM_TY_ADDR, targetY);
    
    EEPROM.put(EEPROM_KP_YAW_ADDR, (int)(Kp_yaw * 100));
    EEPROM.put(EEPROM_KD_YAW_ADDR, (int)(Kd_yaw * 100));
    EEPROM.put(EEPROM_KP_TURN_ADDR, (int)(t_kp * 100));
    EEPROM.put(EEPROM_KD_TURN_ADDR, (int)(t_kd * 100));
    EEPROM.put(EEPROM_KP_WALL_ADDR, (int)(Kp_wall * 100));
    EEPROM.put(EEPROM_KD_WALL_ADDR, (int)(Kd_wall * 100));
}

// ======================== HÀM ĐIỀU KHIỂN MENU ========================
bool checkButtonWithDebounce() {
    if (digitalRead(BTN_START) == LOW) {
        delay(200);
        while (digitalRead(BTN_START) == LOW); 
        delay(100);
        return true;
    }
    return false;
}

float adjustFloatParameter(float minVal, float maxVal, float startVal, const char* label) {
    encRightCount = (long)(startVal * 10);
    long lastEnc = encRightCount;
    while (true) {
        long currentEnc = encRightCount;
        if (currentEnc != lastEnc) { lastEnc = currentEnc; }
        float currentVal = currentEnc / 10.0f;
        currentVal = constrain(currentVal, minVal, maxVal);

        display.clearDisplay();
        display.setTextSize(1); display.setCursor(0, 0); display.print("CHINH PID:");
        display.setCursor(0, 15); display.print(label);
        display.setTextSize(2); display.setCursor(10, 32); display.print(currentVal, 1);
        display.setTextSize(1); display.setCursor(0, 55); display.print("[Nhan nut de LUU]");
        display.display();

        if (checkButtonWithDebounce()) return currentVal;
        delay(20);
    }
}

int adjustParameterValue(int minVal, int maxVal, int startVal, const char* label) {
    encRightCount = startVal * MENU_SENSITIVITY;
    while (true) {
        long currentEnc = encRightCount / MENU_SENSITIVITY;
        int currentVal = currentEnc;
        currentVal = constrain(currentVal, minVal, maxVal);

        display.clearDisplay();
        display.setTextSize(1); display.setCursor(0, 0); display.print("CHINH THONG SO:");
        display.setCursor(0, 15); display.print(label);
        display.setTextSize(2); display.setCursor(10, 32); display.print(currentVal);
        display.setTextSize(1); display.setCursor(0, 55); display.print("[Nhan nut de LUU]");
        display.display();
        if (checkButtonWithDebounce()) return currentVal;
        delay(20);
    }
}

void ExecutePidMenu() {
    int pidSelection = 0; encRightCount = 0; long lastPidEnc = 0;
    while (true) {
        long currentPidEnc = encRightCount / MENU_SENSITIVITY;
        if (currentPidEnc != lastPidEnc) {
            if (currentPidEnc > lastPidEnc) pidSelection++; else pidSelection--;
            if (pidSelection > 6) pidSelection = 0;
            if (pidSelection < 0) pidSelection = 6;
            lastPidEnc = currentPidEnc;
        }

        display.clearDisplay();
        display.setTextSize(1); display.setCursor(0, 0); display.println("--- MENU CHINH PID ---");
        display.setCursor(2, 12); display.print(pidSelection == 0 ? ">Kp Thang:" : " Kp Thang:"); display.print(Kp_yaw, 1);
        display.setCursor(2, 20); display.print(pidSelection == 1 ? ">Kd Thang:" : " Kd Thang:"); display.print(Kd_yaw, 1);
        display.setCursor(2, 28); display.print(pidSelection == 2 ? ">Kp Queo :" : " Kp Queo :"); display.print(t_kp, 1);
        display.setCursor(2, 36); display.print(pidSelection == 3 ? ">Kd Queo :" : " Kd Queo :"); display.print(t_kd, 1);
        display.setCursor(2, 44); display.print(pidSelection == 4 ? ">Kp Tuong:" : " Kp Tuong:"); display.print(Kp_wall, 1);
        display.setCursor(2, 52); display.print(pidSelection == 5 ? ">Kd Tuong:" : " Kd Tuong:"); display.print(Kd_wall, 1);
        display.setCursor(2, 60); display.print(pidSelection == 6 ? ">[QUAY LAI]" : " [QUAY LAI]");
        display.display();
        
        if (checkButtonWithDebounce()) {
            if (pidSelection == 0)      { Kp_yaw = adjustFloatParameter(0.0, 50.0, Kp_yaw, "Kp Thang (Goc:12)"); SaveDataToFlash(); }
            else if (pidSelection == 1) { Kd_yaw = adjustFloatParameter(0.0, 20.0, Kd_yaw, "Kd Thang (Goc:2.5)"); SaveDataToFlash(); }
            else if (pidSelection == 2) { t_kp = adjustFloatParameter(0.0, 30.0, t_kp, "Kp Queo (Goc:3.5)"); SaveDataToFlash(); }
            else if (pidSelection == 3) { t_kd = adjustFloatParameter(0.0, 10.0, t_kd, "Kd Queo (Goc:0.5)"); SaveDataToFlash(); }
            else if (pidSelection == 4) { Kp_wall = adjustFloatParameter(0.0, 40.0, Kp_wall, "Kp Tuong (Goc:8)"); SaveDataToFlash(); }
            else if (pidSelection == 5) { Kd_wall = adjustFloatParameter(0.0, 20.0, Kd_wall, "Kd Tuong (Goc:3)"); SaveDataToFlash(); }
            else if (pidSelection == 6) { break; }
            encRightCount = pidSelection * MENU_SENSITIVITY; lastPidEnc = pidSelection;
        }
        delay(20);
    }
}

void ExecuteSystemMenu() {
    int currentSelection = 0; encRightCount = 0; long lastEncState = 0;
    while (true) {
        long currentEncState = encRightCount / MENU_SENSITIVITY;
        if (currentEncState != lastEncState) {
            if (currentEncState > lastEncState) currentSelection++; else currentSelection--;
            if (currentSelection > 2) currentSelection = 0;
            if (currentSelection < 0) currentSelection = 2;
            lastEncState = currentEncState;
        }

        display.clearDisplay();
        display.setTextSize(1); display.setCursor(0, 0); display.println("--- MICROMOUSE MENU ---");
        display.setCursor(5, 18); display.print(currentSelection == 0 ? "> " : "  "); display.println("RUN");
        display.setCursor(5, 30); display.print(currentSelection == 1 ? "> " : "  "); display.println("THAY DOI TOA DO");
        display.setCursor(5, 42); display.print(currentSelection == 2 ? "> " : "  "); display.println("THAY DOI THONG SO");
        display.display();
        
        if (checkButtonWithDebounce()) {
            if (currentSelection == 0) { break; } 
            else if (currentSelection == 1) {
                noInterrupts();
                encLeftCount = (long)targetY * MENU_SENSITIVITY; encRightCount = (long)targetX * MENU_SENSITIVITY;
                interrupts();
                while (true) {
                    targetY = (int)(encLeftCount / MENU_SENSITIVITY); targetX = (int)(encRightCount / MENU_SENSITIVITY);
                    targetY = constrain(targetY, -15, 15); targetX = constrain(targetX, -15, 15);
                    
                    display.clearDisplay();
                    display.setTextSize(1); display.setCursor(0,0); display.println("SET TARGET (X, Y):");
                    display.setCursor(0,15); display.println("Quay B.Trai->Y, Phai->X");
                    display.setTextSize(2); display.setCursor(10,32); display.print(targetX); display.print(","); display.print(targetY);
                    display.setTextSize(1); display.setCursor(0,55); display.print("[Nhan nut -> LUU]");
                    display.display();
                    
                    if (checkButtonWithDebounce()) { SaveDataToFlash(); break; }
                    delay(30);
                }
                encRightCount = 0; lastEncState = 0;
            } 
            else if (currentSelection == 2) {
                int paramMenuSelection = 0; encRightCount = 0; long lastParamEnc = 0;
                while (true) {
                    long currentParamEnc = encRightCount / MENU_SENSITIVITY;
                    if (currentParamEnc != lastParamEnc) {
                        if (currentParamEnc > lastParamEnc) paramMenuSelection++; else paramMenuSelection--;
                        if (paramMenuSelection > 3) paramMenuSelection = 0;
                        if (paramMenuSelection < 0) paramMenuSelection = 3;
                        lastParamEnc = currentParamEnc;
                    }

                    display.clearDisplay();
                    display.setTextSize(1); display.setCursor(0, 0); display.println("--- CONFIG MENU ---");
                    display.setCursor(5, 14); display.print(paramMenuSelection == 0 ? "> Speed: " : "  Speed: "); display.println(customBaseSpeed);
                    display.setCursor(5, 26); display.print(paramMenuSelection == 1 ? "> Pulse/Cell: " : "  Pulse/Cell: "); display.println(customPulsesCell);
                    display.setCursor(5, 38); display.print(paramMenuSelection == 2 ? "> CAI DAT PID..." : "  CAI DAT PID...");
                    display.setCursor(5, 50); display.print(paramMenuSelection == 3 ? "> BACK TO MENU" : "  BACK TO MENU");
                    display.display();
                    
                    if (checkButtonWithDebounce()) {
                        if (paramMenuSelection == 0)      { customBaseSpeed = adjustParameterValue(60, 255, customBaseSpeed, "Base Speed (PWM)"); SaveDataToFlash(); } 
                        else if (paramMenuSelection == 1) { customPulsesCell = adjustParameterValue(200, 800, customPulsesCell, "Pulses Per Cell"); SaveDataToFlash(); } 
                        else if (paramMenuSelection == 2) { ExecutePidMenu(); }
                        else if (paramMenuSelection == 3) { break; }
                        encRightCount = paramMenuSelection * MENU_SENSITIVITY; lastParamEnc = paramMenuSelection;
                    }
                    delay(30);
                }
                encRightCount = 2 * MENU_SENSITIVITY; lastEncState = 2;
            }
        }
        delay(30);
    }
}

// ======================== HÀM DI CHUYỂN NGUYÊN BẢN ========================
void MoveStraight(float dt, int currentBaseSpeed) {
    float steerEnc = (((encLeftCount - prevEncLeft) / dt) - ((encRightCount - prevEncRight) / dt)) * Kp_enc;
    prevEncLeft = encLeftCount; prevEncRight = encRightCount;
    
    float errorYaw = normalizeAngle(mpu_yaw - targetAngle);
    float steerYaw = (errorYaw * Kp_yaw) + (((errorYaw - last_yaw_err) / dt) * Kd_yaw);
    last_yaw_err = errorYaw;
    
    bool hasLeft  = (lastDistL < WALL_MISSING_DIST);
    bool hasRight = (lastDistR < WALL_MISSING_DIST);

    float wall_error = 0.0f;
    if (hasLeft && hasRight) { wall_error = lastDistL - lastDistR; } 
    else if (hasLeft)        { wall_error = lastDistL - TARGET_SIDE_DIST; } 
    else if (hasRight)       { wall_error = TARGET_SIDE_DIST - lastDistR; }

    if (fabs(wall_error) < WALL_DEADBAND) { wall_error = 0.0f; }
    wall_error = constrain(wall_error, -12.0f, 12.0f);
    
    float derivative_wall = (wall_error - last_wall_err) / dt;
    derivative_wall = constrain(derivative_wall, -50.0f, 50.0f); 

    float steerWall = (wall_error * Kp_wall) + (derivative_wall * Kd_wall);
    last_wall_err = wall_error;
    
    int total_corr = (int)(steerYaw + steerWall + steerEnc); 
    total_corr = constrain(total_corr, -35, 35);
    drive(currentBaseSpeed - total_corr, currentBaseSpeed + total_corr);
}

bool MoveCells(int numCells) {
    long startP = encLeftCount;
    int tofCycle = 0;
    long targetPulses = numCells * customPulsesCell; 
    long pulses_slowdown_threshold = 370; 

    const unsigned long TIMEOUT_MS = numCells * 3000;
    unsigned long startMs = millis();

    while (true) {
        MPU_Update();
        uint32_t now = millis();
        if (now - startMs > TIMEOUT_MS) { stopMotors(); return false; }

        long pulses_traveled = labs(encLeftCount - startP);
        long distLeft = targetPulses - pulses_traveled;
        int spd = customBaseSpeed; 
        if (distLeft <= pulses_slowdown_threshold) { spd = customBaseSpeed / 2; }

        if (now - lastSensorTime >= 10) {
            lastSensorTime = now;
            if (tofCycle == 0)      lastDistF = getTOFDistance(tofFront);
            else if (tofCycle == 1) lastDistL = getTOFDistance(tofLeft);
            else                    lastDistR = getTOFDistance(tofRight);
            tofCycle = (tofCycle + 1) % 3;

            if (lastDistF <= WALL_FRONT_DIST) { drive(-80, -80); delay(30); stopMotors(); return true; }
        }

        if (now - lastControlTime >= 10) {
            float dt = max((now - lastControlTime) / 1000.0f, 0.005f);
            MoveStraight(dt, spd);
            lastControlTime = now;
        }

        if (distLeft <= 0) { drive(-50, -50); delay(20); stopMotors(); return true; }
    }
}

void TurnAngleByMPU(float angleChange) {
    targetAngle = normalizeAngle(targetAngle + angleChange);
    float t_lastErr = 0; unsigned long startT = millis(); unsigned long lastT_micros = micros();
    int turnDir = (angleChange > 0) ? 1 : -1;
    while (fabs(normalizeAngle(targetAngle - mpu_yaw)) >= 1.0) {
        if (millis() - startT > 1200) break;
        MPU_Update(); unsigned long now_micros = micros(); 
        float dt = max((now_micros - lastT_micros)/1000000.0f, 0.001f); lastT_micros = now_micros;
        float err = normalizeAngle(targetAngle - mpu_yaw);
        float derivative = (err - t_lastErr) / dt;
        float corr = (err * t_kp) + (derivative * t_kd); t_lastErr = err;
        
        int currentMaxSpeed = 130; int currentMinSpeed = 120;  
        if (fabs(err) <= 45.0) { currentMaxSpeed = 80; currentMinSpeed = 70; }
        corr = constrain(corr, -currentMaxSpeed, currentMaxSpeed);
        int p = (int)corr;
        auto applyTurnPWM = [&](int v) {
            if (v > 0) return max(v, currentMinSpeed);
            if (v < 0) return min(v, -currentMinSpeed);
            return 0;
        };
        drive(applyTurnPWM(p), applyTurnPWM(-p));
    }
    drive(turnDir * -60, turnDir * 60); delay(30); stopMotors(); delay(150);
    if (fabs(normalizeAngle(targetAngle - mpu_yaw)) > 5.0) { targetAngle = mpu_yaw; }

    prevEncLeft = encLeftCount = 0; prevEncRight = encRightCount = 0; 
    last_yaw_err = 0; last_wall_err = 0.0f;
}

void TurnLeft()  { TurnAngleByMPU(-90.0); }
void TurnRight() { TurnAngleByMPU(90.0); }
void TurnAround() { TurnAngleByMPU(179.9); }

// ======================== THUẬT TOÁN FLOOD FILL ========================
void InitMaze() {
    for(int i=0; i<MAZE_SIZE; i++) {
        for(int j=0; j<MAZE_SIZE; j++) { walls[i][j] = 0; maze[i][j] = 255; }
    }
    for(int i=0; i<MAZE_SIZE; i++) {
        walls[i][0] |= WALL_S; walls[i][MAZE_SIZE-1] |= WALL_N;
        walls[0][i] |= WALL_W; walls[MAZE_SIZE-1][i] |= WALL_E;
    }
}

void RunFloodFill() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) { maze[i][j] = 255; }
    }
    int tX = targetX + MAZE_OFFSET; int tY = targetY + MAZE_OFFSET;
    if(tX < 0 || tX >= MAZE_SIZE || tY < 0 || tY >= MAZE_SIZE) return;
    struct Point { int x; int y; };
    Point queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0; int tail = 0;
    maze[tX][tY] = 0; queue[tail].x = tX; queue[tail].y = tY; tail++;

    int dx[] = {0, 1, 0, -1}; int dy[] = {1, 0, -1, 0};
    int wall_masks[] = {WALL_N, WALL_E, WALL_S, WALL_W};
    while (head < tail) {
        Point p = queue[head]; head++;
        int current_dist = maze[p.x][p.y];
        for (int i = 0; i < 4; i++) {
            if (!(walls[p.x][p.y] & wall_masks[i])) {
                int nx = p.x + dx[i]; int ny = p.y + dy[i];
                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    if (maze[nx][ny] == 255) {
                        maze[nx][ny] = current_dist + 1;
                        queue[tail].x = nx; queue[tail].y = ny; tail++;
                    }
                }
            }
        }
    }
}

void SetWall(int x, int y, byte wallType) {
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;
    walls[x][y] |= wallType;
    if (wallType == WALL_N && y < MAZE_SIZE - 1) walls[x][y + 1] |= WALL_S;
    if (wallType == WALL_S && y > 0)             walls[x][y - 1] |= WALL_N;
    if (wallType == WALL_E && x < MAZE_SIZE - 1) walls[x + 1][y] |= WALL_W;
    if (wallType == WALL_W && x > 0)             walls[x - 1][y] |= WALL_E;
}

void UpdateWalls() {
    bool wallF = (lastDistF < WALL_MISSING_DIST);
    bool wallL = (lastDistL < WALL_MISSING_DIST);
    bool wallR = (lastDistR < WALL_MISSING_DIST);

    int pX = currX + MAZE_OFFSET; int pY = currY + MAZE_OFFSET;
    if (heading == DIR_N) {
        if (wallF) SetWall(pX, pY, WALL_N);
        if (wallL) SetWall(pX, pY, WALL_W); if (wallR) SetWall(pX, pY, WALL_E);
    } else if (heading == DIR_E) {
        if (wallF) SetWall(pX, pY, WALL_E);
        if (wallL) SetWall(pX, pY, WALL_N); if (wallR) SetWall(pX, pY, WALL_S);
    } else if (heading == DIR_S) {
        if (wallF) SetWall(pX, pY, WALL_S);
        if (wallL) SetWall(pX, pY, WALL_E); if (wallR) SetWall(pX, pY, WALL_W);
    } else if (heading == DIR_W) {
        if (wallF) SetWall(pX, pY, WALL_W);
        if (wallL) SetWall(pX, pY, WALL_S); if (wallR) SetWall(pX, pY, WALL_N);
    }
}

void DecideNextMove() {
    if (robotState == 2) { UpdateWalls(); RunFloodFill(); }
    int pX = currX + MAZE_OFFSET; int pY = currY + MAZE_OFFSET;
    if (pX < 0 || pX >= MAZE_SIZE || pY < 0 || pY >= MAZE_SIZE) { stopMotors(); return; }

    int minVal = 255; int nextDir = heading;
    if (!(walls[pX][pY] & WALL_N) && pY < MAZE_SIZE - 1) { if (maze[pX][pY + 1] < minVal) { minVal = maze[pX][pY + 1]; nextDir = DIR_N; } }
    if (!(walls[pX][pY] & WALL_E) && pX < MAZE_SIZE - 1) { if (maze[pX + 1][pY] < minVal) { minVal = maze[pX + 1][pY]; nextDir = DIR_E; } }
    if (!(walls[pX][pY] & WALL_S) && pY > 0)             { if (maze[pX][pY - 1] < minVal) { minVal = maze[pX][pY - 1]; nextDir = DIR_S; } }
    if (!(walls[pX][pY] & WALL_W) && pX > 0)             { if (maze[pX - 1][pY] < minVal) { minVal = maze[pX - 1][pY]; nextDir = DIR_W; } }

    if (minVal == 255) { stopMotors(); return; }

    int dirDiff = (nextDir - heading + 4) % 4;
    if (dirDiff == 1)      TurnRight();
    else if (dirDiff == 2) TurnAround();
    else if (dirDiff == 3) TurnLeft();

    heading = nextDir; delay(50);

    int cellsToMove = 1;
    if (robotState == 4) { 
        cellsToMove = 0; int testX = pX; int testY = pY; int currentDist = maze[pX][pY];
        while(currentDist > 0) {
            bool wallAhead = false; int nextX = testX, nextY = testY;
            if (heading == DIR_N)      { nextY++; wallAhead = walls[testX][testY] & WALL_N; }
            else if (heading == DIR_E) { nextX++; wallAhead = walls[testX][testY] & WALL_E; }
            else if (heading == DIR_S) { nextY--; wallAhead = walls[testX][testY] & WALL_S; }
            else if (heading == DIR_W) { nextX--; wallAhead = walls[testX][testY] & WALL_W; }
            
            if (wallAhead) break;
            if (maze[nextX][nextY] == currentDist - 1) { cellsToMove++; testX = nextX; testY = nextY; currentDist--; } 
            else break;
        }
        if (cellsToMove == 0) cellsToMove = 1;
    }

    lastDistF = getTOFDistance(tofFront);
    if (lastDistF < WALL_MISSING_DIST) {
        if (robotState == 2) {
            if (heading == DIR_N)      SetWall(pX, pY, WALL_N);
            else if (heading == DIR_E) SetWall(pX, pY, WALL_E);
            else if (heading == DIR_S) SetWall(pX, pY, WALL_S);
            else if (heading == DIR_W) SetWall(pX, pY, WALL_W);
        }
        stopMotors(); return;
    }

    if (MoveCells(cellsToMove)) {
        if (heading == DIR_N)      currY += cellsToMove;
        else if (heading == DIR_E) currX += cellsToMove;
        else if (heading == DIR_S) currY -= cellsToMove;
        else if (heading == DIR_W) currX -= cellsToMove;
    } else { stopMotors(); }
}

// ======================== SETUP ========================
void setup() {
    Serial.begin(115200); Wire.begin(); Wire.setClock(400000);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
    
    display.clearDisplay(); display.setTextColor(WHITE); display.setTextSize(1);
    display.setCursor(0, 0); display.print("Booting System..."); display.display();

    LoadSavedData();
    pinMode(XSHUT_FRONT, OUTPUT); pinMode(XSHUT_LEFT, OUTPUT); pinMode(XSHUT_RIGHT, OUTPUT);
    digitalWrite(XSHUT_FRONT, LOW); digitalWrite(XSHUT_LEFT, LOW); digitalWrite(XSHUT_RIGHT, LOW); delay(10);
    
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); stopMotors();
    pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
    pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCA_L), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCA_R), isrRight, RISING);
    pinMode(BTN_START, INPUT_PULLUP);
    
    ExecuteSystemMenu();

    robotState = 1;
    display.clearDisplay(); display.setTextSize(2); display.setCursor(0,0); display.print("INIT TOF..."); display.display();
    digitalWrite(XSHUT_FRONT, HIGH); delay(10); 
    if (!tofFront.begin(0x30)) { display.print("F FAIL"); display.display(); while(1); }
    digitalWrite(XSHUT_LEFT, HIGH); delay(10);
    if (!tofLeft.begin(0x31)) { display.print("L FAIL"); display.display(); while(1); }
    digitalWrite(XSHUT_RIGHT, HIGH); delay(10); 
    if (!tofRight.begin(0x32)) { display.print("R FAIL"); display.display(); while(1); }

    display.clearDisplay(); display.setCursor(0,0); display.print("INIT MPU..."); display.display();
    MPU_Init(); MPU_Calibrate();
    InitMaze(); 
    
    MPU_Update();
    targetAngle = mpu_yaw; 
    currX = 0; currY = 0; heading = DIR_N; 
    prevEncLeft = encLeftCount = 0; prevEncRight = encRightCount = 0;
    last_yaw_err = 0.0f; last_wall_err = 0.0f;

    // ĐÃ SỬA CHÍNH XÁC: Đồng bộ hóa mốc thời gian ngay sau khi kích hoạt chế độ chạy ô để loại bỏ lỗi đứng im
    lastControlTime = millis();
    lastSensorTime  = millis();
    mpu_last_micros = micros();
    robotState = 2;
}

// ======================== MAIN LOOP ========================
void loop() {
    MPU_Update(); 
    uint32_t now = millis();
    
    // ĐÃ SỬA CHÍNH XÁC: Hạ chu kỳ quét cảm biến xuống 10ms đồng bộ dữ liệu chuẩn
    if (now - lastSensorTime >= 10) {
        lastSensorTime = now;
        lastDistF = getTOFDistance(tofFront);
        lastDistL = getTOFDistance(tofLeft);
        lastDistR = getTOFDistance(tofRight);
    }

    if (now - lastOledTime >= 200) {
        lastOledTime = now;
        display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0);
        if(robotState == 2) display.println("EXPLORING...");
        if(robotState == 3) display.println("WAIT FAST RUN");
        if(robotState == 4) display.println("SPEED RUN !");
        if(robotState == 5) display.println("DONE! PRESS BTN");
        
        display.setCursor(0, 15); display.print("Pos: "); display.print(currX); display.print(","); display.print(currY);
        display.print(" Tgt: "); display.print(targetX); display.print(","); display.print(targetY);
        display.setCursor(0, 30); display.print("Dist to Tgt: "); display.println(maze[currX + MAZE_OFFSET][currY + MAZE_OFFSET]);
        display.display();
    }

    if (robotState == 2 || robotState == 4) {
        if (now - lastControlTime >= 10) {
            lastControlTime = now;
            if (currX == targetX && currY == targetY) {
                stopMotors();
                if (robotState == 2)      robotState = 3; 
                else if (robotState == 4) robotState = 5;
                return;
            }
            DecideNextMove();
        }
    }
    else if (robotState == 3) {
        if (digitalRead(BTN_START) == LOW) {
            delay(300);
            display.clearDisplay(); display.setTextSize(2); display.setCursor(0, 0); display.print("RE-CALIB..."); display.display();
            stopMotors(); MPU_Calibrate(); MPU_Update();

            currX = 0; currY = 0; heading = DIR_N;
            RunFloodFill();
            targetAngle = normalizeAngle(mpu_yaw);

            encLeftCount = 0; encRightCount = 0; prevEncLeft = 0; prevEncRight = 0;
            last_yaw_err = 0.0f; last_wall_err = 0.0f;
            lastDistF = 999.0; lastDistL = 999.0; lastDistR = 999.0;

            lastControlTime = millis(); lastSensorTime  = millis();
            mpu_last_micros = micros();
            robotState = 4; 
        }
    }
    else if (robotState == 5) {
        if (digitalRead(BTN_START) == LOW) {
            delay(200);
            while (digitalRead(BTN_START) == LOW); delay(100);
            
            ExecuteSystemMenu();
            
            currX = 0; currY = 0; heading = DIR_N;
            MPU_Update(); targetAngle = mpu_yaw;

            encLeftCount = 0; encRightCount = 0; prevEncLeft = 0; prevEncRight = 0;
            last_yaw_err = 0.0f; last_wall_err = 0.0f;
            lastDistF = 999.0; lastDistL = 999.0; lastDistR = 999.0;

            lastControlTime = millis(); lastSensorTime  = millis();
            robotState = 2;
        }
    }
}