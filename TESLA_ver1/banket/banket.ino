#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======================== PIN CONFIGURATION ========================
// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MPU6050
MPU6050 mpu(Wire);

// Motors
const int ENA = PB0;  const int IN1 = PA8;  const int IN2 = PA9;
const int ENB = PB1;  const int IN3 = PA10; const int IN4 = PA11;

// Ultrasonic Sensors
const int TRIG_FRONT = PA2; const int ECHO_FRONT = PA3;
const int TRIG_LEFT  = PA0; const int ECHO_LEFT  = PA1;
const int TRIG_RIGHT = PA4; const int ECHO_RIGHT = PA5;

// User Interface
const int buttonPin = PB11;

// ======================== SYSTEM PARAMETERS ========================
// Movement
int baseSpeed = 110;
int sp_speed = 198;
int maxCorrection = 80;
const int MIN_PWM = 75;

// Sensor Thresholds (cm)
const float FRONT_WALL_THRESHOLD = 17.0;
const float SIDE_OPEN_THRESHOLD = 19.0;
const float TOO_CLOSE = 2.6;
const float TARGET_DIST = 7.5;
const float WALL_DETECTION_RANGE = 12.0;

// PID Gains
float Kp = 18.5;
float Ki = 0.0;
float Kd = 4.0;

// Filtering & State
float alpha = 0.7;
float lastError = 0, integralError = 0, filteredError = 0;
float targetAngle = 0;

float histF[3], histL[3], histR[3];
float lastDistF = 999.0, lastDistL = 999.0, lastDistR = 999.0;

bool isRunning = false;
bool lastButtonState = HIGH;
unsigned long lastPIDTime = 0, lastSensorTime = 0, cooldownUntil = 0, lastDebounceTime = 0;

const int COOLDOWN_TIME = 500;
const int debounceDelay = 50;

// ======================== HELPER FUNCTIONS ========================

float normalizeAngle(float a) {
    while (a > 180.0)  a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

void smartDelay(int ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        mpu.update();
    }
}

void stopMotors() {
    analogWrite(ENA, 0); 
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void drive(int targetSL, int targetSR) {
    targetSL = constrain(targetSL, -255, 255);
    targetSR = constrain(targetSR, -255, 255);
    // Left Motor
    if (targetSL >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    else               { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); targetSL = -targetSL; }
    // Right Motor
    if (targetSR >= 0) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
    else               { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  targetSR = -targetSR; }

    analogWrite(ENA, targetSL);
    analogWrite(ENB, targetSR);
}

// ======================== SENSOR LOGIC ========================

float getRawDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 5000);
    return (duration == 0) ? 999.0 : (duration * 0.0343 / 2.0);
}

// Giữ nguyên tên hàm cũ để không làm lỗi các phần khác trong code
float getFilteredDistance(int trigPin, int echoPin, float history[3]) {
    // 1. Cập nhật mảng lịch sử
    history[2] = history[1];
    history[1] = history[0];
    
    // 2. Đọc giá trị mới
    history[0] = getRawDistance(trigPin, echoPin);

    // 3. Trả về trung vị (Median) để lọc nhiễu
    float a = history[0], b = history[1], c = history[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

void refreshSensors() {
    for (int i = 0; i < 3; i++) {
        lastDistF = getFilteredDistance(TRIG_FRONT, ECHO_FRONT, histF);
        lastDistL = getFilteredDistance(TRIG_LEFT, ECHO_LEFT, histL);
        lastDistR = getFilteredDistance(TRIG_RIGHT, ECHO_RIGHT, histR);
        smartDelay(15);
    }
}

// ======================== DISPLAY ========================

void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    auto printDist = [](const char* label, float dist) {
        display.print(label);
        if (dist > 100) display.println("---");
        else { display.print(dist, 1); display.println(" cm"); }
    };

    printDist("F: ", lastDistF);
    printDist("L: ", lastDistL);
    printDist("R: ", lastDistR);

    display.print("Tg: ");  display.print(normalizeAngle(targetAngle), 1);
    display.print(" Cur: "); display.println(normalizeAngle(mpu.getAngleZ()), 1);
    display.display();
}

// ======================== MOVEMENT CONTROL ========================

void runStraight(float dt) {
    float currentAngle = mpu.getAngleZ();
    float wallAdjustment = 0.0;
    const float Kw = 1.5;

    bool hasLeftWall = (lastDistL < WALL_DETECTION_RANGE);
    bool hasRightWall = (lastDistR < WALL_DETECTION_RANGE);

    if (hasLeftWall && hasRightWall)      wallAdjustment = (lastDistR - lastDistL) * Kw;
    else if (hasLeftWall)                 wallAdjustment = (TARGET_DIST - lastDistL) * Kw;
    else if (hasRightWall)                wallAdjustment = (lastDistR - TARGET_DIST) * Kw;

    wallAdjustment = constrain(wallAdjustment, -15.0, 15.0);
    float activeTargetAngle = normalizeAngle(targetAngle + wallAdjustment);

    // PID Calculation
    float rawError = normalizeAngle(activeTargetAngle - currentAngle);
    filteredError = (alpha * rawError) + ((1.0 - alpha) * filteredError);
    
    integralError = constrain(integralError + (filteredError * dt), -20.0, 20.0);
    float derivative = (filteredError - lastError) / dt;
    float correction = (filteredError * Kp) + (integralError * Ki) + (derivative * Kd);

    // Proximity Safety
    if (lastDistL < TOO_CLOSE)      correction -= 35.0;
    else if (lastDistR < TOO_CLOSE) correction += 35.0;

    correction = constrain(correction, -maxCorrection, maxCorrection);
    
    int targetSL = baseSpeed - (int)correction;
    int targetSR = baseSpeed + (int)correction;

    // Apply Minimum PWM
    auto applyMinPWM = [](int pwm) {
        if (pwm > 0 && pwm < MIN_PWM) return MIN_PWM;
        if (pwm < 0 && pwm > -MIN_PWM) return -MIN_PWM;
        return pwm;
    };

    drive(applyMinPWM(targetSL), applyMinPWM(targetSR));
    lastError = filteredError;
}

void executeTurn(float targetTurnAngle, int sp) {
    // 1. Tối ưu PID cho việc quay (Gains thấp hơn để tránh rung)
    float t_kp = 19.5; 
    float t_kd = 4.0;
    
    float t_lastError = 0;
    unsigned long turnStartTime = millis();
    unsigned long lastUpdateTime = millis();

    // 2. Vòng lặp quay với điều kiện dừng chặt chẽ hơn
    // Cho phép dừng khi sai số < 0.8 độ hoặc hết 1.8 giây (timeout)
    while (fabs(normalizeAngle(targetTurnAngle - mpu.getAngleZ())) >= 1.5) {
        if (millis() - turnStartTime > 1120) break; 

        mpu.update();
        unsigned long now = millis();
        float dt = (now - lastUpdateTime) / 1000.0;
        if (dt < 0.005) continue; // Chạy ở tần số cao hơn để mượt
        lastUpdateTime = now;

        float error = normalizeAngle(targetTurnAngle - mpu.getAngleZ());

        // 3. Tính toán Correction
        float derivative = (error - t_lastError) / dt;
        float correction = (error * t_kp) + (derivative * t_kd);
        t_lastError = error;

        // 4. Giới hạn tốc độ quay để không bị trượt bánh
        correction = constrain(correction, -sp,sp);

        // 5. Áp dụng Deadzone (minT) để thắng ma sát
        int p = (int)correction;
        int minTurnSpeed = 120; // Tăng nhẹ để robot không bị đứng im khi error nhỏ
        
        auto applyTurnPWM = [&](int v) {
            if (v > 0) return max(v, minTurnSpeed);
            if (v < 0) return min(v, -minTurnSpeed);
            return 0;
        };

        drive(applyTurnPWM(p), applyTurnPWM(-p));
    }

    // 6. Phanh chủ động (Active Braking)
    // Quay ngược nhẹ một chút để triệt tiêu quán tính
    if (normalizeAngle(targetTurnAngle - mpu.getAngleZ()) > 0) drive(-80, 80);
    else drive(80, -80);
    delay(30); 

    stopMotors();
    smartDelay(150); // Đợi Gyro ổn định lại
    
    // Cập nhật lại targetAngle mới để chạy thẳng sau khi rẽ
    targetAngle = targetTurnAngle; 
    refreshSensors();
}

void turn90(bool isRight) { executeTurn(mpu.getAngleZ() + (isRight ? 90.0 : -90.0),sp_speed); }
void turn180() { executeTurn(mpu.getAngleZ() + 180.0,sp_speed); }

void moveForwardSlightly(int ms) {
  unsigned long start = millis();
  while(millis() - start < ms) {
    mpu.update();
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDTime >= 10) {
      float dt = (currentMillis - lastPIDTime) / 1000.0;
      lastPIDTime = currentMillis;
      runStraight(dt); 
    }
  }
  stopMotors();
  smartDelay(100); 
}

// Hàm kiểm tra kỹ xem có thực sự bị chặn 3 phía không
bool isPathBlocked() {
    int wallCountF = 0, wallCountL = 0, wallCountR = 0;
    const int CHECK_SAMPLES = 5;

    // Dừng hẳn xe để cảm biến không bị nhiễu do rung động
    stopMotors();
    smartDelay(200); 

    for (int i = 0; i < CHECK_SAMPLES; i++) {
        refreshSensors(); // Cập nhật lại toàn bộ cảm biến
        if (lastDistF < FRONT_WALL_THRESHOLD) wallCountF++;
        if (lastDistL < SIDE_OPEN_THRESHOLD)  wallCountL++;
        if (lastDistR < SIDE_OPEN_THRESHOLD)  wallCountR++;
        delay(30);
    }

    // Chỉ trả về TRUE nếu đa số các lần đọc đều xác nhận có tường bao quanh
    // (Ít nhất 4/5 lần đọc báo có tường)
    return (wallCountF >= 4 && wallCountL >= 4 && wallCountR >= 4);
}

// Trả về true nếu hướng đó thực sự TRỐNG (Open) sau khi kiểm tra nhiều mẫu
bool confirmPath(int trigPin, int echoPin, float threshold) {
    int openCount = 0;
    const int SAMPLES = 5;
    
    stopMotors();
    smartDelay(100); // Dừng ngắn để ổn định sóng âm

    for (int i = 0; i < SAMPLES; i++) {
        // Sử dụng getRawDistance để lấy giá trị tươi nhất, không dùng history cũ
        float d = getRawDistance(trigPin, echoPin);
        if (d > threshold) openCount++;
        delay(20);
    }
    
    // Nếu ít nhất 4/5 lần đọc đều báo trống -> Xác nhận trống
    return (openCount >= 4);
}

// ======================== MAIN LOOPS ========================

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
    display.clearDisplay(); display.setTextColor(WHITE);
    display.setCursor(0, 0); display.println("INIT..."); display.display();

    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
    pinMode(buttonPin, INPUT_PULLUP);

    stopMotors();
    if (mpu.begin() != 0) { display.println("MPU FAIL"); display.display(); while (1); }
    
    display.clearDisplay(); display.println("CALIB MPU"); display.display();
    mpu.calcOffsets();
    display.clearDisplay(); display.println("READY!"); display.display();
}

void loop() {
    mpu.update();
    unsigned long now = millis();

    // Button Handling
    bool reading = digitalRead(buttonPin);
    if (reading != lastButtonState) lastDebounceTime = now;
    if ((now - lastDebounceTime) > debounceDelay && reading == LOW && !isRunning) {
        isRunning = true;
        targetAngle = mpu.getAngleZ();
        lastError = integralError = filteredError = 0;
        refreshSensors();
        lastPIDTime = lastSensorTime = millis();
    }
    lastButtonState = reading;

    if (!isRunning) {
        static unsigned long lastIdle = 0;
        if (now - lastIdle > 200) { lastIdle = now; updateOLED(); }
        return;
    }

// ======================== FULL DOUBLE-CHECK MAZE LOGIC ========================
if (now - lastSensorTime >= 20) {
    lastSensorTime = now;
    lastDistF = getFilteredDistance(TRIG_FRONT, ECHO_FRONT, histF);
    lastDistL = getFilteredDistance(TRIG_LEFT, ECHO_LEFT, histL);
    lastDistR = getFilteredDistance(TRIG_RIGHT, ECHO_RIGHT, histR);

    if (now >= cooldownUntil) {
        
        // --- TRƯỜNG HỢP 1: NGHI NGỜ CÓ ĐƯỜNG RẼ TRÁI ---
        if (lastDistL > SIDE_OPEN_THRESHOLD) {
            // Dừng lại kiểm tra xem có phải nhiễu không
            if (confirmPath(TRIG_LEFT, ECHO_LEFT, SIDE_OPEN_THRESHOLD)) {
                moveForwardSlightly(188); 
                turn90(false); // Thực hiện rẽ trái
                cooldownUntil = millis() + COOLDOWN_TIME;
            }
        } 
        
        // --- TRƯỜNG HỢP 2: TRƯỚC MẶT CÓ TƯỜNG (Cần rẽ phải hoặc quay đầu) ---
        else if (lastDistF <= FRONT_WALL_THRESHOLD) {
            stopMotors();
            smartDelay(150);
            
            // Đọc lại cảm biến F để chắc chắn không phải nhiễu
            if (getRawDistance(TRIG_FRONT, ECHO_FRONT) <= FRONT_WALL_THRESHOLD) {
                
                // Kiểm tra xem bên PHẢI có trống không
                if (confirmPath(TRIG_RIGHT, ECHO_RIGHT, SIDE_OPEN_THRESHOLD)) {
                    turn90(true); // Rẽ phải
                } 
                // Nếu cả TRÁI và PHẢI và TRƯỚC đều kín -> QUAY ĐẦU
               else {
                if (isPathBlocked()) {    // Gọi hàm kiểm tra kỹ 5 lần
                    turn180();
                    cooldownUntil = millis() + COOLDOWN_TIME;
                } else {
                    // Nếu kiểm tra lại thấy vẫn có đường (do nhiễu), thoát ra để chạy tiếp
                    refreshSensors();
                }
               }
                    cooldownUntil = millis() + COOLDOWN_TIME;
                }
        }

        lastPIDTime = millis();
    }
}

    if (now - lastPIDTime >= 10) {
        runStraight((now - lastPIDTime) / 1000.0);
        lastPIDTime = now;
    }
}