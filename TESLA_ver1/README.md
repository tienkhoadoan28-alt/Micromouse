# TESLA_ver1 – Micromouse bám tường trái

Code cuối: [`banket/banket.ino`](banket/banket.ino)

## 1. Phần cứng

| Linh kiện | Chân STM32 | Ghi chú |
|---|---|---|
| Động cơ trái | `ENA = PB0`, `IN1 = PA8`, `IN2 = PA9` | PWM qua ENA |
| Động cơ phải | `ENB = PB1`, `IN3 = PA10`, `IN4 = PA11` | PWM qua ENB |
| Siêu âm trước | `TRIG = PA2`, `ECHO = PA3` | HC-SR04 |
| Siêu âm trái | `TRIG = PA0`, `ECHO = PA1` | HC-SR04 |
| Siêu âm phải | `TRIG = PA4`, `ECHO = PA5` | HC-SR04 |
| MPU6050 | I2C (SDA/SCL) | Thư viện `MPU6050_light` |
| OLED SSD1306 | I2C, địa chỉ `0x3C` | 128×64 |
| Nút Start | `PB11` | `INPUT_PULLUP` |

## 2. Nguyên lý hoạt động

### 2.1 Đọc và lọc cảm biến
- Mỗi cảm biến siêu âm được đo bằng `pulseIn` với timeout 5 ms (≈ 85 cm). Không có tiếng vọng → trả về `999` (coi như đường trống).
- **Lọc trung vị 3 mẫu** (`getFilteredDistance`) loại bỏ các giá trị nhảy bất thường.
- Chu kỳ quét cảm biến: **20 ms**.

### 2.2 Đi thẳng – `runStraight()`
Góc mục tiêu được hiệu chỉnh theo khoảng cách tường, sau đó đưa vào PID theo góc Z của MPU6050:

```
wallAdjustment = (R − L)·Kw                (có 2 tường)
               = (TARGET − L)·Kw           (chỉ tường trái)
               = (R − TARGET)·Kw           (chỉ tường phải)
activeTarget   = targetAngle + clamp(wallAdjustment, ±15°)
correction     = Kp·e + Ki·∫e + Kd·de/dt   (e được lọc low-pass, α = 0.7)
```

- `Kp = 18.5`, `Ki = 0`, `Kd = 4.0`, `Kw = 1.5`, `TARGET_DIST = 7.5 cm`.
- Nếu quá sát tường (< 2.6 cm) → cộng/trừ thêm 35 để đẩy xe ra.
- PWM tối thiểu `MIN_PWM = 75` để thắng ma sát tĩnh.

### 2.3 Rẽ – `executeTurn()`
- PD theo góc (`t_kp = 19.5`, `t_kd = 4.0`), tốc độ tối đa `sp_speed = 198`, tối thiểu 120.
- Dừng khi sai số < 1.5° hoặc quá 1.12 s (timeout).
- **Phanh chủ động:** quay ngược 30 ms để triệt tiêu quán tính, sau đó chờ gyro ổn định.

### 2.4 Logic giải mê cung (Left-Hand Rule)

```
           ┌────────────────────────┐
           │ Trái trống (> 19 cm)?  │──Có──► Xác nhận 5 lần ──► Tiến 188 ms ──► Rẽ TRÁI
           └──────────┬─────────────┘
                     Không
           ┌──────────▼─────────────┐
           │ Trước có tường ≤ 17cm? │──Không──► Tiếp tục ĐI THẲNG
           └──────────┬─────────────┘
                      Có (đọc lại xác nhận)
           ┌──────────▼─────────────┐
           │ Phải trống?            │──Có──► Rẽ PHẢI
           └──────────┬─────────────┘
                     Không
           ┌──────────▼─────────────┐
           │ Bị chặn 3 phía (4/5)?  │──Có──► QUAY ĐẦU 180°
           └──────────┬─────────────┘
                     Không ──► Đọc lại cảm biến, chạy tiếp
```

- Sau mỗi lần rẽ có **cooldown 500 ms** để tránh quyết định lặp tại cùng một ngã.
- `confirmPath()` và `isPathBlocked()` dừng xe rồi đọc nhiều mẫu → giảm hẳn rẽ nhầm do nhiễu siêu âm.

## 3. Sử dụng
1. Bật nguồn → OLED hiển thị `INIT...` → `CALIB MPU` (để xe **đứng yên**) → `READY!`.
2. Khi chờ, OLED hiển thị khoảng cách F / L / R và góc hiện tại.
3. Nhấn nút **PB11** để xe bắt đầu chạy.

## 4. Thông số chính có thể chỉnh

| Biến | Giá trị | Ý nghĩa |
|---|---|---|
| `baseSpeed` | 110 | Tốc độ đi thẳng |
| `sp_speed` | 198 | Tốc độ tối đa khi rẽ |
| `FRONT_WALL_THRESHOLD` | 17.0 cm | Ngưỡng phát hiện tường trước |
| `SIDE_OPEN_THRESHOLD` | 19.0 cm | Ngưỡng coi bên hông là trống |
| `TARGET_DIST` | 7.5 cm | Khoảng cách mong muốn tới tường |
| `COOLDOWN_TIME` | 500 ms | Thời gian chờ sau khi rẽ |

## 5. Hạn chế
- Bám tường trái **không đảm bảo** tìm được đích nếu đích nằm giữa mê cung (đảo tường) và không tìm đường ngắn nhất.
- Không có encoder → không biết chính xác đã đi bao nhiêu ô.
- Siêu âm dễ nhiễu khi góc tới lớn. → Các hạn chế này được khắc phục ở [TESLA_ver2](../TESLA_ver2/).
