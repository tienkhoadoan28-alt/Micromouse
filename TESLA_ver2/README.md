# TESLA_ver2 – Micromouse Flood Fill

Code cuối: [`test2/test2.ino`](test2/test2.ino)

## 1. Phần cứng

| Linh kiện | Chân STM32 | Ghi chú |
|---|---|---|
| Động cơ trái | `IN1 = PA8`, `IN2 = PA9` | PWM trực tiếp trên IN (driver kiểu DRV8833) |
| Động cơ phải | `IN3 = PA10`, `IN4 = PA11` | |
| Encoder trái | `A = PA6`, `B = PA7` | Ngắt cạnh lên trên kênh A |
| Encoder phải | `A = PB3`, `B = PB4` | |
| VL53L0X trước | `XSHUT = PA1` | Địa chỉ I2C `0x30` |
| VL53L0X trái | `XSHUT = PA2` | Địa chỉ I2C `0x31` |
| VL53L0X phải | `XSHUT = PA0` | Địa chỉ I2C `0x32` |
| MPU6050 | I2C (400 kHz) | Gyro ±2000 °/s, lọc 44 Hz |
| OLED SSD1306 | I2C `0x3C` | Hiển thị menu & trạng thái |
| Nút | `PB11` | Chọn / Lưu / Start |

> Ba cảm biến VL53L0X cùng địa chỉ mặc định, nên lúc khởi động cả ba bị kéo `XSHUT` xuống LOW, sau đó lần lượt bật từng con và gán địa chỉ mới `0x30 / 0x31 / 0x32`.

## 2. Bộ điều khiển chuyển động

### 2.1 Tính góc Yaw
`MPU_Update()` tích phân vận tốc góc trục Z (đã trừ offset hiệu chuẩn từ 1000 mẫu) theo `micros()` để ra góc yaw.

### 2.2 Đi thẳng – `MoveStraight()`
Tổng hiệu chỉnh gồm 3 thành phần:

```
steerYaw  = Kp_yaw·e_yaw  + Kd_yaw·de_yaw/dt
steerWall = Kp_wall·e_wall + Kd_wall·de_wall/dt     (deadband 0.5 cm, giới hạn ±12)
steerEnc  = (vL − vR)·Kp_enc                        (cân bằng tốc độ 2 bánh)
corr      = clamp(steerYaw + steerWall + steerEnc, ±35)
PWM_L = base − corr ,  PWM_R = base + corr
```

### 2.3 Đi theo ô – `MoveCells(n)`
- Mục tiêu: `n × customPulsesCell` xung encoder (mặc định 445 xung/ô).
- Giảm tốc còn 1/2 khi còn ≤ 370 xung; dừng sớm nếu tường trước ≤ 7 cm; có timeout `3 s × n`.
- Cảm biến ToF đọc **xoay vòng** (trước → trái → phải) mỗi 10 ms để không chặn vòng điều khiển.

### 2.4 Rẽ – `TurnAngleByMPU()`
- PD theo góc (`t_kp`, `t_kd`), tốc độ 120–130, khi còn ≤ 45° thì giảm xuống 70–80 để không vượt góc.
- Dừng khi sai số < 1° hoặc quá 1.2 s, phanh ngược 30 ms.

## 3. Thuật toán Flood Fill

- Bản đồ `32 × 32` ô với `MAZE_OFFSET = 16` → hỗ trợ **tọa độ âm** (xuất phát tại `(0,0)`, đích trong khoảng −15…15). Nhờ vậy không cần biết trước xe đặt ở góc nào của mê cung.
- Mỗi ô lưu tường bằng bitmask: `N = 1, E = 2, S = 4, W = 8`; `SetWall()` cập nhật luôn tường của ô kề.
- `RunFloodFill()`: BFS từ ô đích, gán khoảng cách cho mọi ô đến được.
- `DecideNextMove()`: cập nhật tường → flood lại → chọn ô kề có giá trị nhỏ nhất → rẽ (trái/phải/quay đầu) → đi.

### Máy trạng thái

| `robotState` | Ý nghĩa |
|---|---|
| 1 | Khởi tạo ToF, MPU |
| 2 | **Khám phá** – đi từng ô, vừa đi vừa vẽ bản đồ |
| 3 | Đã tới đích – chờ nhấn nút, đặt xe về ô xuất phát |
| 4 | **Chạy nhanh** – dùng bản đồ đã biết, **gộp các ô thẳng liên tiếp** thành 1 lần chạy |
| 5 | Hoàn thành – nhấn nút để vào lại menu |

## 4. Menu OLED

Điều hướng bằng cách **xoay bánh xe bằng tay** (đọc encoder, độ nhạy `MENU_SENSITIVITY = 20` xung/bước) và nhấn nút để chọn.

```
--- MICROMOUSE MENU ---
> RUN
  THAY DOI TOA DO      ← bánh trái chỉnh Y, bánh phải chỉnh X
  THAY DOI THONG SO
      ├─ Speed (60–255)
      ├─ Pulse/Cell (200–800)
      ├─ CAI DAT PID...  (Kp/Kd Thẳng, Kp/Kd Quẹo, Kp/Kd Tường)
      └─ BACK TO MENU
```

Mọi giá trị được **lưu vào Flash** qua thư viện `EEPROM` của STM32 (có magic number `0x1234567A` để kiểm tra dữ liệu hợp lệ; lần đầu sẽ ghi giá trị mặc định).

| Tham số | Mặc định |
|---|---|
| Base speed | 130 |
| Pulses / cell | 445 |
| Kp_yaw / Kd_yaw | 12.0 / 2.5 |
| t_kp / t_kd | 3.5 / 0.5 |
| Kp_wall / Kd_wall | 8.0 / 3.0 |

## 5. Quy trình chạy
1. Bật nguồn → menu → chọn **THAY DOI TOA DO** để đặt ô đích (tính từ ô xuất phát, hướng Bắc là hướng xe đang nhìn).
2. Chọn **RUN** → xe khởi tạo cảm biến, hiệu chuẩn MPU (để yên) → bắt đầu **khám phá**.
3. Tới đích, OLED hiện `WAIT FAST RUN` → đặt xe về ô xuất phát, nhấn nút → xe hiệu chuẩn lại và **chạy nhanh**.
4. `DONE! PRESS BTN` → nhấn để quay lại menu.

## 6. Cải tiến so với ver1
- Laser ToF chính xác và ít nhiễu hơn siêu âm.
- Encoder giúp đi **đúng số ô** và cân bằng tốc độ 2 bánh.
- Flood Fill tìm được **đường ngắn nhất**, đích có thể nằm ở bất cứ đâu.
- Chỉnh thông số ngay trên xe, không cần nạp lại code.
