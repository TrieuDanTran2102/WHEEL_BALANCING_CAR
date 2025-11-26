🛞 WHEEL BALANCING CAR – Xe Tự Cân Bằng 2 Bánh (STM32F407)

Xe tự cân bằng hai bánh sử dụng vi điều khiển STM32F407, cảm biến MPU6050, động cơ DC điều khiển qua L298N, nguồn hạ áp Buck Converter và mô-đun ESP32 hỗ trợ điều khiển không dây. Hệ thống có khả năng tự giữ thăng bằng bằng cách đo góc nghiêng và điều chỉnh tốc độ động cơ theo thời gian thực.

📌 1. Tính năng nổi bật (Features)

🚀 Tự giữ thăng bằng dựa trên đo đạc từ MPU6050.

🎛 Điều khiển động cơ DC bằng thuật toán PID.

🔁 Thuật toán lọc kết hợp (Complementary Filter hoặc Kalman) để ổn định góc.

🔋 Bộ nguồn ổn định với Buck Converter cho STM32, MPU6050 và ESP32.

📡 Điều khiển không dây qua ESP32 (WiFi): xem góc nghiêng, tốc độ motor, chỉnh PID.

🔄 Xe có khả năng chạy tiến–lùi–quay bằng điều khiển từ xa.

⚙ Tốc độ phản hồi nhanh, hoạt động ổn định.

📈 Xuất dữ liệu real-time để debug (UART hoặc WiFi).

🔧 2. Phần cứng sử dụng (Hardware)
Linh kiện	Mô tả
STM32F407VET6	Vi điều khiển chính, xử lý PID và đọc sensor
MPU6050	Cảm biến đo góc nghiêng (Gyro + Accelerometer)
Động cơ DC + Encoder (nếu có)	Tạo lực cân bằng
Driver L298N	Điều khiển tốc độ & chiều quay động cơ
Buck Converter	Hạ áp từ 12V xuống 5V / 3.3V
ESP32	Điều khiển WiFi, giao tiếp UART với STM32
Pin 18650 / 12V	Nguồn cho toàn hệ thống
Khung xe + bánh + trục	Phần cơ khí

4.	Sơ đồ nối mạch quay động cơ DC 
<img width="975" height="683" alt="image" src="https://github.com/user-attachments/assets/2d7b3d0a-c3a4-4173-9960-d3665685e991" />
