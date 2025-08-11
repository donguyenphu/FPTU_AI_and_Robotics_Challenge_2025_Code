#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <components/define.h>
#include <components/function.h>
/******************************************************************
 * Cài đặt chân cho thư viện :
 * - Trên mạch Motorshield của VIA Makerbot BANHMI, có header 6 chân
 *   được thiết kế để cắm tay cầm PS2.
 * Sơ đồ chân header và sơ đồ GPIO tương ứng:
 *   MOSI | MISO | GND | 3.3V | CS | CLK
 *    12     13    GND   3.3V   15   14
 ******************************************************************/
/******************************************************************
 * Lựa chọn chế độ cho tay cầm PS2 :
 *   - pressures = đọc giá trị analog từ các nút bấm
 *   - rumble    = bật/tắt chế độ rung
 ******************************************************************/

PS2X ps2x;                                                // khởi tạo class PS2x
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // khởi tạo class Servo

//--------------- Khởi tạo giá trị -------------
void setup() {
  Serial.begin(115200);
  Serial.print("Ket noi voi tay cam PS2:");
  pwm.begin();
  pwm.setPWMFreq(50);
  // Stop all motor
  pwm.setPin(MOTOR_1_CHANNEL_A, 0);
  pwm.setPin(MOTOR_1_CHANNEL_B, 0);
  pwm.setPin(MOTOR_2_CHANNEL_A, 0);
  pwm.setPin(MOTOR_2_CHANNEL_B, 0);
  pwm.setPin(MOTOR_3_CHANNEL_A, 0);
  pwm.setPin(MOTOR_3_CHANNEL_B, 0);
  pwm.setPin(MOTOR_4_CHANNEL_A, 0);
  pwm.setPin(MOTOR_4_CHANNEL_B, 0);
  // Init all servo
  pwm.setPWM(SERVO_1_CHANNEL, 0, 205);
  pwm.setPWM(SERVO_2_CHANNEL, 0, 205);
  pwm.setPWM(SERVO_3_CHANNEL, 0, 320);
  pwm.setPWM(SERVO_4_CHANNEL, 0, 320);
  pwm.setPWM(SERVO_5_CHANNEL, 0, 205);
  pwm.setPWM(SERVO_6_CHANNEL, 0, 205);


  int error = -1;
  for (int i = 0; i < 5; i++)  // thử kết nối với tay cầm ps2 trong 5 lần
  {
    delay(1000);  // đợi 1 giây
    // cài đặt chân và các chế độ: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
  }

  switch (error)  // kiểm tra lỗi nếu sau 10 lần không kết nối được
  {
    case 0:
      Serial.println(" Ket noi tay cam PS2 thanh cong");
      break;
    case 1:
      Serial.println(" LOI: Khong tim thay tay cam, hay kiem tra day ket noi vơi tay cam ");
      break;
    case 2:
      Serial.println(" LOI: khong gui duoc lenh");
      break;
    case 3:
      Serial.println(" LOI: Khong vao duoc Pressures mode ");
      break;
  }
}

// Chương trình chính
void loop() {
  ps2x.read_gamepad(false, false);  // gọi hàm để đọc tay điều khiển

  // các trả về giá trị TRUE (1) khi nút được giữ
  if (ps2x.Button(PSB_START))  // nếu nút Start được giữ, in ra Serial monitor
    Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT))  // nếu nút Select được giữ, in ra Serial monitor
    Serial.println("Select is being held");

  if (ps2x.Button(PSB_PAD_UP))  // tương tự như trên kiểm tra nút Lên (PAD UP)
  {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);  // đọc giá trị analog ở nút này, xem nút này được bấm mạnh hay nhẹ
  }
  if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
  }
  if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
  }
  if (ps2x.NewButtonState()) {  // Trả về giá trị TRUE khi nút được thay đổi trạng thái (bật sang tắt, or tắt sang bật)
    if (ps2x.Button(PSB_L3))
      Serial.println("L3 pressed");
    if (ps2x.Button(PSB_R3))
      Serial.println("R3 pressed");
    if (ps2x.Button(PSB_L2))
      Serial.println("L2 pressed");
    if (ps2x.Button(PSB_R2))
      Serial.println("R2 pressed");
    if (ps2x.Button(PSB_TRIANGLE))
      Serial.println("△ pressed");
  }
  //△□○×
  if (ps2x.ButtonPressed(PSB_CIRCLE))  // Trả về giá trị TRUE khi nút được ấn (từ tắt sang bật)
    Serial.println("○ just pressed");
  if (ps2x.NewButtonState(PSB_CROSS))  // Trả về giá trị TRUE khi nút được thay đổi trạng thái
    Serial.println("× just changed");
  if (ps2x.ButtonReleased(PSB_SQUARE))  //  Trả về giá trị TRUE khi nút được ấn (từ tắt sang bật)
    Serial.println("□ just released");

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { // các trả về giá trị TRUE khi nút được giữ
    // Đọc giá trị 2 joystick khi nút L1 hoặc R1 được giữ
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY));  // đọc trục Y của joystick bên trái. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }
  delay(50);

  driveBase();
  slideArm();
  openBall();
  openLit();
}