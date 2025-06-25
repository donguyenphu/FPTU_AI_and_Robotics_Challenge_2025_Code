#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

/******************************************************************
 * Cài đặt chân cho thư viện :
 * - Trên mạch Motorshield của VIA Makerbot BANHMI, có header 6 chân
 *   được thiết kế để cắm tay cầm PS2.
 * Sơ đồ chân header và sơ đồ GPIO tương ứng:
 *   MOSI | MISO | GND | 3.3V | CS | CLK
 *    12     13    GND   3.3V   15   14
 ******************************************************************/

#define PS2_DAT 12  // MISO
#define PS2_CMD 13  // MOSI
#define PS2_SEL 15  // SS
#define PS2_CLK 14  // SLK

#define SERVO_1_CHANNEL 2
#define SERVO_2_CHANNEL 3
#define SERVO_3_CHANNEL 4
#define SERVO_4_CHANNEL 5
#define SERVO_5_CHANNEL 6
#define SERVO_6_CHANNEL 7

#define MOTOR_1_CHANNEL_A 8
#define MOTOR_1_CHANNEL_B 9
#define MOTOR_2_CHANNEL_A 10
#define MOTOR_2_CHANNEL_B 11
#define MOTOR_3_CHANNEL_A 12
#define MOTOR_3_CHANNEL_B 13
#define MOTOR_4_CHANNEL_A 14
#define MOTOR_4_CHANNEL_B 15

#define MAX_MOTOR 4095
#define MIN_MOTOR 0

#define ANGLE_0 205
#define ANGLE_90 307
#define ANGLE_180 410

#define CONVERT_PWM_RATIO 10

/******************************************************************
 * Lựa chọn chế độ cho tay cầm PS2 :
 *   - pressures = đọc giá trị analog từ các nút bấm
 *   - rumble    = bật/tắt chế độ rung
 ******************************************************************/
#define pressures false
#define rumble true

PS2X ps2x;                                                // khởi tạo class PS2x
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // khởi tạo class Servo

//--------------- Khai báo hàm --------------
void setServo(uint8_t channel, uint16_t pulse) {
  pwm.setPWM(channel, 0, pulse);
}
//----------------------
void drivertrain() {
  unsigned char JSL = ps2x.Analog(PSS_LY);
  unsigned char JSR = ps2x.Analog(PSS_RY);
  unsigned char ratio = 5;
  if ((JSL >= 130) && (JSL <= 255)) {
    pwm.setPWM(MOTOR_1_CHANNEL_A, 0, ratio * JSL);
    pwm.setPWM(MOTOR_1_CHANNEL_B, 0, 0);
  } else if ((JSL >= 0) && (JSL <= 124)) {
    pwm.setPWM(MOTOR_1_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_1_CHANNEL_B, 0, ratio * (255 - JSL));
  } else {
    pwm.setPWM(MOTOR_1_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_1_CHANNEL_B, 0, 0);
  }

  if ((JSR >= 130) && (JSR <= 255)) {
    pwm.setPWM(MOTOR_2_CHANNEL_A, 0, ratio * JSR);
    pwm.setPWM(MOTOR_2_CHANNEL_B, 0, 0);
  } else if ((JSR >= 0) && (JSR <= 124)) {
    pwm.setPWM(MOTOR_2_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_2_CHANNEL_B, 0, ratio * (255 - JSR));
  } else {
    pwm.setPWM(MOTOR_2_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_2_CHANNEL_B, 0, 0);
  }
}

void setMotors(int forwardChannel, int backChannel, int speed) {
  if (speed > 0) {
    pwm.setPWM(forwardChannel, 0, speed);
    pwm.setPWM(backChannel, 0, 0);
  } else if (speed < 0) {
    pwm.setPWM(forwardChannel, 0, 0);
    pwm.setPWM(backChannel, 0, -speed);
  } else {
    pwm.setPWM(forwardChannel, 0, 0);
    pwm.setPWM(backChannel, 0, 0);
  }
}

void driveBase() {
  int JLX = ps2x.Analog(PSS_LX) - 127;
  int JLY = ps2x.Analog(PSS_LY) - 128;
  int leftValue = JLX + JLY;
  int rightValue = JLY - JLX;
  setMotors(MOTOR_1_CHANNEL_A, MOTOR_1_CHANNEL_B, leftValue * CONVERT_PWM_RATIO);
  setMotors(MOTOR_2_CHANNEL_B, MOTOR_2_CHANNEL_A, rightValue * CONVERT_PWM_RATIO);
}

int stateUpHang = 0;
int stateDownHang = 0;
bool recentUpHang = false;
bool recentDownHang = false;

void slideHang() {
  /// up mot 3 A
  bool currentUpHang = ps2x.Button(PSB_R1);
  if (currentUpHang && !recentUpHang) {
    stateUpHang = 1 - stateUpHang;
    pwm.setPWM(MOTOR_3_CHANNEL_A, 0, stateUpHang * MAX_MOTOR);
    pwm.setPWM(MOTOR_3_CHANNEL_B, 0, 0);
  }
  recentUpHang = currentUpHang;
  /// down mot 3 B
  bool currentDownHang = ps2x.Button(PSB_L1);
  if (currentDownHang && !recentDownHang) {
    stateDownHang = 1 - stateDownHang;
    pwm.setPWM(MOTOR_3_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_3_CHANNEL_B, 0, stateDownHang * MAX_MOTOR);
  }
  recentDownHang = currentDownHang;
}

int stateUpArm = 0;
int stateDownArm = 0;
bool recentUpArm = false;
bool recentDownArm = false;


void slideArm() {
  /// up mot 4 A
  // bool currentUpArm = ps2x.Button(PSB_R2);
  // if (currentUpArm && !recentUpArm) {
  //   stateUpArm = 1 - stateUpArm;
  //   pwm.setPWM(MOTOR_4_CHANNEL_A, 0, stateUpArm * MAX_MOTOR);
  //   pwm.setPWM(MOTOR_4_CHANNEL_B, 0, 0);
  // }
  // recentUpArm = currentUpArm;
  /// down mot 4 B
  // bool currentDownArm = ps2x.Button(PSB_L2);
  // if (currentDownArm && !recentDownArm) {
  //   stateDownArm = 1 - stateDownArm;
  //   pwm.setPWM(MOTOR_4_CHANNEL_A, 0, 0);
  //   pwm.setPWM(MOTOR_4_CHANNEL_B, 0, stateDownArm * MAX_MOTOR);
  // }
  // recentDownArm = currentDownArm;
  int value = ps2x.Analog(PSS_RY);
  if (value < 128) {
    pwm.setPWM(MOTOR_3_CHANNEL_B, 0, (255 - value) * 10);
    pwm.setPWM(MOTOR_3_CHANNEL_A, 0, 0);
  }
  else if (value > 128) {
    pwm.setPWM(MOTOR_3_CHANNEL_B, 0, 0);
    pwm.setPWM(MOTOR_3_CHANNEL_A, 0, value * 10);
  }
  else if (value == 128) {
    pwm.setPWM(MOTOR_3_CHANNEL_A, 0, 0);
    pwm.setPWM(MOTOR_3_CHANNEL_B, 0, 0);
  }
}

/// for 180

bool recentOpenBall12 = false;
int stateOpenBall12 = 0;

void openBall() {
  bool currentOpenBall = ps2x.Button(PSB_TRIANGLE);
  if (currentOpenBall && !recentOpenBall12) {
    stateOpenBall12 = 1 - stateOpenBall12;
    if (stateOpenBall12 == 0) {
      pwm.setPWM(SERVO_1_CHANNEL, 0, 205);
      pwm.setPWM(SERVO_2_CHANNEL, 0, 205);

    }
    else if (stateOpenBall12 == 1) {
      pwm.setPWM(SERVO_1_CHANNEL, 0, 510);
      pwm.setPWM(SERVO_2_CHANNEL, 0, 510);
    }
  }
  recentOpenBall12 = currentOpenBall;
}

// for 360

bool recentOpenLit = false;
int stateOpenLit = 2; ///  0 : stop, 1 : forward, 2: backward
/// 370 : stop
/// > 370 : clockwise
/// < 370 : counterclockwise
void openLit() {
  bool currentOpenLit = ps2x.Button(PSB_CIRCLE);
  if (currentOpenLit && !recentOpenLit) {
    stateOpenLit = (stateOpenLit + 1) % 3;
    if (stateOpenLit == 0) {
      pwm.setPWM(SERVO_3_CHANNEL, 0, 320);
      pwm.setPWM(SERVO_4_CHANNEL, 0, 320);
      Serial.println(stateOpenLit);
    }
    else if (stateOpenLit == 1) {
      pwm.setPWM(SERVO_3_CHANNEL, 0, 420);
      pwm.setPWM(SERVO_4_CHANNEL, 0, 220);
      Serial.println(stateOpenLit);

    }
    else if (stateOpenLit == 2) {
      pwm.setPWM(SERVO_3_CHANNEL, 0, 220);
      pwm.setPWM(SERVO_4_CHANNEL, 0, 420);
      Serial.println(stateOpenLit);
    }
  }
  recentOpenLit = currentOpenLit;
}


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

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))  // các trả về giá trị TRUE khi nút được giữ
  {                                                // Đọc giá trị 2 joystick khi nút L1 hoặc R1 được giữ
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

  // drivertrain();
  driveBase();
  slideArm();
  // slideHang();
  openBall();
  // openBall2();
  openLit();
  // openBall4();
}