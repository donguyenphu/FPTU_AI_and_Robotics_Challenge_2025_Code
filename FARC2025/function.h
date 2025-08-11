void setServo(uint8_t channel, uint16_t pulse) {
  pwm.setPWM(channel, 0, pulse);
}

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