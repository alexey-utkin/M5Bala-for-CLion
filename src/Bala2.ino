#define M5STACK_MPU6886

#include <M5Stack.h>
#undef min
#include <atomic>
#include <complex>

#include "MadgwickAHRS.h"
#include "bala.h"
#include "bala_img.h"
#include "bala_snd.h"
#include "calibration.h"
#include "freertos/FreeRTOS.h"
#include "imu_filter.h"
#include "pid.h"

constexpr int CORRECTION_TICKS = 5;

enum class MotionState : char {
  Stop = 0,
  Stabilize = 1,
  Move = 2
};

std::atomic<MotionState> STATE{MotionState::Stabilize};
std::atomic<int16_t> pwm_delta_lspeed{0};
std::atomic<int16_t> pwm_delta_rspeed{0};
std::atomic<int16_t> pwm_speed_mid{0};
std::atomic<int16_t> gStepNo{0};

struct Step {
  const char *name;
  int16_t left;
  int16_t right;
  int16_t duration;

  void showStep(const char *s) const {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print(s);
    M5.Lcd.println(name);
  }
};
std::vector<Step> dance = {
  {"Fw L",  5,  1,   100},
  {"Rv L", -5, -1,   100},
  {"Fw R",  1,  5,   100},
  {"Rv R", -1, -5,   100},
  {"Stop",  0,  0,   100},
  // {"Fw R",  1,  5,   100},
  // {"Rv R", -1, -5,   100},
  // {"Fw L",  5,  1,   100},
  // {"Rv L", -5, -1,   100},
  // {"Stop",  0,  0,   100},
};

template <typename T>
const T &clamp(const T &v, const T &minv, const T &maxv) {
  return std::max(minv, std::min(v, maxv));
}

void M5playMusic(const uint8_t* music_data, size_t music_len, uint16_t sample_rate, uint8_t volume, int t, int n) {
    uint16_t _delay_interval = ((uint32_t)1000000 / sample_rate);
    uint8_t  _volume        = 11 - volume;

    // fade out last sample value
    for (int i = t; i < music_len && i < t + n; i++) {
        dacWrite(SPEAKER_PIN, music_data[i] / _volume);
        delayMicroseconds(_delay_interval);
    }

    ledcAttachPin(SPEAKER_PIN, TONE_PIN_CHANNEL);
}

constexpr int16_t MAX_POWER = 1023;

void PIDTask(void *arg);
void PathTask(void *arg);
void draw_waveform();

static float angle_point = -1.5;

float kp = 24.0f, ki = 0.0f, kd = 90.0f;
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f;

bool calibration_mode = false;

Bala bala;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);

// the setup routine runs once when M5Stack starts up
void setup() {
  // Initialize the M5Stack object

  M5.begin(true, false, false, false);

  Serial.begin(115200);
  M5.IMU.Init();

  int16_t x_offset, y_offset, z_offset;
  float angle_center;
  calibrationInit();

  if (M5.BtnB.isPressed()) {
    calibrationGryo();
    calibration_mode = true;
  }

  if (M5.BtnC.isPressed()) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Charge mode");
    while (true) {
      if (M5.Power.isCharging()) {
        M5.Lcd.println("Start charging...");
        while (true) {
          if (M5.Power.isChargeFull())
            M5.Lcd.println("Charge completed!");
          delay(5000);
        }
      }
      delay(500);
    }
  }

  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
  Serial.printf("x: %d, y: %d, z:%d, angle: %.2f", x_offset, y_offset, z_offset,
                angle_center);

  angle_point = angle_center;
  pid.SetPoint(angle_point);

  M5.Lcd.setTextSize(4);

  SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
  bala.SetMutex(&i2c_mutex);
  ImuTaskStart(x_offset, y_offset, z_offset, &i2c_mutex);

  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(PathTask, "path_task", 4 * 1024, NULL, 2, NULL, 1);

  M5.Lcd.drawJpg(bala_img, sizeof(bala_img));
  if (calibration_mode) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("calibration mode");
  }
  M5.Speaker.setVolume(4);
}

int16_t smstep(double x, double s, int16_t minV, int16_t maxV) {
  double v1 = 1.0 / (1.0 + -std::exp(9.0 * (x - 0.5)));
  double v2 = x;
  if (x >= 1) {
    v2 = 1;
  } else if (x <= 0) {
    v2 = 0;
  }
  double v = s*v1 + (1-s)*v2;
  return minV + int16_t(double(maxV - minV) * v);
}

// the loop routine runs over and over again forever
void loop() {
  M5.update();

  Step step = dance[gStepNo % dance.size()];
  int16_t LCDw = M5.Lcd.width();
  int16_t LCDh = M5.Lcd.height();

  static uint32_t next_show_time = 0;
  if (millis() > next_show_time) {
    M5.Lcd.startWrite();
    draw_waveform();

    constexpr int EYE_SIZE = 20;
    constexpr int EYE_GAP = 100;
    constexpr int EYE_SIZE_IN = EYE_SIZE*0.65;
    constexpr int EYE_OFFSET_H = 50;
    constexpr int X_SCALE = 3;

    double angle_d = constrain((int16_t)(getAngle() * X_SCALE), 0, 100);
    double offset_s = double(std::sin(double(millis())/100.0)*0.5)+0.5;
    double speed_l = angle_d/100.0; //double(pwm_delta_lspeed)+double(pwm_speed_mid)*3.0+angle_d;
    double speed_r = angle_d/100.0; //double(pwm_delta_rspeed)+double(pwm_speed_mid)*3.0+angle_d;

    int16_t s_of = smstep(offset_s, 0, -EYE_SIZE+EYE_SIZE_IN, EYE_SIZE-EYE_SIZE_IN);
    int16_t l_of = smstep(speed_l, 0, -EYE_SIZE+EYE_SIZE_IN, EYE_SIZE-EYE_SIZE_IN);
    int16_t r_of = smstep(speed_r, 0, -EYE_SIZE+EYE_SIZE_IN, EYE_SIZE-EYE_SIZE_IN);

    M5.Lcd.fillCircle(LCDw/2-EYE_SIZE-EYE_GAP/2, LCDh/2+EYE_OFFSET_H, EYE_SIZE, TFT_WHITE);
    M5.Lcd.fillCircle(LCDw/2-EYE_SIZE-EYE_GAP/2+s_of, LCDh/2+EYE_OFFSET_H+l_of, EYE_SIZE*0.65, TFT_BLACK);

    M5.Lcd.fillCircle(LCDw/2+EYE_SIZE+EYE_GAP/2, LCDh/2+EYE_OFFSET_H, EYE_SIZE, TFT_WHITE);
    M5.Lcd.fillCircle(LCDw/2+EYE_SIZE+EYE_GAP/2-s_of, LCDh/2+EYE_OFFSET_H+r_of, EYE_SIZE*0.65, TFT_BLACK);
    M5.Lcd.endWrite();

    next_show_time = millis() + 15;
  }

  static uint32_t sound_t = 0;
  static bool sound_on = false;

  if (sound_on) {
    int sound_n = 100;
    M5playMusic(bala_snd, sizeof(bala_snd), BALA_SND_SAMPLE_RATE, 8, sound_t, sound_n);
    sound_t += sound_n;
    if (sound_t >= sizeof(bala_snd)) {
      sound_t = 0;
    }
  }
  else {
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (M5.BtnA.wasPressed()) {
    if (calibration_mode) {
      angle_point += 0.25;
      pid.SetPoint(angle_point);
    }
  }
  if (M5.BtnC.wasPressed()) {
    if (calibration_mode) {
        angle_point -= 0.25;
        pid.SetPoint(angle_point);
    } else {
      sound_on = !sound_on;
      if (!sound_on) {
        M5.Speaker.mute();
      }
    }
  }
  if (M5.BtnB.wasPressed()) {
    if (calibration_mode) {
      calibrationSaveCenterAngle(angle_point);
    } else {
      sound_t = 0;
      sound_on = true;
    }
  }
}

constexpr int ritmus = 10;
constexpr int strength = 20;

[[noreturn]] void PathTask(void *arg) {
  uint32_t last_ticks = 0;
  int step_no = 0;
  int step_count = dance.size();
  while (true) {
    gStepNo = step_no;
    Step step = dance[step_no % step_count];
    STATE = MotionState::Stabilize;
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(CORRECTION_TICKS * 10));

    STATE = MotionState::Move;

    pwm_delta_lspeed = step.left * strength * 2;
    pwm_delta_rspeed = step.right * strength * 2;
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(CORRECTION_TICKS * 10));

    pwm_delta_lspeed = step.left * strength;
    pwm_delta_rspeed = step.right * strength;
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(step.duration * ritmus));

    ++step_no;
  }
}

[[noreturn]] void PIDTask(void *arg) {
  pid.SetOutputLimits(MAX_POWER, -MAX_POWER);
  pid.SetDirection(-1);

  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(MAX_POWER, -MAX_POWER);
  speed_pid.SetDirection(1);

  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;
  float motor_speed = 0.0f;
  while (true) {
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(CORRECTION_TICKS));

    // in imu task update, update freq is 200HZ
    float bala_angle = getAngle();
    // Get motor encoder value
    bala.UpdateEncoder();

    int32_t encoder = bala.wheel_left_encoder + bala.wheel_right_encoder;

    // motor_speed filter
    motor_speed = 0.8f * motor_speed + 0.2f * (encoder - last_encoder);
    last_encoder = encoder;

    if (fabs(bala_angle) < 70) {
      int16_t pwm_angle = (int16_t)pid.Update(bala_angle);
      int16_t pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      int16_t pwm_output_base = pwm_speed + pwm_angle;
      pwm_speed_mid = pwm_output_base;

      if (STATE == MotionState::Stabilize) {
        bala.SetSpeed(clamp<int16_t>(pwm_output_base, -MAX_POWER, MAX_POWER),
                      clamp<int16_t>(pwm_output_base, -MAX_POWER, MAX_POWER));
      } else if (STATE == MotionState::Stop) {
      stop:
        motor_speed = 0;
        bala.SetSpeed(0, 0);
        bala.SetEncoder(0, 0);
        speed_pid.SetIntegral(0);
        STATE = MotionState::Stabilize;
      } else if (STATE == MotionState::Move) {
        bala.SetSpeed(clamp<int16_t>(pwm_delta_lspeed + pwm_output_base, -MAX_POWER, MAX_POWER),
                      clamp<int16_t>(pwm_delta_rspeed + pwm_output_base, -MAX_POWER, MAX_POWER));
      }
    } else {
      goto stop;
    }
  }
}

void draw_waveform() {
  constexpr int MAX_LEN = 120;
  constexpr int X_OFFSET = 100;
  constexpr int Y_OFFSET = 95;
  constexpr int X_SCALE = 3;

  static int16_t val_buf[MAX_LEN] = {0};
  static int16_t pt = MAX_LEN - 1;
  val_buf[pt] = constrain((int16_t)(getAngle() * X_SCALE), -50, 50);

  if (--pt < 0) {
    pt = MAX_LEN - 1;
  }

  for (int i = 1; i < MAX_LEN; i++) {
    uint16_t now_pt = (pt + i) % MAX_LEN;
    M5.Lcd.drawLine(i + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET,
                    i + 1 + X_OFFSET,
                    val_buf[(now_pt + 2) % MAX_LEN] + Y_OFFSET, TFT_BLACK);
    if (i < MAX_LEN - 1) {
      M5.Lcd.drawLine(i + X_OFFSET, val_buf[now_pt] + Y_OFFSET,
                      i + 1 + X_OFFSET,
                      val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, TFT_GREEN);
    }
  }
}
