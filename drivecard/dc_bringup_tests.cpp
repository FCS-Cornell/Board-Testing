#include <Arduino.h>
#include <mbed.h>
#include <stdarg.h>
 
static constexpr int PIN_EN = 6;
static constexpr int PIN_PH = 5;
static constexpr int PIN_NFAULT = 4;
static constexpr int PIN_SO = A0;
 
static constexpr int ADC_BITS = 12;
static constexpr float ADC_VREF_V = 3.3f;
static constexpr float DRV_VREF_V = 3.3f;
static constexpr float SENSE_GAIN = 20.0f;
static constexpr float RSENSE_OHM = 0.010f;
 
static constexpr float ZERO_TOL_A = 0.25f;
static constexpr float MIN_LOADED_A = 0.05f;
static constexpr float ADC_RAIL_MARGIN_V = 0.05f;
 
static constexpr bool MOTOR_CONNECTED = true;
static constexpr uint32_t DEFAULT_FREQ_HZ = 20000;
static constexpr uint32_t HOLD_MS = 1000;
 
enum class Result { PASS, FAIL, MANUAL };
using TestFn = Result (*)();
 
struct TestEntry {
  const char *name;
  TestFn fn;
  bool needs_motor;
};
 
struct DriveStats {
  bool fault;
  float i_mean;
  float i_peak;
  float v_min;
  float v_max;
};
 
static mbed::PwmOut *g_pwm = nullptr;
 
static void say(const char *fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}
 
static void drv_init() {
  pinMode(PIN_PH, OUTPUT);
  pinMode(PIN_NFAULT, INPUT_PULLUP);
  digitalWrite(PIN_PH, LOW);
  g_pwm = new mbed::PwmOut(digitalPinToPinName(PIN_EN));
  g_pwm->write(0.0f);
  analogReadResolution(ADC_BITS);
}
 
static void drv_brake() { g_pwm->write(0.0f); }
 
static void set_dir(bool fwd) { digitalWrite(PIN_PH, fwd ? HIGH : LOW); }
 
static void set_pwm(float duty, uint32_t freq_hz) {
  g_pwm->period(1.0f / (float)freq_hz);
  g_pwm->write(duty);
}
 
static bool fault_asserted() { return digitalRead(PIN_NFAULT) == LOW; }
 
static float read_so_volts(int n = 1) {
  uint32_t sum = 0;
  for (int i = 0; i < n; i++) sum += analogRead(PIN_SO);
  return ((float)sum / n) * ADC_VREF_V / (float)((1 << ADC_BITS) - 1);
}
 
static float volts_to_amps(float v) {
  return (v - DRV_VREF_V / 2.0f) / (SENSE_GAIN * RSENSE_OHM);
}
 
static DriveStats drive_hold(bool fwd, float duty, uint32_t freq_hz, uint32_t ms) {
  DriveStats s = {false, 0.0f, 0.0f, 99.0f, -99.0f};
  double sum = 0.0;
  uint32_t n = 0;
  set_dir(fwd);
  set_pwm(duty, freq_hz);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (fault_asserted()) {
      s.fault = true;
      break;
    }
    float v = read_so_volts();
    float i = volts_to_amps(v);
    sum += i;
    n++;
    if (fabsf(i) > s.i_peak) s.i_peak = fabsf(i);
    if (v < s.v_min) s.v_min = v;
    if (v > s.v_max) s.v_max = v;
  }
  drv_brake();
  if (n) s.i_mean = (float)(sum / n);
  return s;
}
 
static Result run_duty_sweep(bool fwd) {
  static const float duties[] = {0.10f, 0.25f, 0.50f, 0.75f, 1.00f};
  say("  direction: %s, pwm freq: %lu Hz", fwd ? "forward (PH=1)" : "reverse (PH=0)",
      (unsigned long)DEFAULT_FREQ_HZ);
  for (float d : duties) {
    DriveStats s = drive_hold(fwd, d, DEFAULT_FREQ_HZ, HOLD_MS);
    say("  duty %3.0f%%: I_mean = %.3f A, I_peak = %.3f A", d * 100.0f, s.i_mean, s.i_peak);
    if (s.fault) {
      say("  nFAULT asserted at duty %.0f%%", d * 100.0f);
      return Result::FAIL;
    }
    delay(300);
  }
  say("  automated: nFAULT stayed high. Confirm direction and speed visually.");
  return Result::PASS;
}
 
static Result test_nfault_idle() {
  uint32_t t0 = millis();
  while (millis() - t0 < 200) {
    if (fault_asserted()) {
      say("  nFAULT low while idle");
      return Result::FAIL;
    }
  }
  say("  nFAULT high for 200 ms");
  return Result::PASS;
}
 
static Result test_sense_zero_offset() {
  drv_brake();
  delay(50);
  float vmin = 99.0f, vmax = -99.0f;
  double sum = 0.0;
  const int n = 256;
  for (int i = 0; i < n; i++) {
    float v = read_so_volts();
    sum += v;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  float v = (float)(sum / n);
  float i = volts_to_amps(v);
  say("  SO mean = %.3f V (expected %.3f V), min/max = %.3f/%.3f V", v, DRV_VREF_V / 2.0f, vmin,
      vmax);
  say("  zero-current reading = %.3f A (tolerance %.2f A)", i, ZERO_TOL_A);
  return fabsf(i) <= ZERO_TOL_A ? Result::PASS : Result::FAIL;
}
 
static Result test_pwm_duty_sweep_forward() { return run_duty_sweep(true); }
 
static Result test_pwm_duty_sweep_reverse() { return run_duty_sweep(false); }
 
static Result test_pwm_frequency_sweep() {
  static const uint32_t freqs[] = {1000, 5000, 10000, 20000, 50000, 100000};
  for (uint32_t f : freqs) {
    say("  pwm freq: %lu Hz, duty 50%%, forward", (unsigned long)f);
    DriveStats s = drive_hold(true, 0.5f, f, HOLD_MS);
    say("  I_mean = %.3f A, I_peak = %.3f A", s.i_mean, s.i_peak);
    if (s.fault) {
      say("  nFAULT asserted at %lu Hz", (unsigned long)f);
      return Result::FAIL;
    }
    delay(300);
  }
  say("  automated: nFAULT stayed high. Listen/scope for audible or waveform issues.");
  return Result::PASS;
}
 
static Result test_direction_reversal() {
  say("  forward 50%% -> brake -> reverse 50%% -> brake, pwm freq: %lu Hz",
      (unsigned long)DEFAULT_FREQ_HZ);
  DriveStats s = drive_hold(true, 0.5f, DEFAULT_FREQ_HZ, HOLD_MS);
  if (s.fault) {
    say("  nFAULT asserted during forward");
    return Result::FAIL;
  }
  delay(500);
  s = drive_hold(false, 0.5f, DEFAULT_FREQ_HZ, HOLD_MS);
  if (s.fault) {
    say("  nFAULT asserted during reverse");
    return Result::FAIL;
  }
  say("  confirm motor reversed direction");
  return Result::MANUAL;
}
 
static Result test_brake() {
  set_dir(true);
  set_pwm(0.5f, DEFAULT_FREQ_HZ);
  delay(1500);
  if (fault_asserted()) {
    say("  nFAULT asserted during spin-up");
    drv_brake();
    return Result::FAIL;
  }
  say("  brake (EN low): motor should stop quickly");
  drv_brake();
  delay(2000);
  return Result::MANUAL;
}
 
static Result test_current_sense_under_load() {
  say("  forward 50%%, pwm freq: %lu Hz", (unsigned long)DEFAULT_FREQ_HZ);
  DriveStats s = drive_hold(true, 0.5f, DEFAULT_FREQ_HZ, HOLD_MS);
  say("  I_mean = %.3f A, I_peak = %.3f A, SO range = %.3f-%.3f V", s.i_mean, s.i_peak, s.v_min,
      s.v_max);
  if (s.fault) {
    say("  nFAULT asserted");
    return Result::FAIL;
  }
  bool saturated = s.v_min < ADC_RAIL_MARGIN_V || s.v_max > ADC_VREF_V - ADC_RAIL_MARGIN_V;
  bool detected = s.i_mean >= MIN_LOADED_A;
  if (saturated) say("  SO near ADC rail");
  if (!detected) say("  mean current below %.2f A", MIN_LOADED_A);
  return (!saturated && detected) ? Result::PASS : Result::FAIL;
}
 
static Result test_scope_hold_for_probing() {
  say("  forward, duty 30%%, pwm freq: %lu Hz, 10 s", (unsigned long)DEFAULT_FREQ_HZ);
  say("  probe phase nodes, gate signals, and VM");
  DriveStats s = drive_hold(true, 0.3f, DEFAULT_FREQ_HZ, 10000);
  if (s.fault) {
    say("  nFAULT asserted");
    return Result::FAIL;
  }
  return Result::MANUAL;
}
 
#define TEST(fn, needs_motor) {#fn, fn, needs_motor}
 
static const TestEntry TESTS[] = {
    TEST(test_nfault_idle, false),
    TEST(test_sense_zero_offset, false),
    TEST(test_pwm_duty_sweep_forward, true),
    TEST(test_pwm_duty_sweep_reverse, true),
    TEST(test_pwm_frequency_sweep, true),
    TEST(test_direction_reversal, true),
    TEST(test_brake, true),
    TEST(test_current_sense_under_load, true),
    TEST(test_scope_hold_for_probing, false),
};
 
static void run_tests() {
  int pass = 0, fail = 0, manual = 0, skip = 0;
  say("=== DRV8701 bring-up tests ===");
 
  for (const TestEntry &t : TESTS) {
    say("");
    say("[RUN ] %s", t.name);
    if (t.needs_motor && !MOTOR_CONNECTED) {
      say("[SKIP] %s: no motor connected", t.name);
      skip++;
      continue;
    }
    drv_brake();
    Result r = t.fn();
    drv_brake();
    switch (r) {
      case Result::PASS:
        say("[PASS] %s", t.name);
        pass++;
        break;
      case Result::FAIL:
        say("[FAIL] %s", t.name);
        fail++;
        if (fault_asserted()) {
          say("nFAULT still asserted; power cycle the driver and rerun");
          say("=== Aborted: %d pass, %d fail, %d manual, %d skipped ===", pass, fail, manual, skip);
          return;
        }
        break;
      case Result::MANUAL:
        say("[MANUAL] %s: verify by observation", t.name);
        manual++;
        break;
    }
    delay(500);
  }
 
  say("");
  say("=== Summary: %d pass, %d fail, %d manual, %d skipped ===", pass, fail, manual, skip);
}
 
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 5000) {
  }
  drv_init();
  run_tests();
  drv_brake();
}
 
void loop() {}
