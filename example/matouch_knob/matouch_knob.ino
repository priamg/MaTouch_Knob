#include <SimpleFOC.h>
#include <Wire.h>
#include <EEPROM.h>
#include <USB.h>
#include <USBHID.h>
#include "interface.h"

// Define SPI pins for MT6701
#define MT6701_SDA 1
#define MT6701_SCL 2
#define MT6701_SS 42

// SPI Sensor Initialization
SPIClass *hspi = nullptr;
GenericSensor sensor(
    []() -> float {
      hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
      digitalWrite(hspi->pinSS(), LOW);
      uint16_t ag = hspi->transfer16(0);
      digitalWrite(hspi->pinSS(), HIGH);
      hspi->endTransaction();
      ag = ag >> 2;
      float rad = (float)ag * 2 * PI / 16384;
      return (rad < 0) ? rad + 2 * PI : rad;
    },
    []() {
      hspi = new SPIClass(HSPI);
      hspi->begin(MT6701_SCL, MT6701_SDA, -1, MT6701_SS);
      pinMode(hspi->pinSS(), OUTPUT);
    });

// Motor setup
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(17, 16, 15);

// Knob Configuration
struct KnobConfig {
  int32_t num_positions;
  int32_t position;
  float position_width_radians;
  float detent_strength_unit;
  float endstop_strength_unit;
  float snap_point;
};
KnobConfig config = {
    .num_positions = 0,
    .position = 0,
    .position_width_radians = 1 * _PI / 180,
    .detent_strength_unit = 1,
    .endstop_strength_unit = 1,
    .snap_point = 1.1,
};

float current_detent_center = 0.0;

// Initialize FOC
void initFOC() {
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 5;
  driver.pwm_frequency = 50000;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.PID_velocity.P = 2;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0.08;
  motor.PID_velocity.output_ramp = 10000;
  motor.PID_velocity.limit = 10;

  motor.voltage_limit = 5;
  motor.LPF_velocity.Tf = 0.01;
  motor.velocity_limit = 40;

  motor.init();
  motor.initFOC();

  current_detent_center = motor.shaft_angle;
  Serial.println("Motor ready.");
}

// Main motor loop
void motorLoop() {
  float angle_to_detent_center = motor.shaft_angle - current_detent_center;

  if (angle_to_detent_center > config.position_width_radians * config.snap_point) {
    current_detent_center += config.position_width_radians;
    angle_to_detent_center -= config.position_width_radians;
    config.position--;
  } else if (angle_to_detent_center < -config.position_width_radians * config.snap_point) {
    current_detent_center -= config.position_width_radians;
    angle_to_detent_center += config.position_width_radians;
    config.position++;
  }

  float dead_zone_adjustment = constrain(
      angle_to_detent_center,
      -config.position_width_radians * 0.2,
      config.position_width_radians * 0.2);

  motor.PID_velocity.P = config.detent_strength_unit * 4;
  float torque = motor.PID_velocity(-angle_to_detent_center + dead_zone_adjustment);
  motor.move(torque);
  motor.loopFOC();
}

void setup() {
  Serial.begin(115200);

  initFOC();

  Serial.println("Motor and HID ready.");
}

void loop() {
  motorLoop();
}
