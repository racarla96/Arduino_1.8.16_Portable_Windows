#include <Wire.h>
#include "DFRobot_INA219.h"

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);
float DFRobot_INA219_Reading_mA = 1000;
float DFRobot_INA219_ExtMeter_Reading_mA = 1000;

float resolution = 4096;
float cte_voltage = 12;

float k_voltage;
float voltage;

void setup() {

  Serial.begin(115200);

  while (ina219.begin() != true) {}
  ina219.linearCalibrate(DFRobot_INA219_Reading_mA, DFRobot_INA219_ExtMeter_Reading_mA);

}

void loop() {

  Serial.print("BusVoltage: ");
  voltage = ina219.getBusVoltage_V();
  Serial.print(voltage, 2);
  Serial.print("V");
  Serial.print(" ");
  Serial.print("Voltage Constant Estimated: ");
  k_voltage = voltage / resolution;
  Serial.println(k_voltage, 6);

  Serial.print("BusVoltage: ");
  Serial.print(cte_voltage, 2);
  Serial.print("V");
  Serial.print(" ");
  Serial.print("Voltage Constant Estimated: ");
  k_voltage = cte_voltage / resolution;
  Serial.println(k_voltage, 6);

  Serial.println();

  delay(1000);

}
