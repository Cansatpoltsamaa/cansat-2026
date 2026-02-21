void setup() {
  Serial.begin(115200);
  // TODO: hiljem lisa IMU init
  Serial.println("IMU test (kohahoidja) start");
}

void loop() {
  Serial.print("acc_x="); Serial.print(0);
  Serial.print(", acc_y="); Serial.print(0);
  Serial.print(", acc_z="); Serial.print(1); // ~1g kohahoidja
  Serial.print(", gyro_x="); Serial.print(0);
  Serial.print(", gyro_y="); Serial.print(0);
  Serial.print(", gyro_z="); Serial.println(0);
  delay(50);
}