void setup() {
  Serial.begin(115200);
  // TODO: hiljem lisa magnetomeetri teegi init (nt Wire.begin(); mag.begin();)
  Serial.println("Magnet test (kohahoidja) start");
}

void loop() {
  Serial.print("mag_x="); Serial.print(0);
  Serial.print(", mag_y="); Serial.print(0);
  Serial.print(", mag_z="); Serial.println(0);
  delay(100);
}