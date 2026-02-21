void setup() {
  Serial.begin(115200);
  Serial.println("Termistor test (kohahoidja) start");
}

void loop() {
  static int val = 0;
  Serial.print("temp_ext_raw="); Serial.print(val++);
  Serial.print(", temp_ext_c="); Serial.println(20.0);
  delay(500);
}