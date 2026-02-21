void setup() {
  Serial.begin(115200);
  // TODO: hiljem lisa BME280 või LPS22HB init
  Serial.println("Baro test (kohahoidja) start");
}

void loop() {
  float pressure_hpa = 1013.25; // merepinna ISA kohahoidja
  float altitude_m = 0.0;
  Serial.print("pressure_hpa="); Serial.print(pressure_hpa);
  Serial.print(", altitude_m="); Serial.println(altitude_m);
  delay(200);
}