#include <TFMPlus.h>
#undef SOFT_RESET
#include <VL53L1X.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// ================== TFMini Plus ==================
TFMPlus tfmini;
SoftwareSerial TFSerial(2, 3); // RX, TX

int16_t tf_dist = 0;
int16_t tf_strength = 0;
int16_t tf_temp = 0;

// ================== VL53L1X ==================
VL53L1X vl53;

// ================== Timing ==================
unsigned long lastPrint = 0;
const unsigned long PRINT_PERIOD = 200; // ms

void setup() {
  Serial.begin(9600);
  Serial.println("Starting sensors...");

  // ---- TFMini ----
  TFSerial.begin(115200);
  tfmini.begin(&TFSerial);

  // ---- VL53L1X ----
  Wire.begin();
  Wire.setClock(400000);

  vl53.setTimeout(500);
  if (!vl53.init()) {
    Serial.println("VL53L1X init failed!");
    while (1);
  }

  vl53.setDistanceMode(VL53L1X::Long);
  vl53.setMeasurementTimingBudget(50000);
  vl53.startContinuous(50);

  Serial.println("Sensors started.");
}

void loop() {

  // ====== Leer TFMini ======
  bool tf_ok = tfmini.getData(tf_dist, tf_strength, tf_temp);

  // ====== Leer VL53 ======
  uint16_t vl_dist = vl53.read();
  bool vl_timeout = vl53.timeoutOccurred();

  // ====== Salida periódica ======
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_PERIOD) {
    lastPrint = now;

    Serial.print("[TFMini] ");
    if (tf_ok) {
      Serial.print("Dist: ");
      Serial.print(tf_dist);
      Serial.print(" cm | Strength: ");
      Serial.print(tf_strength);
    } else {
      Serial.print("No data");
    }

    Serial.print(" || [VL53L1X] Dist: ");
    Serial.print(vl_dist);
    Serial.print(" mm");
    if (vl_timeout) Serial.print(" TIMEOUT");

    Serial.println();
  }
}
