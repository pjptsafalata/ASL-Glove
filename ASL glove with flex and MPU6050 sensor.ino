#include <Wire.h>

#define MPU_ADDR 0x68

int16_t ax, ay, az, gx, gy, gz;
int rest_ax = 0, rest_ay = 0, rest_az = 0;
int rest_gx = 0, rest_gy = 0, rest_gz = 0;
int rest_flex = 0;

String sentence = "";

// Timing for gesture hold
unsigned long gestureStartTime = 0;
bool inNeutralGesture = false;

// === Moving Average Filter Class ===
class MovingAverage {
  public:
    MovingAverage(int size = 20) {
      windowSize = size;
      index = 0;
      count = 0;
      buffer = new int16_t[size];
    }

    int16_t filter(int16_t newValue) {
      buffer[index] = newValue;
      index = (index + 1) % windowSize;
      if (count < windowSize) count++;

      long sum = 0;
      for (int i = 0; i < count; i++) {
        sum += buffer[i];
      }

      return sum / count;
    }

  private:
    int windowSize;
    int index;
    int count;
    int16_t* buffer;
};

// === Create filter objects for each axis ===
MovingAverage axFilter(20);
MovingAverage ayFilter(20);
MovingAverage azFilter(20);
MovingAverage gxFilter(20);
MovingAverage gyFilter(20);
MovingAverage gzFilter(20);

// =============================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println(" MPU6050 connected at 0x68");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);  // Wake up
    Wire.endTransmission();
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  delay(1000);
  Serial.println("Hold hand in resting position for calibration...");
  delay(3000);
  calibrateNeutralPoint();
}

// =============================
void loop() {
  readMPU();
  int flex = analogRead(A0);

  // Calculate normalized values (subtract resting baseline)
  int norm_ax = ax - rest_ax;
  int norm_ay = ay - rest_ay;
  int norm_az = az - rest_az;
  int norm_gx = gx - rest_gx;
  int norm_gy = gy - rest_gy;
  int norm_gz = gz - rest_gz;
  int norm_flex = flex - rest_flex;

  // Increased dead zone threshold to reduce fluctuations
  const int deadZone = 250;

  norm_ax = (abs(norm_ax) < deadZone) ? 0 : norm_ax;
  norm_ay = (abs(norm_ay) < deadZone) ? 0 : norm_ay;
  norm_az = (abs(norm_az) < deadZone) ? 0 : norm_az;
  norm_gx = (abs(norm_gx) < deadZone) ? 0 : norm_gx;
  norm_gy = (abs(norm_gy) < deadZone) ? 0 : norm_gy;
  norm_gz = (abs(norm_gz) < deadZone) ? 0 : norm_gz;
  norm_flex = (abs(norm_flex) < deadZone) ? 0 : norm_flex;

  // Print normalized and filtered sensor data
  Serial.print("| Flex: "); Serial.print(norm_flex);
  Serial.print(" | ax: "); Serial.print(norm_ax);
  Serial.print(" | ay: "); Serial.print(norm_ay);
  Serial.print(" | az: "); Serial.print(norm_az);
  Serial.print(" | gx: "); Serial.print(norm_gx);
  Serial.print(" | gy: "); Serial.print(norm_gy);
  Serial.print(" | gz: "); Serial.println(norm_gz);

  delay(200);

  checkForResetGesture(flex);

  if (inNeutralGesture) return;

  // Example Letter Recognition
  if (norm_ax > 900 && norm_ax < 7500 && norm_ay > -1000 && norm_ay < 1000 && norm_az < -1000) {
  appendLetter("A");
  } else if (norm_ax < -7000 && norm_az < -1500 && norm_gy > 1400 && norm_gy < 1700) {
    appendLetter("B");
  }

  if (norm_flex < -60 && abs(norm_ay) > 12000) {
    sendPhrase("THANK YOU");
  }

  delay(800);
}

// =============================
void checkForResetGesture(int flex) {
  bool palmDown = abs(ax) < 2000 && abs(ay + 16000) < 3000 && abs(az) < 3000;
  bool flatFlex = abs(flex - rest_flex) < 30;

  if (palmDown && flatFlex) {
    if (!inNeutralGesture) {
      gestureStartTime = millis();
      inNeutralGesture = true;
    } else {
      if (millis() - gestureStartTime > 4000) {
        Serial.println("🔄 Recalibrating neutral point...");
        calibrateNeutralPoint();
        inNeutralGesture = false;
      }
    }
  } else {
    inNeutralGesture = false;
  }
}

// =============================
void readMPU() {
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true); // Read accel + gyro

    sum_ax += Wire.read() << 8 | Wire.read();
    sum_ay += Wire.read() << 8 | Wire.read();
    sum_az += Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    sum_gx += Wire.read() << 8 | Wire.read();
    sum_gy += Wire.read() << 8 | Wire.read();
    sum_gz += Wire.read() << 8 | Wire.read();

    delay(5);
  }

  // Apply moving average filter here
  ax = axFilter.filter(sum_ax / 5);
  ay = ayFilter.filter(sum_ay / 5);
  az = azFilter.filter(sum_az / 5);
  gx = gxFilter.filter(sum_gx / 5);
  gy = gyFilter.filter(sum_gy / 5);
  gz = gzFilter.filter(sum_gz / 5);
}

// =============================
void calibrateNeutralPoint() {
  long sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0, sum_flex = 0;

  for (int i = 0; i < 50; i++) {
    readMPU();
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    sum_flex += analogRead(A0);
    delay(20);
  }

  rest_ax = sum_ax / 50;
  rest_ay = sum_ay / 50;
  rest_az = sum_az / 50;
  rest_gx = sum_gx / 50;
  rest_gy = sum_gy / 50;
  rest_gz = sum_gz / 50;
  rest_flex = sum_flex / 50;

  Serial.println("✅ Calibrated neutral point:");
  Serial.print("| flex: "); Serial.println(rest_flex);
  Serial.print(" | Δax: "); Serial.print(ax - rest_ax);
  Serial.print(" | Δay: "); Serial.print(ay - rest_ay);
  Serial.print(" | Δaz: "); Serial.println(az - rest_az);
  Serial.print(" | Δgx: "); Serial.print(gx - rest_gx);
  Serial.print(" | Δgy: "); Serial.print(gy - rest_gy);
  Serial.print(" | Δgz: "); Serial.println(gz - rest_gz);
  Serial.println();
}

// =============================
void appendLetter(String letter) {
  sentence += letter;
  Serial.print("📩 Letter: "); Serial.println(letter);
  Serial.print("📜 Sentence: "); Serial.println(sentence);
  delay(300);
}

void sendPhrase(String phrase) {
  Serial.print("💬 Phrase: "); Serial.println(phrase);
  sentence = "";
  delay(1000);
}
