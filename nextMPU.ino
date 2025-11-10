#include <Wire.h>
#include <MPU9250_asukiaaa.h>

// XIAO ESP32S3 I2C pins
#define SDA_PIN 5
#define SCL_PIN 6
#define I2C_HZ  400000   // Use 100000 if you see I2C NACKs

MPU9250_asukiaaa imu;

uint8_t sensorId = 0;

// Calibration storage
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;   // deg/s
float accBiasX=0,  accBiasY=0,  accBiasZ=0;    // g 

//  median filter
template<size_t N>
struct Median {
  float buf[N]; uint8_t idx=0; bool primed=false;
  Median(){ for(size_t i=0;i<N;i++) buf[i]=0; }
  float push(float v){
    buf[idx++] = v; if(idx>=N){ idx=0; primed=true; }
    float tmp[N]; for(size_t i=0;i<N;i++) tmp[i]=buf[i];
    for(size_t i=1;i<N;i++){ float key=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>key){ tmp[j+1]=tmp[j]; j--; } tmp[j+1]=key; }
    return primed ? tmp[N/2] : v; // before primed returns latest
  }
};
Median<5> gxMed, gyMed, gzMed;
Median<5> axMed, ayMed, azMed;

// EMA (accel smoothing)
struct EMA {
  float y=0; bool init=false;
  float update(float x, float alpha){
    if(!init){ y=x; init=true; }
    else y = alpha*x + (1.0f-alpha)*y;
    return y;
  }
} axEMA, ayEMA, azEMA;

// simple 1D Kalman filter per gyro axis
struct Kalman1D {
  float x=0.0f;   // estimate
  float P=1.0f;   // covariance
  float Q=0.03f;  // process noise  (↑ = faster, ↓ = smoother)
  float R=0.60f;  // measure noise   (↑ = smoother)
  bool init=false;
  Kalman1D() {}
  Kalman1D(float q,float r): Q(q),R(r){}
  void reset(float v){ x=v; P=1.0f; init=true; }
  float update(float z){
    if(!init){ reset(z); return x; }
    P = P + Q;
    float K = P / (P + R);
    x = x + K * (z - x);
    P = (1.0f - K) * P;
    return x;
  }
};
Kalman1D kx(0.03f, 0.60f), ky(0.03f, 0.60f), kz(0.03f, 0.60f);

// tunables 
const uint16_t CAL_SAMPLES   = 1000;    // more samples = better bias
const uint16_t CAL_DELAY_MS  = 4;       // ~250 Hz during calibration
const uint8_t  LOOP_AVG_N    = 5;       // sub-samples per loop (fast average)
const uint16_t LOOP_SUB_MS   = 2;       // spacing between sub-samples
const float    ACC_EMA_ALPHA = 0.18f;   // 0.1–0.3
const float    OUTLIER_GYR_DPS = 20.0f; // reject sudden > this diff vs previous
const float    OUTLIER_ACC_G   = 0.30f;

// helpers to read once
bool readAccelOnce(float& x,float& y,float& z){
  if(imu.accelUpdate()!=0) return false;
  x = imu.accelX(); y = imu.accelY(); z = imu.accelZ(); // g
  return true;
}
bool readGyroOnce(float& x,float& y,float& z){
  if(imu.gyroUpdate()!=0) return false;
  x = imu.gyroX(); y = imu.gyroY(); z = imu.gyroZ();    // deg/s
  return true;
}

// sub-sample averaged reads with outlier clamp
bool readAccelAvg(float& x,float& y,float& z){
  float sx=0, sy=0, sz=0;
  static bool havePrev=false; static float px=0,py=0,pz=0;
  for(uint8_t i=0;i<LOOP_AVG_N;i++){
    float ax,ay,az;
    if(!readAccelOnce(ax,ay,az)) return false;
    if(havePrev){
      if(fabs(ax-px)>OUTLIER_ACC_G) ax=px;
      if(fabs(ay-py)>OUTLIER_ACC_G) ay=py;
      if(fabs(az-pz)>OUTLIER_ACC_G) az=pz;
    }
    sx+=ax; sy+=ay; sz+=az;
    px=ax; py=ay; pz=az; havePrev=true;
    delay(LOOP_SUB_MS);
  }
  x=sx/LOOP_AVG_N; y=sy/LOOP_AVG_N; z=sz/LOOP_AVG_N;
  return true;
}
bool readGyroAvg(float& x,float& y,float& z){
  float sx=0, sy=0, sz=0;
  static bool havePrev=false; static float px=0,py=0,pz=0;
  for(uint8_t i=0;i<LOOP_AVG_N;i++){
    float gx,gy,gz;
    if(!readGyroOnce(gx,gy,gz)) return false;
    if(havePrev){
      if(fabs(gx-px)>OUTLIER_GYR_DPS) gx=px;
      if(fabs(gy-py)>OUTLIER_GYR_DPS) gy=py;
      if(fabs(gz-pz)>OUTLIER_GYR_DPS) gz=pz;
    }
    sx+=gx; sy+=gy; sz+=gz;
    px=gx; py=gy; pz=gz; havePrev=true;
    delay(LOOP_SUB_MS);
  }
  x=sx/LOOP_AVG_N; y=sy/LOOP_AVG_N; z=sz/LOOP_AVG_N;
  return true;
}

// stationary calibration: gyro + accel (board flat & still)
void calibrateStationary(uint16_t samples=CAL_SAMPLES){
  Serial.println("\n[Cal] Keep board FLAT & STILL …");
  delay(1500);

  double gxSum=0, gySum=0, gzSum=0;
  double axSum=0, aySum=0, azSum=0;

  for(uint16_t i=0;i<samples;i++){
    float gx,gy,gz, ax,ay,az;
    if(readGyroOnce(gx,gy,gz)){ gxSum+=gx; gySum+=gy; gzSum+=gz; }
    if(readAccelOnce(ax,ay,az)){ axSum+=ax; aySum+=ay; azSum+=az; }
    delay(CAL_DELAY_MS);
  }

  gyroBiasX = gxSum / samples;
  gyroBiasY = gySum / samples;
  gyroBiasZ = gzSum / samples;

  float axMean = axSum / samples;
  float ayMean = aySum / samples;
  float azMean = azSum / samples;
  accBiasX = axMean;
  accBiasY = ayMean;
  accBiasZ = azMean - 1.0f;

  // reset filters to the zero-biased streams
  kx.reset(0); ky.reset(0); kz.reset(0);
  gxMed = Median<5>(); gyMed = Median<5>(); gzMed = Median<5>();
  axMed = Median<5>(); ayMed = Median<5>(); azMed = Median<5>();
  axEMA.init = ayEMA.init = azEMA.init = false;

  Serial.println("[Cal] Gyro bias (deg/s): " + String(gyroBiasX,4) + ", " + String(gyroBiasY,4) + ", " + String(gyroBiasZ,4));
  Serial.println("[Cal] Acc  bias (g)   : " + String(accBiasX,4) + ", " + String(accBiasY,4) + ", " + String(accBiasZ,4));

  // sanity: |acc| should be close to 1 g
  float gmag = sqrtf((axMean-accBiasX)*(axMean-accBiasX) + (ayMean-accBiasY)*(ayMean-accBiasY) + (azMean-accBiasZ-1.0f)*(azMean-accBiasZ-1.0f));
  if (fabs((azMean - accBiasZ) - 1.0f) > 0.15f) {
    Serial.println("[Cal] Acc magnitude off, retrying once…");
    delay(800);
    calibrateStationary(samples/2); // quick retry
  }
}

// auto re-zero gyro when stationary (handles temp drift)
void autoRezeroGyroIfStill() {
  const float GYR_STILL_DPS   = 0.45f;   // all |gyro| below → still
  const float ACC_MAG_TARGET  = 1.0f;    // |acc| ≈ 1g when still
  const float ACC_MAG_TOL     = 0.05f;   // ±0.05 g
  const uint32_t STILL_MS     = 3000;    // hold time before re-zero
  const int REZERO_SAMPLES    = 260;
  const int REZERO_DELAY_MS   = 3;

  static uint32_t tStart = 0;
  static bool tracking = false;

  float ax,ay,az, gx,gy,gz;
  if(!readAccelOnce(ax,ay,az)) return;
  if(!readGyroOnce(gx,gy,gz))  return;

  gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
  ax -= accBiasX;  ay -= accBiasY;  az -= accBiasZ;

  float gyrAbsMax = max(max(fabs(gx), fabs(gy)), fabs(gz));
  float accMag = sqrtf(ax*ax + ay*ay + az*az);
  bool still = (gyrAbsMax < GYR_STILL_DPS) && (fabs(accMag - ACC_MAG_TARGET) < ACC_MAG_TOL);

  uint32_t now = millis();
  if(still){
    if(!tracking){ tStart = now; tracking = true; }
    if(now - tStart > STILL_MS){
      double sx=0, sy=0, sz=0;
      for(int i=0;i<REZERO_SAMPLES;i++){
        float x,y,z;
        if(readGyroOnce(x,y,z)){ sx+=x; sy+=y; sz+=z; }
        delay(REZERO_DELAY_MS);
      }
      gyroBiasX = sx/REZERO_SAMPLES;
      gyroBiasY = sy/REZERO_SAMPLES;
      gyroBiasZ = sz/REZERO_SAMPLES;

      // reset filters after bias change
      kx.reset(0); ky.reset(0); kz.reset(0);
      gxMed = Median<5>(); gyMed = Median<5>(); gzMed = Median<5>();

      Serial.println("[AutoZero] Gyro bias (deg/s): " +
        String(gyroBiasX,4) + ", " + String(gyroBiasY,4) + ", " + String(gyroBiasZ,4));
      tracking = false;
    }
  } else {
    tracking = false;
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println("started");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);

  imu.setWire(&Wire);

  // Warm-up (helps gyro temp settle)
  delay(700);

  // WHO_AM_I (0x71=MPU9250, 0x73=MPU9255, 0x70=MPU6500)
  if (imu.readId(&sensorId) == 0) {
    Serial.print("WHO_AM_I: 0x"); Serial.println(sensorId, HEX);
  } else {
    Serial.println("Cannot read WHO_AM_I");
  }

  imu.beginAccel();
  imu.beginGyro();

  calibrateStationary(CAL_SAMPLES);

  Serial.println("[Setup] Done.\n");
}

void loop() {
  // Accel: sub-avg -> bias -> median5 -> EMA
  float ax,ay,az;
  if(readAccelAvg(ax,ay,az)){
    float ax_c = ax - accBiasX;
    float ay_c = ay - accBiasY;
    float az_c = az - accBiasZ;

    ax_c = axMed.push(ax_c);
    ay_c = ayMed.push(ay_c);
    az_c = azMed.push(az_c);

    ax_c = axEMA.update(ax_c, ACC_EMA_ALPHA);
    ay_c = ayEMA.update(ay_c, ACC_EMA_ALPHA);
    az_c = azEMA.update(az_c, ACC_EMA_ALPHA);

    Serial.print("accel g (cal): ");
    Serial.print(ax_c,3); Serial.print('\t');
    Serial.print(ay_c,3); Serial.print('\t');
    Serial.println(az_c,3);
  } else {
    Serial.println("accel read fail");
  }

  // Gyro: sub-avg -> bias -> median5 -> Kalman
  float gx,gy,gz;
  if(readGyroAvg(gx,gy,gz)){
    float gx_c = gx - gyroBiasX;
    float gy_c = gy - gyroBiasY;
    float gz_c = gz - gyroBiasZ;

    gx_c = gxMed.push(gx_c);
    gy_c = gyMed.push(gy_c);
    gz_c = gzMed.push(gz_c);

    gx_c = kx.update(gx_c);
    gy_c = ky.update(gy_c);
    gz_c = kz.update(gz_c);

    Serial.print("gyro dps (kal): ");
    Serial.print(gx_c,3); Serial.print('\t');
    Serial.print(gy_c,3); Serial.print('\t');
    Serial.println(gz_c,3);
  } else {
    Serial.println("gyro read fail");
  }

  // Drift control when still
  autoRezeroGyroIfStill();

  Serial.println();
  delay(160);  // ~15–18 Hz print, internal sampling much higher
}
