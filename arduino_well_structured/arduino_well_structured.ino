#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

// Basic Definitions
#define F_CPU 16000000UL
#define F_SCL 100000UL

// MPU6050 registers
#define MPU6050_ADDR  0x68
#define PWR_MGMT_1    0x6B
#define gyro_x        0x43
#define gyro_y        0x45
#define gyro_z        0x47
#define ax            0x3B
#define ay            0x3D
#define az            0x3F

// Temperature sensor analog pin
#define TEMP_PIN      PC0

// LCD pins (PORTD)
#define RS            PD2    // LCD RS -> PD2
#define EN            PD3    // LCD EN -> PD3

// GPS pins (SoftwareSerial)
#define GPS_RX        8
#define GPS_TX        9

// Tare switch pin
#define tare          10

// Global cable and temperature totals
double cable_tot = 10.0;
double temp_tot  = 10.0;

// Global SoftwareSerial instance for GPS
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

/*
 * CableLength()
 * Calculates the cable length based on the number of turns (T) using the drum's geometry.
 */
double CableLength(float T) {
  float R_outer = 100, R_inner = 27, wire_diameter = 5, drum_width = 60;
  double totalLength = 0.0;
  float N = int(drum_width / wire_diameter);
  int k = 1;
  while (T > 0.0) {
    int layer = int((k - 1) / N);
    float radius = R_outer - layer * wire_diameter;
    if (radius <= R_inner) break;
    float turns = min(N, T);
    totalLength += turns * 2.0 * PI * radius / 1000.0;
    T -= turns;
    k++;
  }
  return totalLength;
}

/*
 * TemperatureSensor Module
 * Initializes the ADC and converts the sensor's analog value to a temperature.
 */
struct TemperatureSensor {
  void init() {
    ADMUX |= (1 << REFS1) | (1 << REFS0);
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
  }
  
  float read(uint8_t pin) {
    ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    int sensor = ADC;
    return sensor * 0.225 * 100.0 / 1023.0;
  }
};

/*
 * LCD Module
 * Provides basic LCD functionality in 4-bit mode.
 * Functions: LCDInit, LCDSend, LCDClear, LCDString, LCDStringPos.
 */
struct LCD {
  void LCDInit() {
    DDRD |= (1 << RS) | (1 << EN) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    _delay_ms(20);
    LCDSend(0x02, 0);
    LCDSend(0x28, 0);
    LCDSend(0x0C, 0);
    LCDSend(0x06, 0);
    LCDClear();
  }
  
  void LCDPulseEnable() {
    PORTD &= ~(1 << EN);
    _delay_us(1);
    PORTD |= (1 << EN);
    _delay_us(1);
    PORTD &= ~(1 << EN);
    _delay_us(100);
  }
  
  void LCDSend(uint8_t data, uint8_t mode) {
    if (mode == 0)
      PORTD &= ~(1 << RS);
    else
      PORTD |= (1 << RS);
    PORTD = (PORTD & 0x0F) | (data & 0xF0);
    LCDPulseEnable();
    PORTD = (PORTD & 0x0F) | ((data << 4) & 0xF0);
    LCDPulseEnable();
    _delay_ms(2);
  }
  
  void LCDClear() {
    LCDSend(0x01, 0);
    _delay_ms(2);
    LCDSend(0x80, 0);
  }
  
  void LCDString(String str) {
    for (int i = 0; i < str.length(); i++) {
      LCDSend(str[i], 1);
    }
  }
  
  void LCDStringPos(int row, int pos, String str) {
    if (pos < 16) {
      if (row == 0)
        LCDSend(0x80 | pos, 0);
      else if (row == 1)
        LCDSend(0xC0 | pos, 0);
      LCDString(str);
    }
  }
};

/*
 * MPU6050 Module
 * Handles I2C communication with the MPU6050 sensor and fuses gyro and accelerometer data via a Kalman filter.
 * Functions: MPU6050Init, MPU6050ReadReg, getAcc, getGyro, get_theta_acc, kalman_filter, new_cycles.
 */
struct MPU6050 {
  float prev;
  int cnt;
  float st;
  
  MPU6050() : prev(0), cnt(0), st(0) { }
  
private:
  void i2cInit() {
    TWSR = 0x00;
    TWBR = ((F_CPU / F_SCL) - 16) / 2;
    TWCR = (1 << TWEN);
  }
  
  void i2cStart() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
  }
  
  void i2cStop() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO));
  }
  
  void i2cWrite(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
  }
  
  uint8_t i2cRead(bool ack) {
    TWCR = (1 << TWEN) | (1 << TWINT) | (ack ? (1 << TWEA) : 0);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
  }
  
public:
  int16_t MPU6050ReadReg(uint8_t reg) {
    i2cStart();
    i2cWrite(MPU6050_ADDR << 1);
    i2cWrite(reg);
    i2cStart();
    i2cWrite((MPU6050_ADDR << 1) | 1);
    int16_t value = (i2cRead(true) << 8) | i2cRead(false);
    i2cStop();
    return value;
  }
  
  float getAcc(uint8_t reg) {
    return MPU6050ReadReg(reg) / 16384.0;
  }
  
  float getGyro(uint8_t reg) {
    float sum = 0.0;
    for (int i = 0; i < 50; i++) {
      sum += MPU6050ReadReg(reg) / 131.0;
    }
    return sum / 50.0;
  }
  
  float get_theta_acc() {
    float accY = getAcc(ay);
    float accX = getAcc(ax);
    float val = -atan2(accY, accX) * 180.0 / PI;
    return (val < 0 ? val + 360 : val);
  }
  
  float kalman_filter(float gyroRate, float sensorAngle, float dt) {
    static float angle = 0.0, bias = 0.0, P[2][2] = { {1.0, 0.0}, {0.0, 1.0} };
    const float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
    float rate = gyroRate - bias;
    angle += dt * rate;
    P[0][0] = P[0][0] + dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] = P[0][1] - dt * P[1][1];
    P[1][0] = P[1][0] - dt * P[1][1];
    P[1][1] = P[1][1] + Q_bias * dt;
    float y = sensorAngle - angle;
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S, K1 = P[1][0] / S;
    angle += K0 * y;
    bias  += K1 * y;
    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;
    return angle;
  }
  
  // Initializes the MPU6050 sensor
  void MPU6050Init() {
    i2cInit();
    i2cStart();
    i2cWrite(MPU6050_ADDR << 1);
    i2cWrite(PWR_MGMT_1);
    i2cWrite(0x00);
    i2cStop();
    prev = get_theta_acc();
    st = new_cycles();
  }
  
  // Reads sensor data, applies sensor fusion, and computes rotation cycles
  float new_cycles() {
    float startTime = millis();
    float gz = getGyro(gyro_z);
    delay(100);
    float theta_acc = get_theta_acc();
    float dt = (millis() - startTime) / 1000.0;
    if (prev - theta_acc > 300.0)
      theta_acc += cnt * 360.0;
    else
      theta_acc += max((cnt - 1) * 360.0, 0);
    float filtered_theta = kalman_filter(gz, theta_acc, dt);
    prev = fmod(theta_acc, 360.0);
    cnt = ceil(theta_acc / 360.0);
    return filtered_theta / 360.0;
  }
};

/*
 * GPS Module
 * Uses TinyGPS++ to update and store current latitude and longitude.
 */
struct GPSModule {
  TinyGPSPlus gps;
  float lat;
  float lng;
  SoftwareSerial* serial;
  
  GPSModule(SoftwareSerial &serialRef) : lat(0.0), lng(0.0), serial(&serialRef) { }
  
  void update() {
    while (serial->available()) {
      gps.encode(serial->read());
      if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
      }
    }
  }
};

// Global module objects using original nomenclature
TemperatureSensor temp;
LCD lcd;
MPU6050 mpu;
GPSModule gpsModule(gpsSerial);

void setup() {
  temp.init();
  lcd.LCDInit();
  DDRB &= ~(1 << PB2);
  mpu.MPU6050Init();
  gpsSerial.begin(9600);
  Serial.begin(9600);
  // Wait for valid GPS data
  while (gpsModule.lat == 0.0) {
    if (gpsSerial.available()) {
      gpsModule.gps.encode(gpsSerial.read());
      if (gpsModule.gps.location.isUpdated()) {
        gpsModule.lat = gpsModule.gps.location.lat();
        gpsModule.lng = gpsModule.gps.location.lng();
      }
    }
  }
}

void loop() {
  float temp_val = temp.read(TEMP_PIN);
  float cycles = mpu.new_cycles();
  float currentLength = max(min(cable_tot - CableLength(cycles), 10.0), 0.0);
  
  if (PINB & (1 << PB2))
    temp_tot = currentLength;
  
  String gyroStr = "len:" + String(currentLength, 2);
  lcd.LCDClear();
  lcd.LCDString(gyroStr);
  
  String tempStr = String(temp_val) + " | curr:" + String(temp_tot - currentLength, 2);
  lcd.LCDStringPos(1, 0, tempStr);
  
  static int tm = 0;
  if (tm == 5) {
    gpsModule.update();
    Serial.print(currentLength);
    Serial.print(",");
    Serial.print(temp_val);
    Serial.print(",");
    Serial.print(gpsModule.lat, 2);
    Serial.print(",");
    Serial.println(gpsModule.lng, 2);
    tm = 0;
  }
  tm++;
  delay(50);
}
