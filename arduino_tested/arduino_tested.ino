#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <avr/io.h>
#include <util/delay.h>
#define GPS_RX 8
#define GPS_TX 9
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;
#define F_CPU 16000000UL
#define F_SCL 100000UL
#define gyro_x 0x43
#define gyro_y 0x45
#define gyro_z 0x47
#define ax 0x3B
#define ay 0x3D
#define az 0x3F
#define TEMP_PIN PC0
#define tare 10
int num = 0;
int16_t sensor;
float temp;
float gyroscope;
int cnt = 0;
float tot = 0.0;
float prev = 0.0;
float st = 0.0;
float lat = 0.0;
float lng = 0.0;
int tm;
double cable_tot = 10.0;
double temp_tot = 10.0;
#define PI 3.14
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define LCD_DIR DDRD 
#define LCD_PORT PORTD 

#define RS PD2             // RS -> PD2 (Arduino Pin 2)
#define EN PD3             // EN -> PD3 (Arduino Pin 3)

#define F_SCL 100000UL     // I2C clock speed: 100kHz
#define MPU6050_I2C_ADDR 0x68

void LCDPulseEnable() {
  LCD_PORT &= ~(1 << EN);   // EN LOW
  _delay_us(1);
  LCD_PORT |= (1 << EN);    // EN HIGH
  _delay_us(1);
  LCD_PORT &= ~(1 << EN);   // EN LOW
  _delay_us(100);
}

void LCDSend(uint8_t data, uint8_t mode) {
  if (mode == 0)
    LCD_PORT &= ~(1 << RS); // Command mode
  else
    LCD_PORT |= (1 << RS);  // Data mode

  LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0);
  LCDPulseEnable();

  LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);
  LCDPulseEnable();

  _delay_ms(2);
}

void LCDInit() {
  LCD_DIR |= (1 << RS) | (1 << EN) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); // Set pins as output
  _delay_ms(20); // Power-on delay

  LCDSend(0x02, 0); // Initialize LCD in 4-bit mode
  LCDSend(0x28, 0); // 2-line, 5x7 matrix
  LCDSend(0x0C, 0); // Display on, cursor off
  LCDSend(0x06, 0); // Entry mode
  LCDSend(0x01, 0); // Clear display
  _delay_ms(2);
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
  }
  LCDString(str);
}

void LCDClear() {
  LCDSend(0x01, 0); // Clear display
  _delay_ms(2);
  LCDSend(0x80, 0); // Set cursor to 0,0
}

void I2CInit() {
    TWSR = 0x00; // No prescaler
    TWBR = ((F_CPU / F_SCL) - 16) / 2; // Set bit rate
    TWCR = (1 << TWEN); // Enable TWI
}
 
 
void I2CStart() {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT))); // Wait for start condition
}
 
void I2CStop() {
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    while (TWCR & (1 << TWSTO)); // Ensure STOP condition is sent
}
 
void I2CWrite(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT))); // Wait for transmission
}
 
 
uint8_t I2CRead(bool ack) {  
    TWCR = (1 << TWEN) | (1 << TWINT) | (ack ? (1 << TWEA) : 0);
    while (!(TWCR & (1 << TWINT)));  // Wait for completion
    return TWDR;
}
 
 
int16_t MPU6050ReadReg(uint8_t reg) {
    I2CStart();
    I2CWrite(0x68 << 1);
    I2CWrite(reg);
    I2CStart();
    I2CWrite((0x68 << 1) | 1);
    float x = (I2CRead(true)<<8 | I2CRead(false) );
    I2CStop();
    return x;
 
}
 
void MPU6050Init() {
    I2CStart();
    I2CWrite(0x68 << 1);
    I2CWrite(0x6B);
    I2CWrite(0x00);
    I2CStop();
}
 
float getAcc(uint8_t a) {
    return MPU6050ReadReg(a) / 16384.0;
}
 
 
float getGyro(uint8_t a) {
    float sum = 0.0;
    for(int i = 0 ; i < 50 ; ++i) sum += MPU6050ReadReg(a) / 131.0;
    sum /= 50.0;
    return sum;
}
 
float get_theta_acc(){
  float accY =  getAcc(ay);
  float accX =  getAcc(ax);
  float val = -atan2(accY, accX) * 180.0 / PI;
  return (val < 0 ? val+360 : val);
}
 
float kalman_filter(float gyroRate, float sensorAngle, float dt) {
    static float angle = 0.0;
    static float bias  = 0.0;
    static float P[2][2] = { {1.0, 0.0}, {0.0, 1.0} };
    const float Q_angle  = 0.001;
    const float Q_bias   = 0.003;
    const float R_measure = 0.03;
    float rate = gyroRate - bias;
    angle += dt * rate;
    P[0][0] = P[0][0] + dt * ( dt * P[1][1] - P[0][1] - P[1][0] + Q_angle );
    P[0][1] = P[0][1] - dt * P[1][1];
    P[1][0] = P[1][0] - dt * P[1][1];
    P[1][1] = P[1][1] + Q_bias * dt;
    float y = sensorAngle - angle;
    float S = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    angle += K0 * y;
    bias  += K1 * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;
    return angle;
}
float new_cycles(){
  float dt = millis();
  float gz = getGyro(gyro_z);
  delay(100);
  float theta_acc = get_theta_acc();
  dt = (millis() - dt)/1000.0;
  if(prev - theta_acc > 300.0) theta_acc += cnt*360.0;
  else theta_acc +=  max((cnt-1)*360.0 , 0);
  float filtered_theta = kalman_filter(gz , theta_acc , dt);
  tot = filtered_theta;
  prev = fmod(theta_acc , 360.0);
  cnt = ceil(theta_acc / 360.0);
  return filtered_theta / 360.0;
}
void ADCInit(){
  ADMUX |=(1<< REFS1) | (1<< REFS0);
  ADCSRA |=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
}
uint16_t ADCRead(uint8_t pin){
  ADMUX |= (0b11110000 & ADMUX) | (0b00001111 & pin);
  ADCSRA |=(1<<ADSC);
  while (ADCSRA & (1<< ADSC));
  return ADC;
}

double CableLength(float T) {
    float R_outer = 100;
    float R_inner = 27;
    float wire_diameter = 5;
    float drum_width = 60;
    float totalLength = 0.0;
    float N = int(drum_width / wire_diameter);
    int k = 1;

    while(T > 0.0){
        int layer = int((k - 1) / N);
        float radius = R_outer - layer * wire_diameter;
        if (radius <= R_inner) break;
        float turnLength = min(N , T)*2 * M_PI * radius / 1000.0;
        totalLength += turnLength;
        T -= min(N , T);
        k++;
    }
    return totalLength;
} 


void setup() {
  ADCInit();
  I2CInit();
  MPU6050Init();
  DDRB &= ~(1 << PB2);
  gpsSerial.begin(9600);
  Serial.begin(9600);
  while (gpsModule.lat == 0.0) {
    if (gpsSerial.available()) {
      gpsModule.gps.encode(gpsSerial.read());
      if (gpsModule.gps.location.isUpdated()) {
        gpsModule.lat = gpsModule.gps.location.lat();
        gpsModule.lng = gpsModule.gps.location.lng();
      }
    }
  }
  LCDInit();
  LCDClear();
  prev = get_theta_acc();
  st = new_cycles();
  tm = 0;
}

void loop() {
  sensor = ADCRead(TEMP_PIN);
  temp = (float)sensor *  0.225 * 100.0 / 1023.0;
  gyroscope = new_cycles()-st;
  float tot = max(min(cable_tot - CableLength(gyroscope) , 10.0) , 0.0);
  if(PINB & (1 << PB2)){
    temp_tot=tot; 
  }
  String gyroStr = "len:" + String(tot, 2);
  LCDClear();
  LCDString(gyroStr);
  LCDStringPos(1, 0, String(temp)+ " | curr:"+String(temp_tot - tot, 2));
  if(tm == 5){
    Serial.print(tot);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(gpsModule.lat, 2);
    Serial.print(",");
    Serial.println(gpsModule.lng, 2);
    tm = 0;
  }
  tm++;
  delay(50);
}