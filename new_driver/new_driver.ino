#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <GyverPID.h>
#include "GyverFilters.h"
GyverPID pid;
#include <Servo.h> 

int en = 23;
int ina = 5;
int inb = 7;
int pwm_pin = 13;
int alphaX0 = 0;
int velocity = 0;
MPU6050 GY521;                       // Создаем объект, символизирующий модуль датчика
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп

// Рабочие переменные
int16_t f_ax;
Servo motor;

GKalman testFilter(400, 40, 0.5);
void setup() {
  GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
 
  // put your setup code here, to run once:
  pinMode(en, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  motor.attach(11);
 motor.writeMicroseconds(2300);
 delay(2000);
 motor.writeMicroseconds(800);
 delay(6000);
  Wire.begin();                         // Инициализация Wire
  Serial.begin(9600);                  // Инициализация последовательного порта
  Serial.println("Initializing I2C devices..."); // Печать текста
  GY521.initialize();                   // Инициализация MPU
  delay(100);                           // Пауза
  delay(3000);
  alphaX0 = 0;
  pid.Kp = 0.02;
  pid.Ki = 0.002;
  pid.Kd = 0.0002;
  pid.setDt(100);
  pid.setDirection(NORMAL);
  pid.setpoint = 0;
  pid.setLimits(0, 255);


}
void limiter(int value, int mini=-100, int maxi = 100){
  if (value < mini){
    return (mini);
  }
  else if (value > maxi){
    return (maxi);
  }
  else{
    return (value);
  }
}
void moving(int v){
  if (v>0){
    
    digitalWrite(ina, 1);
    digitalWrite(inb, 0 );
  }
  else{
    
    digitalWrite(ina, 0);
    digitalWrite(inb, 1);
  }
  digitalWrite(en, 1);
  analogWrite(pwm_pin, abs(v));
}
void loop() {
 GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
 
  // вывод значений в монитор
  Serial.print("a/g:\t");
  
  //Serial.print(ay); Serial.print("\t");
  //Serial.print(az); Serial.print("\t");
  //Serial.print(gx); Serial.println("\t");
  //Serial.print(gy); Serial.print("\t");
  //Serial.println(gz);
  
  f_ax = testFilter.filtered((int)ax);
  //Serial.print(f_ax); Serial.println("\t");
  //Serial.print(ax); Serial.println("\t");
  delay(50); 
  
  pid.input = f_ax;
  pid.getResult();
  velocity = pid.output;
  if (f_ax < 0){
    moving(velocity);
  }
  else{
    moving(-velocity);
  }
  motor.writeMicroseconds(1000);
  
  Serial.print(velocity); Serial.println("\t");
  
}
