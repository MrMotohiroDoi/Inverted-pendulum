#include <SPI.h>
#include <stdio.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <nRF24L01.h>
#include <RF24.h>
 
//arduino
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12
#define PIN_SPI_SCK 13
#define PIN_SPI_SS 10
//control
#define BNO055_SAMPLERATE_DELAY_MS (5)
#define spdKp     0.02//0.001
#define spdKi     0.0
#define stbKp     40//40
#define stbKd     10//10
//range
int32_t maxspd=20000;


float dt, preTime;
float estimated_spd,pre_estimated_spd,rotspd,throttle=0,target_angle,pretarget_angle,control_output,motor1,motor2,filtered_est_spd,Vz,z ,preZ;
float Ispd;
float U;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

RF24 radio(7, 8);                // CE,CSNピンの指定
const byte address[6] = "00001";  // データを受信するアドレス
int V[3] = {0,0}; 


void setup(){
  delay(1000);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  radio.begin();                      // 無線オブジェクトの初期化 
  radio.openReadingPipe(0,address);  // データ受信アドレスを指定
  radio.setPALevel(RF24_PA_MIN);      // 出力を最小に
  radio.startListening();   

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  MAX_SPEED_setting_dual(maxspd);

  
  KVAL_setting_dual(0x09,255);//KVAL_HOLD
  KVAL_setting_dual(0x0A,255);//KVAL_RUN
  KVAL_setting_dual(0x0B,255);//KVAL_ACC
  KVAL_setting_dual(0x0C,255);//KVAL_DEC
  L6470_run_dual(1,1,0,0);//おまじない
  delay(10000);
}


void loop(){
  dt = (micros() - preTime) / 1000000;

   if (radio.available()){
    radio.read(&V, sizeof(V));  // スイッチの状態を受信する
  }
    
  
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyroscope=bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  z=6.2+euler.z();
  Vz=(z-preZ)/dt;
  rotspd=(read_spd1()*(read_dir1()-0.5)*2+read_spd2()*(read_dir2()-0.5)*2)/2;
  
  
  estimated_spd=rotspd*0.01302+Vz*0.6283;
  filtered_est_spd=0.9*estimated_spd+0.1*pre_estimated_spd;
  
  
  target_angle=speedPIcontrol(filtered_est_spd,V[0]);
  control_output+=stabilityPDcontrol(z,target_angle);
  motor1=control_output-V[1]*50;
  motor2=control_output+V[1]*50;
  

  if (motor1>0&&motor2>0){
    L6470_run_dual(0,0,motor1,motor2);
    }
  else if (motor1<=0&&motor2<=0){
    L6470_run_dual(1,1,-motor1,-motor2);
    }
  else if (motor1>0&&motor2<=0){
    L6470_run_dual(0,1,motor1,-motor2);
    } 
  else if (motor1<=0&&motor2>0){
    L6470_run_dual(1,0,-motor1,motor2);
    }

  preTime=micros();
  preZ=z;
  pre_estimated_spd=estimated_spd;
  pretarget_angle=target_angle;
  Serial.print("dT: ");
  Serial.print(dt);
  Serial.print("  target_angle: ");
  Serial.print(target_angle);
  Serial.print("  z: ");
  Serial.print(z);
  Serial.print(" filtered_est_spd ");
  Serial.print(filtered_est_spd);
  Serial.println("");

  

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
