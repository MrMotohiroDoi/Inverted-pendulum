//address+val(1 byte)
void KVAL_setting_dual(int8_t add,uint8_t val){
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(add);//KVAL_HOLD:0x09.....
  SPI.transfer(add);//KVAL_HOLD:0x09.....
  digitalWrite(PIN_SPI_SS,HIGH);
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(val); 
  SPI.transfer(val);   
  digitalWrite(PIN_SPI_SS,HIGH);
}
void MAX_SPEED_setting_dual(uint32_t val){
  int8_t buf[2];
  buf[0]=val>>8;
  buf[1]=val&0xff;
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(0x07);//address
  SPI.transfer(0x07);
  digitalWrite(PIN_SPI_SS,HIGH);
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(buf[0]); 
  SPI.transfer(buf[0]);   
  digitalWrite(PIN_SPI_SS,HIGH);
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(buf[1]); 
  SPI.transfer(buf[1]);   
  digitalWrite(PIN_SPI_SS,HIGH);
}
inline void L6470_run_dual(int8_t dir0,int8_t dir1,int32_t spd0,int32_t spd1){
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(0x50|dir0);
  SPI.transfer(0x50|dir1);
  digitalWrite (PIN_SPI_SS,HIGH);

  int8_t buf[6];
  buf[0]=spd0>>16;
  buf[1]=(spd0>>8)&0xff;
  buf[2]=spd0&0xff;
  buf[3]=spd1>>16;
  buf[4]=(spd1>>8)&0xff;
  buf[5]=spd1&0xff;
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(buf[0]);
  SPI.transfer(buf[3]);
  digitalWrite (PIN_SPI_SS,HIGH);
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(buf[1]);
  SPI.transfer(buf[4]);
  digitalWrite (PIN_SPI_SS,HIGH);
  digitalWrite (PIN_SPI_SS,LOW);
  SPI.transfer(buf[2]);
  SPI.transfer(buf[5]);
  digitalWrite (PIN_SPI_SS,HIGH); 
}
inline int32_t read_spd1() {
  int32_t val1 = 0;
  int32_t val2=0;
  int8_t  add = 0x04 | 0x20;
  digitalWrite(10, LOW);
  SPI.transfer(add);
  SPI.transfer(add);
  digitalWrite(10, HIGH);

  digitalWrite(10, LOW);
  val1 = SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
  //digitalWrite(10, LOW);
  val2 = SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  val1 = val1 << 8;
  digitalWrite(10, LOW);
  val1 = val1 | SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
   val2 = val2 << 8;
 // digitalWrite(10, LOW);
  val2 = val2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  val1 = val1 << 8;
  digitalWrite(10, LOW);
  val1 = val1| SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
  val2 = val2 << 8;
  //digitalWrite(10, LOW);
  val2 = val2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);
  return val1;
}
inline int32_t read_spd2() {
  int32_t val1 = 0;
  int32_t val2=0;
  int8_t  add = 0x04 | 0x20;
  digitalWrite(10, LOW);
  SPI.transfer(add);
  SPI.transfer(add);
  digitalWrite(10, HIGH);

  digitalWrite(10, LOW);
  val1 = SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
  //digitalWrite(10, LOW);
  val2 = SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  val1 = val1 << 8;
  digitalWrite(10, LOW);
  val1 = val1 | SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
   val2 = val2 << 8;
 // digitalWrite(10, LOW);
  val2 = val2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  val1 = val1 << 8;
  digitalWrite(10, LOW);
  val1 = val1| SPI.transfer(0x00);
  //digitalWrite(10, HIGH);
  val2 = val2 << 8;
  //digitalWrite(10, LOW);
  val2 = val2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);
  return val2;
}

inline int8_t read_dir1() {
  int32_t stat1=0;
  int32_t stat2=0;
  int8_t  add = 0x19 | 0x20;
  digitalWrite(10, LOW);
  SPI.transfer(add);
  SPI.transfer(add);
  digitalWrite(10, HIGH);

  digitalWrite(10, LOW);
  stat1 = SPI.transfer(0x00);
  stat2 = SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  stat1 = stat1 << 8;
  stat2 = stat2 << 8;
  digitalWrite(10, LOW);
  stat1 = stat1 | SPI.transfer(0x00);
  stat2 = stat2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  
  stat1&=0b0000000000010000;
  int8_t dir1=stat1>>4;
  return dir1;
  
}

inline int8_t read_dir2() {
  int32_t stat1=0;
  int32_t stat2=0;
  int8_t  add = 0x19 | 0x20;
  digitalWrite(10, LOW);
  SPI.transfer(add);
  SPI.transfer(add);
  digitalWrite(10, HIGH);

  digitalWrite(10, LOW);
  stat1 = SPI.transfer(0x00);
  stat2 = SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  stat1 = stat1 << 8;
  stat2 = stat2 << 8;
  digitalWrite(10, LOW);
  stat1 = stat1 | SPI.transfer(0x00);
  stat2 = stat2 | SPI.transfer(0x00);
  digitalWrite(10, HIGH);

  
  stat2&=0b0000000000010000;
  int8_t dir2=stat2>>4;
  return dir2;
  
}

inline float speedPIcontrol(float processval,float setpoint){             //processval:speed, setpoint:throttle
  float Pspd,Uspd;
  Pspd=setpoint-processval;
  Ispd+=Pspd*dt;
  Uspd=spdKp*Pspd+spdKi*Ispd;
  return Uspd;
}

inline float stabilityPDcontrol(float processval,float setpoint){         //processval:angle, setpoint:target-angle
  float Pstb,Dstb,Ustb;
  Pstb=setpoint-processval;
  Dstb=((target_angle-pretarget_angle)-(z-preZ))/dt;
  Ustb=stbKp*Pstb+stbKd*Dstb;
  return Ustb;
}
