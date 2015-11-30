//Calibrar ESCs de 1000-2000 microsegundos.
//Codigo echo para ser usado con Arduino Nano.


#include <EEPROM.h>
#include "MPU6050.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;//MPU6050

#define GYRO_ESCALA 16.4
#define DEBUG 0                      //0:No     1:Señal RC    2:Angulo MPU    3:Gyroscopio    4:RC escalado ACRO   5:Salida PID    6:Angulo raw MPU6050   7:RC escalado ESTABLE
#define DEBUG2 1                     //0:No     1:Si
#define PRINT(x)   if (DEBUG2>0) Serial.print(x);
#define PRINTLN(x) if (DEBUG2>0) Serial.println(x);
#define MAX_RC_THROTTLE 1780
#define MIN_RC_THROTTLE 1140
#define CICLO 5000                   //Duracion ciclo principal en microsegundos.
#define BANDA 13
#define ACRO 0
#define ESTABLE 1
#define ESTABLE 1
#define INICIO '*'
#define FINAL '#'
#define DIVISOR '|'
#define CMD_TEXT 12

String inText;
int comando;
int contador; 

byte canal_1, canal_2, canal_3, canal_4;
int modo=1;            //0:Acro   1:Estable
int RC_roll, RC_pitch, RC_throttle, RC_yaw;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long tiempo_pasado1, tiempo_pasado2, tiempo_pasado3, tiempo_pasado4, tiempo_actual;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw, gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float xGyro_offset, yGyro_offset, zGyro_offset;
float angulo_pitch, angulo_roll, angulo_yaw, yawGyro, pitchGyro, rollGyro;
int16_t ax, ay, az, gx, gy, gz;
float ciclo=CICLO;
float tiempoCiclo = ciclo/1000000;
float pid_error_anterior_roll, pid_error_anterior_pitch, pid_error_anterior_yaw, pid_error_anterior_pitch_estable, pid_error_anterior_roll_estable;
float integral_roll, integral_pitch, integral_yaw, integral_roll_estable, integral_pitch_estable;
float pid_roll_setpoint, pid_pitch_setpoint, pid_yaw_setpoint, pid2_roll_setpoint, pid2_pitch_setpoint, pid2_yaw_setpoint;
float roll, pitch , yaw;

float P=3;
float I=0;
float D=0;
float max_pid = 400;

float P_yaw=0;
float I_yaw=0;
float D_yaw=0;
float max_pid_yaw = 400;

float P_estable=0;
float I_estable=0;
float D_estable=0;
float max_pid_estable=100;

float sensivilidad_RC_gyro=5;                              //Sensivilidad mandos RC en modo ACRO.
float sensivilidad_RC_estable=8;                           //Sensivilidad mandos RC en modo ESTABLE.

void setup() {
  #if DEBUG==0
        Serial.begin(9600);
    #elif DEBUG>0 
        Serial.begin(115200);
    #endif
    
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  DDRB |= B00011000;   //Pin digital 12 como salida para Led. y Vcc para HC-06.
  DDRD |= B11110000;   //Pines digitales 4,5,6 y 7 como salida control ESCs.

  digitalWrite(11,HIGH);   //Enciendo bluetooth.

  PRINTLN("Inicializando varibles de eeprom");
  inicializar_variables();

  PRINTLN("Inicializando dispositivos I2C...");
  mpu.initialize();
    
  PRINTLN("Verificando conexion MPU6050...");
  PRINTLN(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 ERROR!!!!!!!!!!!!!");

  PRINTLN("Configurando MPU6050...");
    
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);  //10,20,42,98,188  
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  
  PRINTLN("Obteniendo offsets MPU6050...");
    
  mpu.setXAccelOffset(-1193);
  mpu.setYAccelOffset(79);
  mpu.setZAccelOffset(1256); 
  

  for (int calc_off=0;calc_off<2000;calc_off++){
    mpu.getRotation(&gx, &gy, &gz);
    xGyro_offset += gx;
    yGyro_offset += gy;
    zGyro_offset += gz;
    if (calc_off % 30 == 0) PRINT(".");
    if (calc_off % 10 == 0) digitalWrite(12, !digitalRead(12));
    PORTD |= B11110000;                                        
    delayMicroseconds(1000);                                  
    PORTD &= B00001111;                                        
    delay(3); 
  }
  xGyro_offset = xGyro_offset / 2000;
  yGyro_offset = yGyro_offset / 2000;
  zGyro_offset = zGyro_offset / 2000;
  
  PRINTLN(".");
  PRINT("X:");PRINT(xGyro_offset);
  PRINT("    Y:");PRINT(yGyro_offset);
  PRINT("    Z:");PRINTLN(zGyro_offset);  
  PRINTLN("Offsets MPU6050 OK!!");
    
  PCICR |= (1 << PCIE1);                                               //Activo interrupcion por hardware 1.
  PCMSK1 |= (1 << PCINT8);                                             //Pin A0. ROLL
  PCMSK1 |= (1 << PCINT9);                                             //Pin A1. PITCH
  PCMSK1 |= (1 << PCINT10);                                            //Pin A2. THROTTLE
  PCMSK1 |= (1 << PCINT11);                                            //Pin A4. YAW

  for (int i=0;i<3;i++){                                               //Señal listos para armar motores.
      for (int x=0;x<10;x++){
        PORTD |= B11110000;                                        
        delayMicroseconds(1500);                                  
        PORTD &= B00001111;
        delay(3);
      }
    for (int x=0;x<50;x++){
        PORTD |= B11110000;                                        
        delayMicroseconds(1000);                                  
        PORTD &= B00001111;
        delay(3);
      }
  }
  
   //Throttle al minimo y yaw al minimo para armar motores.
  while(RC_throttle < 990 || RC_throttle > 1050 || RC_yaw > 1200){     
    start ++;                                                  
    PORTD |= B11110000;                                        
    delayMicroseconds(1000);                                  
    PORTD &= B00001111;                                        
    delay(3);                                                  
    if(start == 125){                                          
      digitalWrite(12, !digitalRead(12));                      
      start = 0;                                               
    }
  }
  start = 0;
                        
}

void loop() {
  leerMPU();
  debug();
  #if DEBUG==0
      if (start==0) bluetooth();
  #endif
  
  //Para arrancar motores primero throttle al minimo y yaw al minimo.
  if(RC_throttle < 1050 && RC_yaw < 1050)start = 1;
  //Segundo yaw al centro y arrancamos.
  if(start == 1 && RC_throttle < 1050 && RC_yaw > 1440){
    start = 2;
    
    integral_roll = 0;
    pid_error_anterior_roll = 0;
    integral_pitch = 0;
    pid_error_anterior_pitch = 0;
    integral_yaw = 0;
    pid_error_anterior_yaw = 0;
    integral_pitch_estable = 0;
    integral_roll_estable = 0;
    
    digitalWrite(12, HIGH);   
  }

  //Para parar motores throttle al minimo y yaw al maximo.
  if(start == 2 && RC_throttle < 1050 && RC_yaw > 1900){
    start = 0;
    digitalWrite(12, LOW);   
  }

  //Modo ESTABLE.
  if (modo == ESTABLE){                                                                
    
  pid2_roll_setpoint = 0;

  if(RC_throttle > 1100){
  if(RC_roll > (1500+BANDA))pid2_roll_setpoint = (RC_roll - (1500+BANDA))/sensivilidad_RC_estable;
  else if(RC_roll < (1500-BANDA))pid2_roll_setpoint = (RC_roll - (1500-BANDA))/sensivilidad_RC_estable;
  }
  
  pid2_pitch_setpoint = 0;

  if(RC_throttle > 1100){
  if(RC_pitch > (1500+BANDA))pid2_pitch_setpoint = (RC_pitch - (1500+BANDA))/sensivilidad_RC_estable;
  else if(RC_pitch < (1500-BANDA))pid2_pitch_setpoint = (RC_pitch - (1500-BANDA))/sensivilidad_RC_estable;
   }

  pid_roll_setpoint  = estable_pid_roll(angulo_roll, pid2_roll_setpoint, P_estable, I_estable, D_estable);
  pid_pitch_setpoint = estable_pid_pitch(angulo_pitch, pid2_pitch_setpoint, P_estable, I_estable, D_estable);
  }
  
  //Si estoy en modo ACRO mapeo los valores obtenidos del RC
  if (modo == ACRO){                                                                
    
  pid_roll_setpoint = 0;

  if(RC_throttle > 1100){
  if(RC_roll > (1500+BANDA))pid_roll_setpoint = (RC_roll - (1500+BANDA))/sensivilidad_RC_gyro;
  else if(RC_roll < (1500-BANDA))pid_roll_setpoint = (RC_roll - (1500-BANDA))/sensivilidad_RC_gyro;
  }
  
  pid_pitch_setpoint = 0;

  if(RC_throttle > 1100){
  if(RC_pitch > (1500+BANDA))pid_pitch_setpoint = (RC_pitch - (1500+BANDA))/sensivilidad_RC_gyro;
  else if(RC_pitch < (1500-BANDA))pid_pitch_setpoint = (RC_pitch - (1500-BANDA))/sensivilidad_RC_gyro;
   }
  }
  pid_yaw_setpoint = 0;
 
  if(RC_throttle > 1100){ 
    if(RC_yaw > (1500+BANDA))pid_yaw_setpoint = (RC_yaw - (1500+BANDA))/sensivilidad_RC_gyro;
    else if(RC_yaw < (1500-BANDA))pid_yaw_setpoint = (RC_yaw - (1500-BANDA))/sensivilidad_RC_gyro;
  }
  
  
  //Calculo PIDs gyros.
  roll = calcular_pid_roll(rollGyro, pid_roll_setpoint, P, I, D);           
  pitch = calcular_pid_pitch(pitchGyro, pid_pitch_setpoint, P, I, D);
  yaw = calcular_pid_yaw(yawGyro, pid_yaw_setpoint, P_yaw, I_yaw, D_yaw);
  
  //Si los motores estan armados...
  if (start == 2){                                                          
    
    throttle = RC_throttle + 200;
    
    esc_1 = throttle + pitch - roll - yaw; 
    esc_2 = throttle + pitch + roll + yaw; 
    esc_3 = throttle - pitch + roll - yaw; 
    esc_4 = throttle - pitch - roll + yaw; 
       
    if (esc_1 < 1000) esc_1 = 1000;                                         //1200 es la velocidad minima cuando los motores estan armados.
    if (esc_2 < 1000) esc_2 = 1000;                                         
    if (esc_3 < 1000) esc_3 = 1000;                                         
    if (esc_4 < 1000) esc_4 = 1000;                                        
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Velocidad maxima 2000.
    if(esc_2 > 2000)esc_2 = 2000;                                           
    if(esc_3 > 2000)esc_3 = 2000;                                           
    if(esc_4 > 2000)esc_4 = 2000;                                             
  }
  
  else{
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                           
  }
  
  //Espero hasta que se cumpla el CICLO.  
  while((micros() - ciclo) < CICLO);                                                              
  ciclo = micros();        
  
  
  //Mando los pulsos a los ESCs.
  PORTD |= B11110000;                                                                                                                      
  timer_channel_1 = esc_1 + ciclo;                                     
  timer_channel_2 = esc_2 + ciclo;                                     
  timer_channel_3 = esc_3 + ciclo;                                     
  timer_channel_4 = esc_4 + ciclo;                                     
  
  while(PORTD >= 16){                                                       
    esc_loop_timer = micros();                                              
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                
  }
}

//////////////////////////////////////////////////////////////
//////////////INTERRUPCIONES SEÑAL RC/////////////////////////
//////////////////////////////////////////////////////////////


ISR(PCINT1_vect){
  tiempo_actual = micros();
  //ROLL=============================================
  if(PINC & B00000001){                                        
    if(canal_1 == 0){                                   
      canal_1 = 1;                                      
      tiempo_pasado1 = tiempo_actual;                                  
    }
  }
  else if(canal_1 == 1){                                
    canal_1 = 0;                                        
    RC_roll = tiempo_actual - tiempo_pasado1;         
  }
  //PITCH============================================
  if(PINC & B00000010){                                        
    if(canal_2 == 0){                                   
      canal_2 = 1;                                      
      tiempo_pasado2 = tiempo_actual;                                  
    }
  }
  else if(canal_2 == 1){                                
    canal_2 = 0;                                        
    RC_pitch = tiempo_actual - tiempo_pasado2;      
  }
  //THROTTLE=========================================
  if(PINC & B00000100){                                        
    if(canal_3 == 0){                                   
      canal_3 = 1;                                      
      tiempo_pasado3 = tiempo_actual;                                  
    }
  }
  else if(canal_3 == 1){                                
    canal_3 = 0;                                        
    RC_throttle = tiempo_actual - tiempo_pasado3;      
    RC_throttle = map(RC_throttle, MIN_RC_THROTTLE, MAX_RC_THROTTLE, 1000, 1600);
    RC_throttle = constrain(RC_throttle, 1000, 1600);
  }
  //YAW=============================================
  if(PINC & B00001000){                                        
    if(canal_4 == 0){                                   
      canal_4 = 1;                                      
      tiempo_pasado4 = tiempo_actual;                                  
    }
  }
  else if(canal_4 == 1){                                
    canal_4 = 0;                                        
    RC_yaw = tiempo_actual - tiempo_pasado4;   
  }
}   

///////////////////////////////////////////////////////////////
///////////////////////////DEBUG///////////////////////////////
///////////////////////////////////////////////////////////////

void debug() {

//SEÑAL RC==========================================
  #if DEBUG==1
    Serial.print("    ROLL:");
    Serial.print(RC_roll);
    Serial.print("    PITCH:");
    Serial.print(RC_pitch);
    Serial.print("    THROTTLE:");
    Serial.print(RC_throttle);
    Serial.print("    YAW:");
    Serial.println(RC_yaw);
  #endif

//ANGULO============================================
  #if DEBUG==2
    //Serial.print("YAW:");
    //Serial.print(angulo_yaw);
    Serial.print("\tPITCH:");
    Serial.print(angulo_pitch);
    Serial.print("\tROLL:");
    Serial.println(angulo_roll);
  #endif

//GYROSCOPIO============================================
  #if DEBUG==3
    Serial.print("Yaw:");
    Serial.print(yawGyro);
    Serial.print("     Pitch:");
    Serial.print(pitchGyro);
    Serial.print("     Roll:");
    Serial.println(rollGyro);
  #endif

  //RC ESCALADOS========================================
  #if DEBUG==4
    Serial.print("Yaw:");
    Serial.print(pid_yaw_setpoint);
    Serial.print("     Pitch:");
    Serial.print(pid_pitch_setpoint);
    Serial.print("     Roll:");
    Serial.println(pid_roll_setpoint);
  #endif

  //SALIDA PID==========================================
  #if DEBUG==5
    Serial.print("Yaw:");
    Serial.print(yaw);
    Serial.print("     Pitch:");
    Serial.print(pitch);
    Serial.print("     Roll:");
    Serial.println(roll);
  #endif

  //RC ESCALADOS ESTABLE================================
  #if DEBUG==7
    Serial.print("Yaw:");
    Serial.print(pid2_yaw_setpoint);
    Serial.print("     Pitch:");
    Serial.print(pid2_pitch_setpoint);
    Serial.print("     Roll:");
    Serial.println(pid2_roll_setpoint);
  #endif
  
}

///////////////////////////////////////////////////////////////
////////////////////////LEER MPU6050///////////////////////////
///////////////////////////////////////////////////////////////

void leerMPU()
{
   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ang_x_raw_sq = square(ax);
    float ang_y_raw_sq = square(ay);
    float ang_z_raw_sq = square(az);
    float accYangle_raw = (atan2(ay,sqrt(ang_x_raw_sq+ang_z_raw_sq))+PI)*RAD_TO_DEG -180;
    float accXangle_raw = (atan2(ax,sqrt(ang_z_raw_sq+ang_y_raw_sq))+PI)*RAD_TO_DEG -180;  

    rollGyro = (gx-xGyro_offset)/GYRO_ESCALA;
    pitchGyro = (gy-yGyro_offset)/GYRO_ESCALA;
    yawGyro = (gz-zGyro_offset)/GYRO_ESCALA;   
    
    angulo_pitch = (0.1 * -accXangle_raw) + (0.9*(angulo_pitch + (pitchGyro*tiempoCiclo))); 
    angulo_roll =  (0.1 * accYangle_raw)  + (0.9*(angulo_roll  + (rollGyro*tiempoCiclo))); 

    //ANGULO RAW=========================================
    #if DEBUG==6
        //Serial.print("YAW:");
        //Serial.print(angulo_yaw);
        Serial.print("\tPITCH:");
        Serial.print(accXangle_raw);
        Serial.print("\tROLL:");
        Serial.println(accYangle_raw);
    #endif
    
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////PID//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//ROLL GYRO=============================================================================
float calcular_pid_roll(float gyro, float RC, float KP, float KI, float KD){
  float pid_error_roll = gyro - RC;
  integral_roll += KI * pid_error_roll;
  integral_roll = constrain(integral_roll, -max_pid, max_pid);
  float out = KP * pid_error_roll + integral_roll + KD * (pid_error_roll - pid_error_anterior_roll);
  out = constrain (out, -max_pid, max_pid);
  pid_error_anterior_roll = pid_error_roll;
  return out;
}

//PITCH GYRO=============================================================================
float calcular_pid_pitch(float gyro, float RC, float KP, float KI, float KD){
  float pid_error_pitch = gyro - RC;
  integral_pitch += KI * pid_error_pitch;
  integral_pitch = constrain(integral_pitch, -max_pid, max_pid);
  float out = KP * pid_error_pitch + integral_pitch + KD * (pid_error_pitch - pid_error_anterior_pitch);
  out = constrain (out, -max_pid, max_pid);
  pid_error_anterior_pitch = pid_error_pitch;
  return out;
}

//YAW GYRO===============================================================================
float calcular_pid_yaw(float gyro, float RC, float KP, float KI, float KD){
  float pid_error_yaw = gyro - RC;
  integral_yaw += KI * pid_error_yaw;
  integral_yaw = constrain(integral_yaw, -max_pid_yaw, max_pid_yaw);
  float out = KP * pid_error_yaw + integral_yaw + KD * (pid_error_yaw - pid_error_anterior_yaw);
  out = constrain (out, -max_pid_yaw, max_pid_yaw);
  pid_error_anterior_yaw = pid_error_yaw;
  return out;
}

//PITCH ESTABLE==========================================================================
float estable_pid_pitch(float angulo, float setpoint, float kp, float ki, float kd){
  float pid_error = setpoint - angulo;
  integral_pitch_estable += ki * pid_error;
  integral_pitch_estable = constrain(integral_pitch_estable, -max_pid_estable, max_pid_estable);
  float out = kp * pid_error + integral_pitch_estable + kd * (pid_error - pid_error_anterior_pitch_estable);
  out = constrain (out, -max_pid_estable, max_pid_estable);
  pid_error_anterior_pitch_estable = pid_error;
  return (out);
}

//ROLL ESTABLE===========================================================================
float estable_pid_roll(float angulo, float setpoint, float kp, float ki, float kd){
  float pid_error2 = setpoint - angulo;
  integral_roll_estable += ki * pid_error2;
  integral_roll_estable = constrain(integral_roll_estable, -max_pid_estable, max_pid_estable);
  float out = kp * pid_error2 + integral_roll_estable + kd * (pid_error2 - pid_error_anterior_roll_estable);
  out = constrain (out, -max_pid_estable, max_pid_estable);
  pid_error_anterior_roll_estable = pid_error2;
  return (out);
}
  

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////BLUETOOTH////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void bluetooth()
{  
  
  Serial.flush();
  int ard_command = 0;
  int pin_num = 0;
  int pin_value = 0;

  char get_char = ' ';  //read serial

  // wait for incoming data
  if (Serial.available() < 1) return; // if serial empty, return to loop().
  
  // parse incoming command start flag 
  get_char = Serial.read();
  if (get_char != INICIO)  
  {
    while(Serial.available()>0) Serial.read();
    return;
  }
  
  // parse incoming command type
  ard_command = Serial.parseInt(); // read the command
  
  // parse incoming pin# and value  
  pin_num = Serial.parseInt(); // read the pin
  
  pin_value = Serial.parseInt();  // read the value
  
  if (ard_command == CMD_TEXT)     // Si el comando es de texto:
          {  
              String s = GetLine();
          if (s == "h") {
             Serial.print("acro,estable,pg,ig,dg,pa,ia,da,sgyro,pe,ie,de");
             comando=0;
             return;
          }
                  
          if (s=="s") {
          comando=0;
          guardar_datos ();
          return;
        }
                        
        if (s=="pg") {
          comando=1;
          Serial.print("     P_pitch&roll:");Serial.print(P);
          return;
        } 
        
        if (s=="ig") {
          comando=2;
          Serial.print("     I_pitch&roll:");Serial.print(I);
          return;
        } 
        
        if (s=="dg") {
          comando=3;
          Serial.print("     D_pitch&roll:");Serial.print(D);
          return;
        } 
                
        if (s=="py") {
          comando=4;
          Serial.print("     P_yaw:");Serial.print(P_yaw);
          return;
        } 
        
        if (s=="iy") {
          comando=5;
          Serial.print("     I_yaw:");Serial.print(I_yaw);
          return;
        } 

        if (s=="dy") {
          comando=6;
          Serial.print("     D_yaw:");Serial.print(D_yaw);
          return;
        } 

        if (s=="sgyro") {
          comando=7;
          Serial.print("   Sen_Gyro:");Serial.print(sensivilidad_RC_gyro);
          return;
        } 

        if (s=="pe") {
          comando=8;
          Serial.print("   P_estable:");Serial.print(P_estable);
          return;
        } 

        if (s=="ie") {
          comando=9;
          Serial.print("   I_estable:");Serial.print(I_estable);
          return;
        } 

        if (s=="de") {
          comando=10;
          Serial.print("   D_estable:");Serial.print(D_estable);
          return;
        } 

        if (s=="acro") {
          modo = 0;
          Serial.print("   ACRO mode ON");
          return;
        } 

        if (s=="estable") {
          modo = 1;
          Serial.print("   ESTABLE mode ON");
          return;
        }
        
        if (comando==1) {
          P = string_float(s);
          Serial.print("     P_pitch&roll:");Serial.print(P);
          return;
        }
        
        if (comando==2) {
          I = string_float(s);
          Serial.print("     I_pitch&roll:");Serial.print(I);
          return;
        }
        
        if (comando==3) {
          D = string_float(s);
          Serial.print("     D_pitch&roll:");Serial.print(D);
          return;
        }
        
        if (comando==4) {
          P_yaw = string_float(s);
          Serial.print("     P_yaw:");Serial.print(P_yaw);
          return;
        }
        
        if (comando==5) {
          I_yaw = string_float(s);
          Serial.print("     I_yaw:");Serial.print(I_yaw);
          return;
        }
        if (comando==6) {
          D_yaw = string_float(s);
          Serial.print("     D_yaw:");Serial.print(D_yaw);
          return;
        }
        if (comando==7) {
          sensivilidad_RC_gyro = string_float(s);
          Serial.print("   Sen_Gyro:");Serial.print(sensivilidad_RC_gyro);
          return;
        }
        if (comando==8) {
          P_estable = string_float(s);
          Serial.print("     P_estable:");Serial.print(P_estable);
          return;
        }
        if (comando==9) {
          I_estable = string_float(s);
          Serial.print("   I_estable:");Serial.print(I_estable);
          return;
        }
         if (comando==10) {
          D_estable = string_float(s);
          Serial.print("   D_estable:");Serial.print(D_estable);
          return;
        }
          }
}


String GetLine()
   {   String S = "" ;
       if (Serial.available())
          {    char c = Serial.read(); 
                delay(5) ;
                c = Serial.read();
                while ( c != FINAL)            //Hasta que el caracter sea END_CMD_CHAR
                  {     S = S + c ;
                        delay(5) ;
                        c = Serial.read();
                  }
                return( S ) ;
          }
   }
  


 
 float string_float(String cadena) //Metodo que regresa un numero recibiendo una cadena
{
  char nume[50];  //vector temporal para guardar numero
  cadena.toCharArray(nume,50);  //se convierte la cadena en char y se guarda en nume
  float numero=atof(nume);  //convertimos el vector char en numero
  return numero;  //returnamos el numero
}


///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////EEPROM/////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void escrivir_eeprom(int direccion, float num)
      {
      long valor=num*10000;
      
      byte cuatro = (valor & 0xFF);
      byte tres = ((valor >> 8) & 0xFF);
      byte dos = ((valor >> 16) & 0xFF);
      byte uno = ((valor >> 24) & 0xFF);

      EEPROM.write(direccion, cuatro);
      EEPROM.write(direccion + 1, tres);
      EEPROM.write(direccion + 2, dos);
      EEPROM.write(direccion + 3, uno);
      }

float leer_eeprom(long direccion)
      {
      
      long cuatro = EEPROM.read(direccion);
      long tres = EEPROM.read(direccion + 1);
      long dos = EEPROM.read(direccion + 2);
      long uno = EEPROM.read(direccion + 3);

      float num=((cuatro << 0) & 0xFF) + ((tres << 8) & 0xFFFF) + ((dos << 16) & 0xFFFFFF) + ((uno << 24) & 0xFFFFFFFF);
      return (num/10000);
      }

void guardar_datos () 
      {
               
        escrivir_eeprom (0,P);
        escrivir_eeprom (4,I);
        escrivir_eeprom (8,D);
        escrivir_eeprom (12,P_yaw);
        escrivir_eeprom (16,I_yaw);
        escrivir_eeprom (20,D_yaw);
        escrivir_eeprom (24,P_estable);
        escrivir_eeprom (28,I_estable);
        escrivir_eeprom (32,D_estable);

        Serial.println("Datos guardados.");
      }
      
void inicializar_variables()
      {
        
        
        P = leer_eeprom(0);
        I = leer_eeprom(4);
        D = leer_eeprom(8);
        P_yaw = leer_eeprom(12);
        I_yaw = leer_eeprom(16);
        D_yaw = leer_eeprom(20);
        P_estable = leer_eeprom(24);
        I_estable = leer_eeprom(28);
        D_estable = leer_eeprom(32);
      }

 

