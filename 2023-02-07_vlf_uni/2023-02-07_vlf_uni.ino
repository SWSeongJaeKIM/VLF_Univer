#include <NewPing.h>
#include <Servo.h>
#include <MsTimer2.h>
//////////////////////초음파
#define MaxDistance 300
/////////////////////초음파

////////////////////모터
#define encodPinA1 2
#define encodPinB1 3
#define MOTOR_DIR 4
#define MOTOR_PWM 5
///////////////////모터
////////////////////카메라

#define A0pin A0
#define SIpin 11
#define CLKpin 12
#define NPIXELS 128
byte Pixel[NPIXELS];
int LineSenSor_Data[NPIXELS];
int LineSenSor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];

////////////////////카메라
#define servoPin 8  //서보모터
#define LEFT_STEER_ANGLE  -35  // 실험으로 구할것-45
#define NEURAL_ANGLE 75
#define RIGHT_STEER_ANGLE  35  // 실험으로 구할35
int flag =0;

volatile long encoderPos = 0;
int encoder_error=0;
int count=70;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

NewPing sonar[3] =
{
  NewPing (32, 30, 500),    //왼쪽 30 32
  NewPing (36, 34, 500),  //오른족 34 36
  NewPing (38, 40, 500),  //정면 38 40
};

double F_distance =0.0;
double L_distance =0.0;
double R_distance =0.0;


void sona_dis(void)
{
    L_distance = sonar[0].ping_cm()*10.0;
    R_distance = sonar[1].ping_cm()*10.0;
    F_distance = sonar[2].ping_cm()*10.0;
}

Servo Steeringservo;
int Steering_Angle = NEURAL_ANGLE;


int steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  Steeringservo.write(Steering_Angle);
}

void setup() {
 for (int i=0; i<NPIXELS; i++)
 {
   LineSenSor_Data[i] = 0;
   LineSenSor_Data_Adaption[i] = 0;
   MAX_LineSensor_Data[i] = 1023;
   MIN_LineSensor_Data[i] = 0;
 }
 
  pinMode(SIpin,OUTPUT);
  pinMode(CLKpin,OUTPUT);
  digitalWrite(CLKpin,LOW);
  digitalWrite(SIpin,LOW);

  #if FASTADC
   sbi(ADCSRA, ADPS2);
   cbi(ADCSRA, ADPS1);
   cbi(ADCSRA, ADPS0);
  #endif

  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);
  Steeringservo.attach(servoPin);
  Steeringservo.write(NEURAL_ANGLE);
  
  MsTimer2::set(100, interrupt_setup);
  MsTimer2::start();
  
 Serial.begin(115200);

}
///////////////////////////////////////////////////

void encoder_Serial(){
  interrupt_setup();

//  Serial.print("encoderPos = ");
//  Serial.print(encoderPos);
  encoder_error=encoderPos;
  encoderPos=0;
//  delay(50);
  }

int A=0;

void moter_speed(){
  encoder_Serial();
  if(encoder_error<A){
    count++;
    }
  else if(encoder_error>A){
    count--;
    }
 // Serial.print("moter speed = ");
  //Serial.println(count);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////
//라인 센서
void line_adaptation(void)
{
  int i;
  for(i=0; i<NPIXELS; i++)
  {
    if (LineSenSor_Data[i] >= MAX_LineSensor_Data[i]) MAX_LineSensor_Data[i]=LineSenSor_Data[i];
    if (LineSenSor_Data[i] <= MIN_LineSensor_Data[i]) MIN_LineSensor_Data[i]=LineSenSor_Data[i];
    }
  }

///////////////////////////////////////////////////////////////////////////////////////////
void read_line_sensor(){
  int i;
  int sum = 0;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin,LOW);
  digitalWrite (SIpin,HIGH);
  digitalWrite (CLKpin,HIGH);
  digitalWrite (SIpin,LOW);

  delayMicroseconds (1);

  for(i=0; i<NPIXELS; i++){
    Pixel[i]=analogRead (A0pin);
    digitalWrite (CLKpin,LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin,HIGH);
    }
  for(i=0; i<NPIXELS; i++){
    LineSenSor_Data_Adaption[i]=map(Pixel[i],MIN_LineSensor_Data[i],MAX_LineSensor_Data[i],0,256);
    }
 
  }



int read_line_sensor2()
{
  int i;
  int sum = 0;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin,LOW);
  digitalWrite (SIpin,HIGH);
  digitalWrite (CLKpin,HIGH);
  digitalWrite (SIpin,LOW);

  delayMicroseconds (1);

  for(i=0; i<NPIXELS; i++){
    Pixel[i]=analogRead (A0pin);
    digitalWrite (CLKpin,LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin,HIGH);
    }
  for(i=0; i<NPIXELS; i++){
    LineSenSor_Data_Adaption[i]=map(Pixel[i],MIN_LineSensor_Data[i],MAX_LineSensor_Data[i],0,256);
    }
    for(i=0; i<NPIXELS; i++)
    {
    sum += LineSenSor_Data_Adaption[i];
    }

    return sum;
  }

////////////////////////////////////////////////////////////////////////////////////////////

void motorSpeed(int direction, int speed)
{
  digitalWrite(MOTOR_DIR,direction);
  analogWrite(MOTOR_PWM,speed);
}

////////////////////////////////////////////////////////////////////////////////////////////
//이진화
#define threshold_valus 60
void threshold(void){
  int i;
    for(i=0; i<NPIXELS; i++){
    if(LineSenSor_Data_Adaption[i]>=threshold_valus){
      LineSenSor_Data_Adaption[i]=1;
      }
     else{
      LineSenSor_Data_Adaption[i]=0;
      }
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////////

int steer_data = 0;

void steering_by_camera(void){
  int i;
  long sum=0;
  long x_sum=0;
  steer_data = 0;
  for(i=8; i<NPIXELS-8; i++){
    Serial.print(LineSenSor_Data_Adaption[i]);
    sum += LineSenSor_Data_Adaption[i];
    x_sum += (LineSenSor_Data_Adaption[i])*i;
    }
   steer_data = (x_sum/sum) - NPIXELS/2 ;
   Serial.println(steer_data);
  }
////////////////////////////////////////////////////////////////////////////////////////////

void mode_line(){
  if(steer_data >= -60 && steer_data <= 60){
      Steering_Angle = NEURAL_ANGLE + (steer_data)*1.5;
   }
   steering_control();
     }
//////////////////////////////////////////////////////////////////////////////////////////////
void Flag1(void){    //모드1은 미로들어가서 턴한번까지
  sona_dis();
  if(F_distance <= 1150){
    while(L_distance+R_distance>=1140){
    sona_dis();
    motorSpeed(1,160);
    Steering_Angle=100;
    steering_control();
    }
   if(F_distance > 1500){
      flag=2;
      }
   }
  }
void Flag2(void){  // 모드 2는 벽두개타기
  sona_dis();
  if(R_distance>L_distance && R_distance <= 2000)
{Steering_Angle=85;}
  else if(R_distance<L_distance){       Steering_Angle=60;      } 
  else
{
   Steering_Angle=75;
}
  steering_control();
  moter_speed();
  motorSpeed(1,150); 
  }
void Flag3(void){ // 모드3는 두번쨰 코너돌기 
  sona_dis();
  if(F_distance <= 850 && R_distance>= 1000)
  {
    while(L_distance+R_distance>800){
    sona_dis();
    motorSpeed(1,100);
    Steering_Angle=100;
    steering_control();
   // misson_flag=2;
   
    sona_dis();
    if(F_distance > 700){

      break;
      }
      break;
    }
    flag=5;
   }
  }

////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
 
                    
  line_adaptation();
  sona_dis();
  read_line_sensor();
  threshold();

  if(flag==0){     //문제점 너무 빨리돌고(미로 3의 초음파 값 조정) 가다가 박아버려(flag3이 조건이 조금 애매한거같은데..) 
    A=105;
    moter_speed();
    motorSpeed(1,A);
    steering_by_camera();
    mode_line();
    delay(70);
    if(R_distance+L_distance<=1140){
      flag=1;
      }
    }
   else if(flag==1){
    A=150;
    Flag1();
    Flag2();
    delay(10);
    } 
  else if(flag==2){  
    A=150;
    Flag2();
   Flag3();
  delay(10);
    }

   else if(flag==4){ //이거는 왜한거지? 그냥 필요없는거같은데 
   sona_dis();
   A=100;
   if (F_distance < 900) flag == 5;
   if(R_distance>L_distance && R_distance <= 2000){Steering_Angle=85;}
  else if(R_distance<L_distance){       Steering_Angle=60;      } 
  else{                                             Steering_Angle=75;      }
  steering_control();
  motorSpeed(1,80);
  delay(10);
  

   }

else if(flag==5)
{
   A=130;
    moter_speed();
    motorSpeed(1,A);
    steering_by_camera();
    mode_line();
    delay(70);
    if(F_distance<=350)flag=6;  
    }
    else if(flag==6){
      motorSpeed(0,0);
      }


}-
