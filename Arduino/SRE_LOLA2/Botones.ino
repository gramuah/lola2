#define TIME_PID        200 // Time in miliseconds for each iteration of the PID
  

// PID (pd) constants
  float kp =  0.21*1.5;
  float kd = 10.0*1.5;

// Radios relation allows to describe a curcular movement
float Radios_relation=1.0;


unsigned char orden[1];

unsigned char flag_boton=0;
unsigned char flag=0;

//////////////////////////////////////////////////
//  STRAIGH_DIST
//////////////////////////////////////////////////
void straigh_dist()
{ 
  float s,sl,sr;
  long encoder_long;
  float aux_float;
  unsigned int temp_encDER;
  unsigned int temp_encIZQ;

  // To avoid overflow
  if (encoderDER>10000 && encoderIZQ>10000)
  {
    encoderDER-=1000;
    encoderIZQ-=1000;  
  }
  temp_encDER=encoderDER;
  temp_encIZQ=encoderIZQ;

  // encoder is the difference from both encoder
  // with the nornalization constant for wheel diameter error  
  aux_float=(float)temp_encIZQ-(float)temp_encDER;
  if (aux_float>0)
    aux_float+=0.499999;
  if (aux_float<0)
    aux_float-=0.499999;
  encoder=(int)aux_float;

  error = encoder_ant - encoder;
  encoder_ant = encoder;
  // Implement PID (just PD)
  // Right wheel speed is updated
  // If it is not breaking at the end of the movement
  aux_float=(float)encoder * kp - (float)error * kd;
  if (aux_float>0)
    aux_float+=0.5;
  if (aux_float<0)
    aux_float-=0.5;

  velr += (int)aux_float;
  
  speed_normalization();

  // Write in PWM the speeds for each wheel
  analogWrite(MOT_R_PWM_PIN, velr);
  analogWrite(MOT_L_PWM_PIN, vell);          
}  // fin de straigh_dist()

//////////////////////////////////////////////////
//  DEP:
// This function is used for depuration
// 
//////////////////////////////////////////////////
void dep()
{
  if(digitalRead(SW5_PIN)==LOW){
  Serial2.print(" VR: ");
  Serial2.print(velr);
  Serial2.print(" VL: ");
  Serial2.print(vell);
  Serial2.print(" encoderDER: ");
  Serial2.print(encoderDER);     
  Serial2.print(" encoderIZQ: ");
  Serial2.println(encoderIZQ);
  }
  else {
  Serial.print(" VR: ");
  Serial.print(velr);
  Serial.print(" VL: ");
  Serial.print(vell);
  Serial.print(" encoderDER: ");
  Serial.print(encoderDER);     
  Serial.print(" encoderIZQ: ");
  Serial.println(encoderIZQ);
  }     
}  // fin de dep()

//////////////////////////////////////////////////
//  INIT_MOV
//  unsigned char direc: Indicating move foward (1) or backward (0)
//  unsigned char sp_r:  Speed for the right motor
//  unsigned char sp_l:  Speed for the left motor
//////////////////////////////////////////////////
void init_mov(unsigned char direc,unsigned char sp_r,unsigned char sp_l)
{
  static unsigned long time=0;
  volatile unsigned char flag_reset=1;
  volatile static unsigned char dir_aux=255;
  volatile static unsigned char sp_r_aux=0;
  volatile static unsigned char sp_l_aux=0;

  dep();
  if (dir_aux==direc && sp_r_aux==sp_r && sp_l_aux==sp_l)
    flag_reset=0;
  else 
    flag_reset=1;

  dir_aux=direc;
  sp_r_aux=sp_r;
  sp_l_aux=sp_l;
  
      SPEED_INI_R=sp_r;
      SPEED_INI_L=sp_l;             
      dir_right=direc;
      dir_left=direc;
      if (flag_boton==0 || flag_reset)
      {
        stop_motors(); 
        move_motors();           
        flag_boton=1;
      }
      digitalWrite(LED,1);
      
      if(millis()-time>TIME_PID)
      {
          time=millis();
          if (SPEED_INI_R!=0 && SPEED_INI_L!=0)
            straigh_dist();
          dep();
      }
}  // fin de void iniciar_mov(unsigned char direccion)




void SRE_Botones(){

  static unsigned long time1=0;
  static unsigned long time2=0;
  int LL=0,LC=0,RC=0,RR=0;

  while(1)
  {

    US_sensor_read_sequence();
    
    // Cambio de los nombres para esta función.
    LL=dist_us_sensor_central;
    LC=dist_us_sensor_left;
    RC=dist_us_sensor_right;
    RR=dist_us_sensor_back;


    
    if(digitalRead(SW1_PIN)==HIGH)
    {
          if(digitalRead(SW5_PIN)==LOW)
          Serial2.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          else
          Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          break;
    } 
    if (digitalRead(PIN_FORWARD)==0)
    {        
        if (LL<10 || LC<10)
        {
          stop_motors(); 
          flag_boton=0;
          digitalWrite(BUZZER_PIN, HIGH);
          delay(300);
          digitalWrite(BUZZER_PIN, LOW);
          delay(200);
          digitalWrite(BUZZER_PIN, HIGH);
          delay(100);
          digitalWrite(BUZZER_PIN, LOW);
        }
        else if ( RC<10 || RR<10)
        {
          stop_motors(); 
          flag_boton=0;
          digitalWrite(BUZZER_PIN, HIGH);
          delay(100);
          digitalWrite(BUZZER_PIN, LOW);
          delay(200);
          digitalWrite(BUZZER_PIN, HIGH);
          delay(300);
          digitalWrite(BUZZER_PIN, LOW);
        }
        else if(LL<45 || LC<45 || RC<45 || RR<45)
        {
          digitalWrite(BUZZER_PIN, HIGH);
          delay(30);
          digitalWrite(BUZZER_PIN, LOW);
          if (RR<RC && RR<LC && RR<LL)
          {
            init_mov(1,180,0);
          }
          else if(RC<RR && RC<LC && RC<LL)
          {
            init_mov(1,180,0);
          }
          else if(LC<RR && LC<RC && LC<LL)
          {
            init_mov(1,0,180);
          }
          else
          {
            init_mov(1,0,180);
          }         
        }
        else if(LL<60 || LC<60 || RC<60 || RR<60)
        { 
          init_mov(1,200,200
          );
        }
        else
        {
          init_mov(1,255,255);         
        }
    }     
    else
    {
      if (digitalRead(PIN_BACKWARD)==0)
      {
        init_mov(0,255,255);
      }
      else
      {
        if (digitalRead(PIN_LEFT)==0 && digitalRead(PIN_RIGHT)==HIGH)
        {
            if (LL<10 || LC<10) 
            {
              stop_motors(); 
              flag_boton=0;
              digitalWrite(BUZZER_PIN, HIGH);
              delay(300);
              digitalWrite(BUZZER_PIN, LOW);
              delay(200);
              digitalWrite(BUZZER_PIN, HIGH);
              delay(100);
              digitalWrite(BUZZER_PIN, LOW);
              
            }  
            else if (LL<60 || LC<60)
            {
              init_mov(1,200,0);
            }
            else
            {
              init_mov(1,255,0);  
            }
        }
        else if (digitalRead(PIN_LEFT)==HIGH && digitalRead(PIN_RIGHT)==0)
        {
          if (RC<10 || RR<10)
          {
            stop_motors(); 
            flag_boton=0;
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(200);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(300);
            digitalWrite(BUZZER_PIN, LOW);
           }
           else if (RC<60 || RR<60)
           {
            init_mov(1,0,200);
           }
           else
           {
            init_mov(1,0,255);
           }
         }
         else if (digitalRead(PIN_LEFT)==0 && digitalRead(PIN_RIGHT)==0)
         {
          if(RC<8 || RR<8)
          {
            init_mov(1,200,0);
            if(millis()-time2>200)
            {
              init_mov(1,0,200);
              time2=millis();
            }  
          }
          else if(LC<8 || LL<8)
          {
           init_mov(1,0,200);
           if(millis()-time2>200)
           {
             init_mov(1,0,200);
             time2=millis();
           }     
          }
          else
          {    
//           init_mov(1,0,200);
            if(millis()-time2>200)
            {
              if(flag==0)
              {
                init_mov(1,200,0);
                time2=millis();
                flag=1;
              }
              else
              {
                init_mov(1,0,200);
                time2=millis();            
                flag=0;
              }
            }  
          }  
         }
         else
         {
            // Just in case an error produce movement of the motors
            stop_motors(); 
            flag_boton=0; 
            digitalWrite(LED,0);

            if((millis()-time1)>5000)
            {
              if(digitalRead(SW5_PIN)==LOW)
              Serial2.print(".");
              else
              Serial.print(".");
              time1=millis();
            }
         }
      }
    }     
  }// end of while(1)
 }
