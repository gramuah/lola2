#include <math.h>

//Indicate the analog inputs 
const int X_Pin = A12;
const int Y_Pin = A11;

int X_Value = 0;
int Y_Value = 0;

void read_joystick()
{
  X_Value=analogRead(X_Pin)-510;
  Y_Value=analogRead(Y_Pin)-510;

  if ((X_Value < 20)&& (X_Value>-20))
  {
    X_Value=0;
  }
  if ((Y_Value < 20)&& (Y_Value>-20))
  {
    Y_Value=0;
  }

  if (X_Value>114)
    X_Value=114;
  
  if (X_Value<-114)
    X_Value=-114;
  
  if (Y_Value>114)
    Y_Value=114;  
  
  if (Y_Value<-114)
    Y_Value=-114;
/*
  Serial.print("X: ");
  Serial.print(X_Value);
  Serial.print(" Y: ");
  Serial.print(Y_Value);
  Serial.print(" ");
  */
}   // fin de void read_joystick()


void SRE_Joystick()
{
  static unsigned long time1=0;
  unsigned char estado_botones=0;
  float modulo,angle;
  while(1)
  {
    if(digitalRead(SW1_PIN)==HIGH)
    {
          if(digitalRead(SW5_PIN)==LOW)
          Serial2.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          else
          Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          break;
    }
    read_joystick();
    flag=0;
    modulo=sqrt(pow(X_Value,2)+pow(Y_Value,2));
    if (modulo>114)
      modulo=114;
    angle=atan2(Y_Value,X_Value);
    angle=180*angle/3.1415;
    
    
//    Serial.print(" Mod: ");
//    Serial.println(modulo);
/*    Serial.print(" Ang: ");
    Serial.println(angle);
    */
if (modulo >20)
{    
    if (angle>45 && angle<=135)
    {
      if (estado_botones!=ADE)
      {
        STATE=RESET_STATE;
        estado_botones=ADE;  
      }
      flag=1;
      velDER=velIZQ=127*modulo/114;
      veloc_right=10+271*(unsigned int)velDER/128;      
      veloc_left =10+271*(unsigned int)velIZQ/128;      

//      Serial.println(" Adelante");
    }
    else
      if (angle>-135 && angle<=-45)
      {
        if (estado_botones!=ATA)
        {
          STATE=RESET_STATE;
          estado_botones=ATA;  
        }
        flag=1;
        velDER=velIZQ=255-(60*modulo)/114;
        veloc_right=10+271*(unsigned int)velDER/128;      
        veloc_left =10+271*(unsigned int)velIZQ/128;      
//        Serial.println(" Atrás");
      }
      else
        if (angle>135 && angle <=181|| angle<=-135 && angle>=-181)
        {   
          if (estado_botones!=IZQ)
          {
            STATE=RESET_STATE;
            estado_botones=IZQ;  
          }
          flag=1;
          velDER=50;
          velIZQ=205;
          Serial.println(" Izquierda");
         }
         else
            if (angle>-45 && angle<-1|| angle<=45 ||  angle>=0)
            {   
              if (estado_botones!=DER)
              {
                STATE=RESET_STATE;
                estado_botones=DER;  
              }
              flag=1;
              velDER=205;
              velIZQ=50;
              Serial.println(" Derecha");
             }

} /// end if (modulo>20)
    if (flag==1)
    {     
      if (STATE==RESET_STATE)
      {
 //       Serial.print(" flag ");
        flag_move_motor=1;
        movimientos_vel();
        STATE=BOTON_STATE;
      }
      else
      {
        if((millis()-time1)>10)
        {
          SPEED_INI_R=SPEED_INI_R*1.02;
          time1=millis();
        }
        if (SPEED_INI_R>255)
          SPEED_INI_R=255;
        SPEED_INI_L=SPEED_INI_R;
        update_speeds(0);
      }
    }     
    else
    {
      // Just in case an error produce movement of the motors
      stop_motors(); 
      STATE=RESET_STATE;
      SPEED_INI_R=SPEED_INI_L=160,
      digitalWrite(LED,0);

      if((millis()-time1)>5000)
      {
        if(digitalRead(SW5_PIN)==LOW)
          Serial2.print(".");
        else
          Serial.print(".");
        time1=millis();
      }
    } // else if (flag==1)          
}  // fin while(1)
} // fin void SRE_Joystick()
