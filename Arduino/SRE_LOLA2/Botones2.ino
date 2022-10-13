void SRE_Botones2()
{
  static unsigned long time1=0;
  unsigned char flag=0;
  unsigned char estado_botones=0;

  #define DER 1
  #define IZQ 2
  #define ADE 3
  #define ATA 4
  
  
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
    flag=0;
    if (digitalRead(PIN_FORWARD)==0)
    {
      if (estado_botones!=ADE)
      {
        STATE=RESET_STATE;
        estado_botones=ADE;  
      }
      flag=1;
      velDER=velIZQ=127;
//      Serial.println(" Adelante");
    }
    else
      if (digitalRead(PIN_BACKWARD)==0)
      {
        if (estado_botones!=ATA)
        {
          STATE=RESET_STATE;
          estado_botones=ATA;  
        }
        flag=1;
        velDER=velIZQ=155;
//        Serial.println(" Atrás");
      }
      else
        if (digitalRead(PIN_LEFT)==0)
        {   
          if (estado_botones!=IZQ)
          {
            STATE=RESET_STATE;
            estado_botones=IZQ;  
          }
          flag=1;
          velDER=70;
          velIZQ=185;
 //         Serial.println(" Izquierda");
         }
         else
            if (digitalRead(PIN_RIGHT)==0)
            {   
              if (estado_botones!=DER)
              {
                STATE=RESET_STATE;
                estado_botones=DER;  
              }
              flag=1;
              velDER=185;
              velIZQ=70;
   //           Serial.println(" Derecha");
             }
    
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
//       Serial.print(" update_speeds ");
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
  }   // end of while(1)
 }    // end void SRE_Botones2()
