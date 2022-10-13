////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
int arranca_recto()
{
//  disp_lect_sensores();
  if(digitalRead(SW5_PIN)==LOW)
    Serial2.println("RECTO");
  else
    Serial.println("RECTO");

  SPEED_INI_R = SPEED_INI_L = 60;
  SPEED_INI_R_LIM = SPEED_INI_L_LIM = 200;
  dir_right = dir_left = 1;
  move_motors();
  estado_patrol=PATROL_RECTO;
  return(1);
}

////////////////////////////////////////////////////////////////////////
//                            patrol                                  //
// Patrol function to move in a corridor at least 1.5m width          //
////////////////////////////////////////////////////////////////////////
int patrol()
{
  int LL=0,LC=0,RC=0,RR=0;
  static unsigned int current_pulses=0,last_pulses=0;

  // Cambio de los nombres para esta función.
  LL=dist_us_sensor_central;
  LC=dist_us_sensor_left;
  RC=dist_us_sensor_right;
  RR=dist_us_sensor_back;

  if (LL<80 || LC<80 || RC<80 || RR<80)
  {
    SPEED_INI_L_LIM=SPEED_INI_R_LIM=150;  
  }
  else
  {
    if (estado_patrol==PATROL_RECTO)
    {
      SPEED_INI_L_LIM=SPEED_INI_R_LIM=200;        
    }
    else
    {
      SPEED_INI_L_LIM=SPEED_INI_R_LIM=120;        
    }  
  }
 
  if (LL>40 && LC>40 && RC>40 && RR>40)
  {
    if (estado_patrol!=PATROL_RECTO)
    {
      arranca_recto();
      return(1);      
    }
  } // end of if (LL>100 && LC>100 && RC>100 && RR>100)
  else
  {
      if (estado_patrol!=PATROL_SEPARA)
      {
//        disp_lect_sensores();

//        SERIA.println("GIRO SEPARAR");
        SPEED_INI_R_LIM = SPEED_INI_L_LIM = 90;
        if (RR>LL || RC>LC)
        {
          dir_right = 0;
          dir_left  = 1;
        }
        else
        {
          dir_right = 1;
          dir_left  = 0;
        }        
        move_motors();
        estado_patrol=PATROL_SEPARA;
      }
  }
  if(current_pulses = (unsigned int) (((encoderDER + encoderIZQ) / 2) + 0.4999) - last_pulses > PID_PULSES) 
  {
    last_pulses = current_pulses;
    straigh_dist1();
  }
  return(1);
}  // end of patrol()
