//unsigned long l_pulses = 0;
//unsigned long r_pulses = 0;


/*void enc_L_A (){
   encoderIZQ++;
  //l_pulses++;
}


void enc_R_A (){
   encoderDER++;
  //r_pulses++;
}
*/

int us_range1(int TriggerPin, int EchoPin) {
   
   long duration, distanceCm;
   
   digitalWrite(TriggerPin, LOW);  //Keep the line LOW for 4ms, for a clean flank
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  //Generate a high transition
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);   //After 10us, lower the line
   
   duration = pulseIn(EchoPin, HIGH, 500000);  //medimos el tiempo entre pulsos, en microsegundos
   
   distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
   
   if (distanceCm == 0)
    return -2; 
      
   if (distanceCm < 3000)
    return distanceCm;
    
  else
    return -1;
}


void print_message(){
  
  Serial.print("Selecciona el test a realizar [1-7]:\n");
  Serial.println("\t 1: Test de Bateria");
  Serial.println("\t 2: Test de DIP Switches");
  Serial.println("\t 3: Test de Buzzer");
  Serial.println("\t 4: Test de Motores");
  Serial.println("\t 5: Test Led RGB");
  Serial.println("\t 6: Test de Ultrasonidos");
  Serial.println("\t 7: Test de Botonera");
  Serial.print(": ");
  
}//END print_message()


void Lola_Validate(){

while(1){

if(digitalRead(SW4_PIN)==HIGH)
{
          Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          break;
}
char key; //This will hold the recibed byte
  unsigned long timeout;
  
  //Get a byte from the serial port. Keep reading, untill we recive something meaningfull.
  while (1){
    if(digitalRead(SW4_PIN)==HIGH)
    {
          Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
          break;
    }
    key = Serial.read();
    if (key != -1 & key != '\n' & key != '\r')
      break;
  }
  //Clear the recive buffer, it migth contain a \n o \r
  delay(100);
  while (Serial.available() )
    Serial.read();

  //main test routine
  switch(key){
    case '1':
    
      float adc; //Variable para obtener los valores en el 1 paso
      float voltaje; //Variable para obtener el voltaje en el 2 paso
      Serial.println("Test de bateria ADC");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){

        adc = analogRead(BAT_PIN);
        voltaje = adc * 5 / 1024;
        Serial.print("RAW Battery measurement:\t");
        Serial.println(voltaje);
        delay(500);
      }
      break;

    case '2':
      Serial.println("Test de DIP switches");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){
        Serial.print("Switch status:\t ");
        Serial.print(digitalRead(SW1_PIN) );
        Serial.print(digitalRead(SW2_PIN) );
        Serial.print(digitalRead(SW3_PIN) );
        Serial.print(digitalRead(SW4_PIN) );
        Serial.print(digitalRead(SW5_PIN) );
        Serial.print(digitalRead(SW6_PIN) );
        Serial.print(digitalRead(SW7_PIN) );
        Serial.println(digitalRead(SW8_PIN) );
        delay(500);
      }
      break;
      
    case '3':
      Serial.println("Test de zumbador");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){

          tone(BUZZER_PIN,650,500);
      }
      break;
      
    case '4':
      Serial.println("TEST de motores\n");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){

      //fwd test
        Serial.println("FWD, 0% > 100% > 0%");
        digitalWrite(MOT_L_A_PIN, LOW);
        digitalWrite(MOT_L_B_PIN, HIGH);
        digitalWrite(MOT_R_A_PIN, LOW);
        digitalWrite(MOT_R_B_PIN, HIGH);
        //0 > 100
        for(int i=0; i<255; i+=10){
          analogWrite(MOT_L_PWM_PIN, i);
          analogWrite(MOT_R_PWM_PIN, i);
          Serial.print(i);
          Serial.print("%\t ENC(L,R): ");
          Serial.print(encoderIZQ);
          Serial.print(' ');
          Serial.println(encoderDER);
          delay(250);
        }
        delay(1000);
        //100 >0
        for(int i=250; i>-1; i-=10){
          analogWrite(MOT_L_PWM_PIN, i);
          analogWrite(MOT_R_PWM_PIN, i);
          Serial.print(i);
          Serial.println('%');
//          Serial.print("%\t ENC(L,R): ");
//          Serial.print(encoderIZQ);
//          Serial.print(' ');
//          Serial.println(encoderDER);
          delay(250);
        }
        digitalWrite(MOT_L_PWM_PIN, LOW);
        digitalWrite(MOT_R_PWM_PIN, LOW);
        delay(2000);

      //rwd test
        Serial.println("RWD, 0% > 100% > 0%");
        digitalWrite(MOT_L_A_PIN, HIGH);
        digitalWrite(MOT_L_B_PIN, LOW);
        digitalWrite(MOT_R_A_PIN, HIGH);
        digitalWrite(MOT_R_B_PIN, LOW);
        //0 > 100
        for(int i=0; i<255; i+=10){
          analogWrite(MOT_L_PWM_PIN, i);
          analogWrite(MOT_R_PWM_PIN, i);
          Serial.print('-');
          Serial.print(i);
          Serial.println('%');
//          Serial.print("%\t ENC(L,R): ");
//          Serial.print(encoderIZQ);
//          Serial.print(' ');
//          Serial.println(encoderDER);
          delay(250);
        }
        delay(1000);
        //100>0
        for(int i=250; i>-1; i-=10){
          analogWrite(MOT_L_PWM_PIN, i);
          analogWrite(MOT_R_PWM_PIN, i);
          Serial.print('-');
          Serial.print(i);
          Serial.println('%');
//          Serial.print("%\t ENC(L,R): ");
//          Serial.print(encoderIZQ);
//          Serial.print(' ');
//          Serial.println(encoderDER);
          delay(250);
        }
        digitalWrite(MOT_L_PWM_PIN, LOW);
        digitalWrite(MOT_R_PWM_PIN, LOW);
        delay(2000);

        //BRAKE test
        Serial.println("BRAKE TEST, 0 > 100 BRAKE!");
        for(int i=0; i<255; i+=10){
          analogWrite(MOT_L_PWM_PIN, i);
          analogWrite(MOT_R_PWM_PIN, i);
          Serial.print(i);
          Serial.println('%');
          delay(100);
        }
        delay(2000);
        Serial.println("BRAKE!");
        digitalWrite(MOT_L_A_PIN, LOW);
        digitalWrite(MOT_L_B_PIN, LOW);
        digitalWrite(MOT_R_A_PIN, LOW);
        digitalWrite(MOT_R_B_PIN, LOW);
        
        delay(1000);
        digitalWrite(MOT_L_PWM_PIN, LOW);
        digitalWrite(MOT_R_PWM_PIN, LOW);
        
      }//END TEST
      break;
    case '5': //RGB LED
      Serial.println("Test de LED RGB");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){
        analogWrite(L_RED_PIN, 10);
        analogWrite(R_RED_PIN, 10);
        analogWrite(L_GRE_PIN,  0);
        analogWrite(R_GRE_PIN,  0);
        analogWrite(L_BLU_PIN,  0);
        analogWrite(R_BLU_PIN,  0);
        delay(1000);
  
        analogWrite(L_RED_PIN,  0);
        analogWrite(R_RED_PIN,  0);
        analogWrite(L_GRE_PIN, 10);
        analogWrite(R_GRE_PIN, 10);
        analogWrite(L_BLU_PIN,  0);
        analogWrite(R_BLU_PIN,  0);
        delay(1000);
  
        analogWrite(L_RED_PIN,  0);
        analogWrite(R_RED_PIN,  0);
        analogWrite(L_GRE_PIN,  0);
        analogWrite(R_GRE_PIN,  0);
        analogWrite(L_BLU_PIN, 10);
        analogWrite(R_BLU_PIN, 10);
        delay(1000);

        
        analogWrite(L_RED_PIN, 10);
        analogWrite(R_RED_PIN, 10);
        analogWrite(L_GRE_PIN, 10);
        analogWrite(R_GRE_PIN, 10);
        analogWrite(L_BLU_PIN,  0);
        analogWrite(R_BLU_PIN,  0);
        delay(1000);

        analogWrite(L_RED_PIN, 10);
        analogWrite(R_RED_PIN, 10);
        analogWrite(L_GRE_PIN,  0);
        analogWrite(R_GRE_PIN,  0);
        analogWrite(L_BLU_PIN, 10);
        analogWrite(R_BLU_PIN, 10);
        delay(1000);
  
        analogWrite(L_RED_PIN, 0);
        analogWrite(R_RED_PIN, 0);
        analogWrite(L_GRE_PIN, 0);
        analogWrite(R_GRE_PIN, 0);
        analogWrite(L_BLU_PIN, 0);
        analogWrite(R_BLU_PIN, 0);
        delay(1000);

      }
      break;
      
    case '6': //Ultrasound
      Serial.println("Test de ultrasonidos");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){
        Serial.println("Distancia en cm");
        Serial.print("F: ");
        Serial.print(us_range1(F_US_TRIG, F_US_ECHO));
        Serial.print("\tL: ");
        Serial.print(us_range1(L_US_TRIG, L_US_ECHO));
        Serial.print("\tR: ");
        Serial.print(us_range1(R_US_TRIG, R_US_ECHO));
        Serial.print("\tB: ");
        Serial.print(us_range1(B_US_TRIG, B_US_ECHO));
        Serial.println("");
        //delay(500);
      }
      break;
      
      case '7': //BOTONERA, orden de colores verde, rojo, amarillo, azul.
      Serial.println("Test de la botonera");
      Serial.println("Puedes pulsar una tecla en cualquier momento. El test acabrá al terminar el cliclo");
      while (!Serial.available() ){
        Serial.print("Green button:\t ");
        Serial.println(digitalRead(PIN_FORWARD) );
        Serial.print("Red button:\t ");
        Serial.println(digitalRead(PIN_BACKWARD) );
        Serial.print("Yellow button:\t ");
        Serial.println(digitalRead(PIN_LEFT) );
        Serial.print("Blue button:\t ");
        Serial.println(digitalRead(PIN_RIGHT) );
        Serial.println("------------------");
        delay (1500);
      }
      
    default:
      Serial.println("ERROR: Comando no reconocido");
      break;
  }//END SWITCH
  
  print_message();
  delay(100);
  // If there is something in the buffer, clear it
  while (Serial.available() )
    Serial.read();

  }
}//END lola_validate()
