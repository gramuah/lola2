#include "lola_board.h"

#define VERSION "V1.03"

#define IGNORE_PULSE   1400 // time in micros to ignore encoder pulses if faster

//#include <mcp2515.h>
//MCP2515 mcp2515(53);


unsigned char velIZQ,velDER;
unsigned char flag_move_motor=0;


unsigned int SPEED_INI_L=255;  // 170
unsigned int SPEED_INI_R=255;  // 100
unsigned int SPEED_INI_L_LIM=255;  // 170
unsigned int SPEED_INI_R_LIM=255;  // 100

unsigned int veloc_right;
unsigned int veloc_left;


// Direction of movement
unsigned char dir_right,dir_left;

// Variables to keep each encoder pulse
volatile unsigned int encoderIZQ = 0, encoderDER = 0;

//These variables keep track of the pulses received with a delay shorter than IGNORE_PULSE1 microseconds
volatile unsigned int ignored_left = 0, ignored_right = 0;

// Variables to obtain robot position and orientation (X, Y Theta)
unsigned int aux_encoderIZQ = 0, aux_encoderDER = 0;
volatile signed int encoder = 0;
//unsigned long pulsesDER=0;
//unsigned long pulsesIZQ=0;

// Auxiliar variables to filter false impulses from encoders
volatile unsigned long auxr=0,auxl=0;
// Auxiliar variables to keep micros() at encoders
unsigned long tr,tl;

// Indicate the PWM that is applied to the motors
int velr = SPEED_INI_R;
int vell = SPEED_INI_L;
int error = 0;
int encoder_ant;

// Keeps the last measurement taken by the ultrasound sensors
int dist_us_sensor_central = 0;
int dist_us_sensor_left = 0;
int dist_us_sensor_right = 0;
int dist_us_sensor_back = 0;

unsigned char vigila_patrol=0;
unsigned char estado_patrol=0;

#define PATROL_REPOSO   0
#define PATROL_RECTO    1
#define PATROL_SEPARA   2

// FSM's STATES
unsigned char STATE = 0;
#define RESET_STATE          0
#define ROTATE_CCW_STATE    2
#define ROTATE_CW_STATE     3
#define MOVE_STRAIGHT_STATE 4
#define RIGHT_FASTER_STATE  5
#define LEFT_FASTER_STATE   6
#define CIRC_TEST_R_STATE   7
#define CIRC_TEST_L_STATE   8
#define PATROL_STATE        9
#define MOVE_DIF_SPEED     10
#define CAN_STATE          11
#define BOTON_STATE        12
#define JOY_STATE        13

void setup() {

  //struct can_frame canMsg1;
  //mcp2515.reset();
//  mcp2515.setBitrate(CAN_125KBPS);
  //mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
//  mcp2515.setBitrate(CAN_1000KBPS);
  //mcp2515.setNormalMode();
  
  // Add the interrupt lines for encoders
  attachInterrupt(digitalPinToInterrupt(MOT_R_ENC_B_PIN), cuentaDER, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(MOT_L_ENC_B_PIN), cuentaIZQ, CHANGE);

  //Battery pin for voltaje measurement
  pinMode(BAT_PIN,         INPUT);

  //Dip switch for configuration
  pinMode(SW1_PIN,  INPUT_PULLUP);
  pinMode(SW2_PIN,  INPUT_PULLUP);
  pinMode(SW3_PIN,  INPUT_PULLUP);
  pinMode(SW4_PIN,  INPUT_PULLUP);
  pinMode(SW5_PIN,  INPUT_PULLUP);
  pinMode(SW6_PIN,  INPUT_PULLUP);
  pinMode(SW7_PIN,  INPUT_PULLUP);
  pinMode(SW8_PIN,  INPUT_PULLUP);

  //Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // set all the motor control pins to outputs
  pinMode(MOT_R_PWM_PIN, OUTPUT);
  pinMode(MOT_L_PWM_PIN, OUTPUT);
  
  pinMode(MOT_R_A_PIN, OUTPUT);
  pinMode(MOT_R_B_PIN, OUTPUT);
  pinMode(MOT_L_A_PIN, OUTPUT);
  pinMode(MOT_L_B_PIN, OUTPUT);

  // set encoder pins to inputs
  pinMode(MOT_L_ENC_B_PIN, INPUT);
  pinMode(MOT_R_ENC_B_PIN, INPUT);

  //L RGB LED
  pinMode(L_RED_PIN,      OUTPUT);
  pinMode(L_GRE_PIN,      OUTPUT);
  pinMode(L_BLU_PIN,      OUTPUT);
  
  //R RGB LED
  pinMode(R_RED_PIN,      OUTPUT);
  pinMode(R_GRE_PIN,      OUTPUT);
  pinMode(R_BLU_PIN,      OUTPUT);

  //F Ultrasound sensor
  pinMode(F_US_TRIG,      OUTPUT);
  pinMode(F_US_ECHO,      INPUT);
  //L Ultrasound sensor
  pinMode(L_US_TRIG,      OUTPUT);
  pinMode(L_US_ECHO,      INPUT);
  //R Ultrasound sensor           
  pinMode(R_US_TRIG,      OUTPUT);
//  pinMode(R_US_ECHO,      INPUT); Pin 53 (coincide con el pin CAN necesario en Mega)
  //B Ultrasound sensor
  pinMode(B_US_TRIG,      OUTPUT);
  pinMode(B_US_ECHO,      INPUT);
  
  // set buttons pins
  pinMode(PIN_FORWARD, INPUT_PULLUP);
  pinMode(PIN_BACKWARD, INPUT_PULLUP);
  pinMode(PIN_LEFT, INPUT_PULLUP);
  pinMode(PIN_RIGHT, INPUT_PULLUP);
  
  pinMode(LED, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  digitalWrite(MOT_R_A_PIN, LOW);
  digitalWrite(MOT_R_B_PIN, LOW);    
  digitalWrite(MOT_L_A_PIN, LOW);
  digitalWrite(MOT_L_B_PIN, LOW);

  analogWrite(MOT_R_PWM_PIN, 0);
  analogWrite(MOT_L_PWM_PIN, 0);

  Serial2.begin(38400);      //init the serial port
//  Serial.begin(38400);      //init the serial port
  Serial.begin(115200);      //init the serial port
  
  if(digitalRead(SW5_PIN)==LOW)
  {
  Serial2.print("LOLA INI ");
  Serial2.println(VERSION);
  Serial2.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
  }
  else 
  { 
  Serial.print("LOLA INI ");
  //Serial.println(VERSION);
  //Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
  }
}  // End of setup()


//////////////////////////////////////////////////
//  Right encoder interrupt using only one pin
//////////////////////////////////////////////////
void cuentaDER()
{
  tr=micros();
  // if pulse is too fast from previoius is ignored
//    Serial.println(tr-auxr);
  if (tr-auxr>(unsigned long)IGNORE_PULSE)
  {
    
    auxr=tr;
    encoderDER++;    //Add one pulse
  }
  
}  // end of cuentaDER

//////////////////////////////////////////////////
//  Left encoder interrupt using only one pin
//////////////////////////////////////////////////
void cuentaIZQ()
{
  tl=micros();
  // if pulse is too fast from previoius is ignored
  if (tl-auxl>(unsigned long)IGNORE_PULSE)
  {
    auxl=tl;
    encoderIZQ++;  //Add one pulse
  }
}  // end of cuentaIZQ


///////////////////////////////////////////
//              MOVE MOTORS              //
//  dir_right (1: foward / 0: backwards) //
//  dir_left  (1: foward / 0: backwards) //
///////////////////////////////////////////
void move_motors() 
{
  // now turn off motors
  // Adaptation for L298n
  unsigned int inhib_r = 0xBB, inhib_l = 0xEE;

  encoderIZQ = ignored_left = aux_encoderIZQ ;
  encoderDER =  ignored_right =  aux_encoderDER; 
  
  encoder = encoder_ant = 0;

  //Deactivate both motor's H-bridge
  PORTA &= 0xAA;

  velr = SPEED_INI_R;
  vell = SPEED_INI_L;

  if (!velr)
    PORTB |= 0x08;            
  else {
    inhib_r |= 0x44;
    analogWrite(MOT_R_PWM_PIN, velr);
  }

  if (!vell)
    PORTB |= 0x10;             

  else {
    inhib_l |= 0x11;
    analogWrite(MOT_L_PWM_PIN, vell);
  }  

  if (dir_right && dir_left)
    PORTA |= 0x05 & inhib_r & inhib_l;

  else if (!dir_right && dir_left)
    PORTA |= 0x41 & inhib_r & inhib_l;

  else if (dir_right && !dir_left)
    PORTA |= 0x14 & inhib_r & inhib_l;
  
  else
    PORTA |= 0x50 & inhib_r & inhib_l;
    
//  Serial.println("move_motors");

}  // End of move_motors()


///////////////////
//  STOP_MOTORS  //
///////////////////
void stop_motors() 
{
  PORTB |= 0x3 << 3;                              
  PORTA &= 0xAA;
  //We will fix the same duty cycle for the PWM outputs to make the braking spped equal!
  analogWrite(MOT_R_PWM_PIN, 255);
  analogWrite(MOT_L_PWM_PIN, 255);
  //delay(300);                             ////////////////////////////////////////////////////////////////////////////////////////////////////////
}  // End of stop_motors()

/////////////////////////////////////////////////
//         ULTRA SOUND RANGE DETECTOR          //
//  Return ->  -1:   Distance above 3m         //
//              0:   No return pulse detected  //
//            (int): Distance in cm            //
//                                             //
// Inputs -> TriggerPin                        //
//           EchoPin                           //
/////////////////////////////////////////////////
int us_range(int TriggerPin, int EchoPin) {
   long duration, distanceCm;
   
   digitalWrite(TriggerPin, LOW);   // Keep the line LOW for 4us for a clean flank
   delayMicroseconds(4);
   digitalWrite(TriggerPin, HIGH);  // Generate a high transition
   delayMicroseconds(10);
   digitalWrite(TriggerPin, LOW);   // After 10us, lower the line

   // Measure the duration in microseconds
   // Must be taken into account that this implies a delay of 10ms if no return is detected.
   duration = pulseIn(EchoPin, HIGH, 10000);

   // Check this number
   distanceCm = duration / 58.2;  // Convert the distance to cm

  if (vigila_patrol==1)
  {
    // Si algún sensor tiene dist. inferior a seguridad, se para
    // y se vuelve a reposo en el autómata del arduino y en el de
    // patrulla. 
    if (distanceCm<15 && distanceCm>0)
    {
//      SERIA.println("Comand P5");
      stop_motors();
      estado_patrol=PATROL_REPOSO;
      STATE=RESET_STATE;

    }
  }
   if (distanceCm == 0)
    return 200;

   if (distanceCm < 199)
    return distanceCm;
  else
    return 200;
}  // End of us_range()


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void US_sensor_read_sequence()
{
  static unsigned char us_sensor = 0;
  // Sequence to read the ultra-sound sensors. 
  // In each iteration one sensor is read. 
  // We must take into account that we get a
  // delay by reading the ultra-sound sensor...
//  if (us_sensor == 0)
    dist_us_sensor_central = us_range(F_US_TRIG, F_US_ECHO);
//  else if (us_sensor == 1)
    dist_us_sensor_left = us_range(L_US_TRIG, L_US_ECHO);
//  else if (us_sensor == 2)
    dist_us_sensor_right = us_range(R_US_TRIG, R_US_ECHO);
//  else if (us_sensor == 3)
    dist_us_sensor_back = us_range(B_US_TRIG, B_US_ECHO);
//  if (++us_sensor == 4)
//    us_sensor = 0;
}  // end of US_sensor_sequence()


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void disp_lect_sensores()
{
  static char str[50];
      sprintf(str,"US: %03d %03d %03d %03d ", dist_us_sensor_central,
                                              dist_us_sensor_left,
                                              dist_us_sensor_right,
                                              dist_us_sensor_back);
      if(digitalRead(SW5_PIN)==LOW)
       Serial2.println(str);
      else
       Serial.println(str);

}  // end of disp_lect_sensores()



//  SPEED_NORMALIZATION
//
//  Speeds are normalized in order to work 
//  at maximum speed give as SPEED_INI_X
//
//////////////////////////////////////////////////
void speed_normalization()
{
  if (velr>SPEED_INI_R)

//    if (velr>vell)
    {
      vell -= (velr-SPEED_INI_R);
      velr = SPEED_INI_R;
      if (vell<0)
        vell=0;          
    }

  if (vell>SPEED_INI_L)
//    else
    {
      velr -= (vell-SPEED_INI_L);
      vell = SPEED_INI_L;
      if (velr<0)
        velr = 0;
    }
} // end of speed_normalization


void loop() {
  // put your main code here, to run repeatedly:
  STATE=RESET_STATE;
  if(digitalRead(SW1_PIN)==LOW)
   {
       if(digitalRead(PIN_FORWARD)==LOW && digitalRead(PIN_BACKWARD)==LOW && digitalRead(PIN_LEFT)==LOW && digitalRead(PIN_RIGHT)==LOW)
       {
          if(digitalRead(SW5_PIN)==LOW)
            Serial2.println("MODO SRE_Joystick");
          else
            Serial.println("MODO SRE_Joystick");
          SRE_Joystick();
       }
       else 
       { 
          if(digitalRead(SW5_PIN)==LOW)
            Serial2.println("MODO SRE_BOTONES");
          else
            Serial.println("MODO SRE_BOTONES");
          STATE=RESET_STATE;
          SRE_Botones2();
       }
   }
  else if(digitalRead(SW2_PIN)==LOW)
   {
       if(digitalRead(SW5_PIN)==LOW)
       Serial2.println("MODO BOTONES_ANT");
       else
       Serial.println("MODO BOTONES_ANT");
       SRE_Botones2();
   }
  else if(digitalRead(SW3_PIN)==LOW)
   {
       if(digitalRead(SW5_PIN)==LOW)
       Serial2.println("MODO ROBOT");
       else
       Serial.println("MODO ROBOT");
       Lola();
   }   
  else if(digitalRead(SW4_PIN)==LOW)
   {
       
       if(digitalRead(SW5_PIN)==LOW)
       Serial2.println("MODO LOLA_VALIDATOR");
       else
       Serial.println("MODO LOLA_VALIDATOR");
       //print_message();
       //Lola_Validate();
   }
}
