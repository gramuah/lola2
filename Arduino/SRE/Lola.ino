///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            DIFFERENTIAL ROBOT FIRMWARE                                            //
//                                      DEVELOPED AT THE UNIVERSITY OF ALCALÁ                                        //
// You can find more information at: www.hindawi.com/journals/js/2019/8269256/?utm_medium=author&utm_source=Hindawi  //
//                                              Please include reference                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Date 4/07/19

//Max number of digits for updated data
#define N_DIGS 5

// Defines to setup for LOLA robot or for SRE platform
#define ROBOT_SRE

struct can_frame trama_can_recibida;

float RADIO = 1;

#define PASOS_POR_VUELTA 712.6
#define Kdato 61.95

unsigned long atr = 0;


// OJO CAN
////////////////////////////////////////////////////////////////
int encoderIZQ_aux_can = 0;
int encoderDER_aux_can = 0;
long encoderAAbs = 0, tAabs;
long encoderBAbs = 0, tBabs;
////////////////////////////////////////////////////////////////
// OJO CAN

//////////////////////////////////
//      ERROR VARIABLE          //
// IDENTIFICATION ERRORS CODES  //
//////////////////////////////////
unsigned char ERROR_CODE = 0;
#define NO_ERROR              0
#define NO_NUMBER             1   // Waiting for a number, time-out
#define OUT_RANGE             2   // Received number out of range
#define SPEED_OUT_RANGE       3   // Received speed out of range
#define RR_OUT_RANGE          4   // Radio Ratio out of range
#define NO_AVAILABLE          5   // Received Command non available
#define INERTIA_LIMIT_ERROR   6   // Distance lower than inertia limit

// Global robot position
float X = 0, Y = 0, Theta = 0;

// Calibration constant
float KKI = 1; // Deviation from theoretical left wheel diameter; the rigth wheel is the reference; >1 wheel bigger than the nominal one

//Calibration correction factor for turning
int c_factor = 1;

//Current_pulses and last_pulses for PID calibration
float WHEEL_DIST = 536;   //533      //UAH 578; //UMB          //575;      // Wheel distance in mm
float mmperpulse = 0.8525;//1.705; //1.68        // mm per pulse in the encoders
#define INERTIA_LIMIT  5            // Inertia limit to stop few pulses before the limit
#define PID_TIME       500          // Time in miliseconds for each iteration of the PID
#define BREAK_PULSES    0//30            // Number of pulses from the end to start breaking
#define IGNORE_PULSE1   11000        // time in micros to ignore encoder pulses if faster
#define PID_PULSES 10

// PID (PD) constants
//float kp1 =  0.5;
//float kd1 = 20.0; //kd1 = 20.0
float kp1 =  0.01;
float kd1 = 2;

unsigned char TEST_distances[2000];
unsigned char TEST_pulses[2000];
unsigned int test_counter = 0;


// Radii relation allows to describe a circular movement
float radii_relation = 1.0;

unsigned int PULSES_NUM;

unsigned char order[1];

// Indicate if the rotation is clockwise or counterclockwise
unsigned char clockwise = 0;

int breaking_period = 0;

// Variable to check the theta for test
float theta_max = 0;



///////////////////
//  STRAIGH_DIST //
///////////////////
void straigh_dist1()
{
  long encoder_long;
  unsigned int temp_encDER, temp_encIZQ;
  float s, sl, sr, aux_float;

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;
  //Serial.print("  encoderIZQ: ");
  //Serial.print(encoderIZQ);
  //Serial.print("  encoderDER: ");
  //Serial.print(encoderDER);

  if (test_counter < 2000) {
    TEST_distances[test_counter] = dist_us_sensor_central;
    TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encDER);
  }

  // Encoder (below) is the difference of both encoders times the normalization constant accounting for the diameter's error
  aux_float = KKI * (float) temp_encIZQ - (float) temp_encDER * RADIO;

  if (aux_float > 0)
    aux_float += 0.499999;
  if (aux_float < 0)
    aux_float -= 0.499999;
  encoder = (int) aux_float;

  error = encoder_ant - encoder;
  encoder_ant = encoder;
  // Implement PID (just PD)
  // Right wheel speed is updated if it is not braking at the end of the movement
  if (breaking_period == 0) {
    aux_float = (float) encoder * kp1 - (float) error * kd1;

    if (aux_float > 0)
      aux_float += 0.5;
    if (aux_float < 0)
      aux_float -= 0.5;

    velr += (int) aux_float;

    velr=velr+SPEED_INI_L-vell;
    vell=SPEED_INI_L;

    speed_normalization();

    //Serial.print("VL: ");
    //Serial.print(vell);
    //Serial.print(" VR: ");
    //Serial.println(velr);
    // Write, as PWM duty cycles, the speeds for each wheel
    analogWrite(MOT_R_PWM_PIN, velr);
    analogWrite(MOT_L_PWM_PIN, vell);
//    delay(80);
  }
}  // End of straigh_dist()


///////////////////////////////////////////////////////////
//                  ONE_FASTER_DIST                      //
//  Control the PID algorithm when the right wheel must  //
//  be faster than the left one to perform a circular    //
//                     movement.                         //
///////////////////////////////////////////////////////////
void one_faster_dist() {
  int interval;
  unsigned int temp_encDER, temp_encIZQ;

  update_global_positions();

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;

  if (STATE == CIRC_TEST_R_STATE) {
    if (test_counter < 2000) {
      TEST_distances[test_counter] = dist_us_sensor_central;
      TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encDER);
    }
  }
  else if (STATE == CIRC_TEST_L_STATE) {
    if (test_counter < 2000) {
      TEST_distances[test_counter] = dist_us_sensor_central;
      TEST_pulses[test_counter++ ] = (unsigned int) (0x00FF & temp_encIZQ);
    }
  }

  if (radii_relation)
    if (STATE == RIGHT_FASTER_STATE)
      encoder = (int) (KKI * (float) temp_encIZQ - (float) temp_encDER * radii_relation + 0.499999);
    else
      encoder = (int) ((float) temp_encDER - (float) KKI * (float) temp_encIZQ * radii_relation + 0.499999);
  else
    encoder = 0;

  error = encoder_ant - encoder;
  encoder_ant = encoder;

  // Implement PID (just PD)
  // Right wheel speed is updated if it is not braking at the end of the movement
  if (breaking_period == 0) {
    velr += (encoder * kp1 - error * kd1);

    // Speeds are normalized in order to work at maximum speed
    speed_normalization();
  }

  // Write, as PWM duty cycles, the speeds for each wheel
  if (velr == 0) {
    digitalWrite(MOT_R_A_PIN, LOW);
    digitalWrite(MOT_R_B_PIN, LOW);
    analogWrite(MOT_R_PWM_PIN, 0);
  }
  else {
    if (dir_right == 2) {
      digitalWrite(MOT_R_A_PIN, HIGH);
      digitalWrite(MOT_R_B_PIN, LOW);
      analogWrite(MOT_R_PWM_PIN, velr);
    }
    else {
      digitalWrite(MOT_R_A_PIN, LOW);
      digitalWrite(MOT_R_B_PIN, HIGH);
      analogWrite(MOT_R_PWM_PIN, velr);
    }
  }
  if (vell == 0) {
    digitalWrite(MOT_L_A_PIN, LOW);
    digitalWrite(MOT_L_B_PIN, LOW);
    analogWrite(MOT_L_PWM_PIN, 255);
  }
  else {
    if (dir_left == 2) {
      digitalWrite(MOT_L_A_PIN, LOW);
      digitalWrite(MOT_L_B_PIN, HIGH);
      analogWrite(MOT_L_PWM_PIN, vell);
    }
    else {
      digitalWrite(MOT_L_A_PIN, HIGH);
      digitalWrite(MOT_L_B_PIN, LOW);
      analogWrite(MOT_L_PWM_PIN, vell);
    }
  }
}  // End of one_faster_dist()


///////////////////////
//  DISP_GLOBAL_POS  //
///////////////////////
void disp_global_pos()
{
  char cad[50];

  sprintf(cad, "X: %05d Y: %05d T: %03d", (int)X, (int)Y, (int)(Theta * (float)57.29));


  if (digitalRead(SW5_PIN) == LOW)
    Serial2.println(cad);
  else
    Serial.println(cad);


  /*
    SERIA.print(" X: ");
    SERIA.print((int) X);
    SERIA.print(" Y: ");
    SERIA.print((int) Y);
    //        SERIA.print(" Theta: (rad)");
    //        SERIA.print(Theta);
    SERIA.print(" T: ");
    SERIA.println((int)(Theta * 57.29));
  */
}  // End of disp_global_pos()


/////////////////////////////////////////////////////////////////////
//                  UPDATE GLOBAL POSITIONS                        //
//  Update the X position,Y position and orientation of the robot  //
//      using the enoderIZQ and encoderDER received pulses.        //
/////////////////////////////////////////////////////////////////////
void update_global_positions() {
  float s, sl, sr;
  unsigned int temp_encDER, temp_encIZQ;

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;
  sl = mmperpulse * KKI * (temp_encIZQ - aux_encoderIZQ);
  sr = mmperpulse * (temp_encDER - aux_encoderDER);
  aux_encoderDER = temp_encDER;
  aux_encoderIZQ = temp_encIZQ;

  if (dir_right == 0)
    sr = -sr;
  if (dir_left == 0)
    sl = -sl;
  Theta += (sr - sl) / WHEEL_DIST;
  while (Theta > 6.2832)
    Theta -= 6.28318531;
  while (Theta < -6.2832)
    Theta += 6.28318531;



  s = (sr + sl) / 2;

  X += s * cos(Theta);
  Y += s * sin(Theta);
}  // End of update_global_positions()


//////////////////////////////////////////
//                  DEP                 //
// This function is used for debugging  //
//////////////////////////////////////////
void dep1() {
  if (digitalRead(SW5_PIN) == LOW) {
    Serial2.print(" VR: ");
    Serial2.print(velr);
    Serial2.print(" VL: ");
    Serial2.print(vell);

    if (encoder > 0) {
      Serial2.print("  +");
      Serial2.print(encoder);
    }
    else {
      Serial2.print("  -");
      Serial2.print(-encoder);
    }

    if (error > 0) {
      Serial2.print("    Error: +");
      Serial2.print(error);
    }
    else {
      Serial2.print("    Error: -");
      Serial2.print(-error);
    }

    Serial2.print(" encoderDER: ");
    Serial2.print(encoderDER);
    Serial2.print(" encoderIZQ: ");
    Serial2.print(encoderIZQ);
    Serial2.print(" Igr_IZQ: ");
    Serial2.print(ignored_left);
    Serial2.print(" Igr_DER: ");
    Serial2.println(ignored_right);
  }
  else {
    Serial.print(" VR: ");
    Serial.print(velr);
    Serial.print(" VL: ");
    Serial.print(vell);

    if (encoder > 0) {
      Serial.print("  +");
      Serial.print(encoder);
    }
    else {
      Serial.print("  -");
      Serial.print(-encoder);
    }

    if (error > 0) {
      Serial.print("    Error: +");
      Serial.print(error);
    }
    else {
      Serial.print("    Error: -");
      Serial.print(-error);
    }

    Serial.print(" encoderDER: ");
    Serial.print(encoderDER);
    Serial.print(" encoderIZQ: ");
    Serial.print(encoderIZQ);
    Serial.print(" Igr_IZQ: ");
    Serial.print(ignored_left);
    Serial.print(" Igr_DER: ");
    Serial.println(ignored_right);

  }
}  // End of dep()


///////////////////////////////////////////////////////////////
//                      READ_NUMBER                          //
//  Read a number from the serial port with <number> digits  //
///////////////////////////////////////////////////////////////
short int read_number(int n_digits) {
  char speed[8];

  // Wait to be sure the bytes have arrived
  delay(5);
  //  sprintf(speed,"%d",SERIA.available());
  //  SERIA.println(speed);
  if (digitalRead(SW5_PIN) == LOW) {
    if (Serial2.available() > n_digits - 1)
    {
      Serial2.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
      //      SERIA.println(speed);

      return (unsigned int) atoi(speed);
    }
    else
    {
      ERROR_CODE = NO_NUMBER;
      return 0;
    }
  }
  else {
    if (Serial.available() > n_digits - 1)
    {
      Serial.readBytes(speed, n_digits);
      speed[n_digits] = '\0';  // Append a NULL character to terminate the string! Not needed if initialized with char speed[5] = {0}...
      //      SERIA.println(speed);

      return (unsigned int) atoi(speed);
    }
    else
    {
      ERROR_CODE = NO_NUMBER;
      return 0;
    }
  }
}  // End of read_number()


////////////////////////////////////////////////////////////////////////////
//                            PARSE_INPUT                                 //
// Read data from the serial port in order to update operation constants  //
////////////////////////////////////////////////////////////////////////////
void parse_input(void)
{
  char incoming_byte = '\0', incoming_number[N_DIGS] = {'\0'};
  int i = 0;
  if (digitalRead(SW5_PIN) == LOW) {
    while (Serial2.available() > 0) {
      incoming_byte = Serial2.read();
      if (incoming_byte <= '9' || incoming_byte == '.')
        incoming_number[i++] = incoming_byte;
      else {
        if (i >= 0)
          update_param(incoming_number, incoming_byte);
        i = 0;
        for (int k = 0; k < N_DIGS; k++)
          incoming_number[k] = '\0';
      }
    }
  }
  else {
    while (Serial.available() > 0) {
      incoming_byte = Serial.read();
      if (incoming_byte <= '9' || incoming_byte == '.')
        incoming_number[i++] = incoming_byte;
      else {
        if (i >= 0)
          update_param(incoming_number, incoming_byte);
        i = 0;
        for (int k = 0; k < N_DIGS; k++)
          incoming_number[k] = '\0';
      }
    }
  }
  return;
}  // End of parse_input()


//////////////////////////////////////////////////////
//                    UPDATE_PARAM                  //
// Update operation constants based on parsed data  //
//////////////////////////////////////////////////////
void update_param( char* number,  char parameter)
{
  switch (parameter)
  {
    case 'W': //Adjust the Wheelbase
      WHEEL_DIST = atof(number);
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("The parameter we are updating is: ");
        Serial2.print("Wheelbase");
        Serial2.print(" with a value of: ");
        Serial2.println(WHEEL_DIST);
      }
      else {
        Serial.print("The parameter we are updating is: ");
        Serial.print("Wheelbase");
        Serial.print(" with a value of: ");
        Serial.println(WHEEL_DIST);
      }
      break;
    case 'M': //Adjust the MM to pulses factor
      mmperpulse = atof(number);
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("The parameter we are updating is: ");
        Serial2.print("MM per pulse");
        Serial2.print(" with a value of: ");
        Serial2.println(mmperpulse);
      }
      else {
        Serial.print("The parameter we are updating is: ");
        Serial.print("MM per pulse");
        Serial.print(" with a value of: ");
        Serial.println(mmperpulse);
      }
      break;
    case 'D': //Adjust the diameter's error
      KKI = atof(number);
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("The parameter we are updating is: ");
        Serial2.print("KKI");
        Serial2.print(" with a value of: ");
        Serial2.println(KKI);
      }
      else {
        Serial.print("The parameter we are updating is: ");
        Serial.print("KKI");
        Serial.print(" with a value of: ");
        Serial.println(KKI);
      }
      break;
    case 'R': //Adjust the number of turns during calibration
      c_factor = atoi(number);
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("The parameter we are updating is: ");
        Serial2.print("Correction factor");
        Serial2.print(" with a value of: ");
        Serial2.println(c_factor);
      }
      else {
        Serial.print("The parameter we are updating is: ");
        Serial.print("Correction factor");
        Serial.print(" with a value of: ");
        Serial.println(c_factor);
      }
      break;
    case 'S':
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.println("Paramenter values: ");
        Serial2.print("\tWheelbase: ");
        Serial2.println(WHEEL_DIST, 10);
        Serial2.print("\tMM per pulse: ");
        Serial2.println(mmperpulse, 10);
        Serial2.print("\tKKI: ");
        Serial2.println(KKI, 10);
        Serial2.print("\tC Factor: ");
        Serial2.println(c_factor);
      }
      else {
        Serial.println("Paramenter values: ");
        Serial.print("\tWheelbase: ");
        Serial.println(WHEEL_DIST, 10);
        Serial.print("\tMM per pulse: ");
        Serial.println(mmperpulse, 10);
        Serial.print("\tKKI: ");
        Serial.println(KKI, 10);
        Serial.print("\tC Factor: ");
        Serial.println(c_factor);
      }
      break;
    default:

      if (digitalRead(SW5_PIN) == LOW)
        Serial2.println("ERROR!");
      else
        Serial.println("ERROR!");
  }
  return;
}  // End of update_param()


////////////////////////////////////////////////////////////////////////
//                            ANALYZE_ORDER                           //
// Parse command information and trigger all the necessary functions  //
////////////////////////////////////////////////////////////////////////
void analyze_order() {
  char str[35];
  float s, sl, sr;
  int num, n;

  switch (order[0])
  {
    // '0'  rotate unclockwise with different speeds
    //      Right wheel is faster
    // '1'  rotate clockwise with different speeds
    //      Left Wheel is faster
    // 'C'  Test circular right
    // 'D'  Test circular left
    //      make a circular move for at least 10 rounds and in the loop the keep the
    //      from central sensor to the object in from and the number of pulses
    case   0x30:  // '0'
    case   0x31:  // '1'
    case   0x43:  // 'C'
    case   0x44:  // 'D'
    case   0x4A:  // 'J'
    case   0x4B:  // 'K'
      // The test is done with a lower speed to reduce the error
      if (order[0] == 0x43 || order[0] == 0x44) {
        test_counter = 0;
        SPEED_INI_R = 220;
        SPEED_INI_L = 220;
      }
      else
        SPEED_INI_R = SPEED_INI_L = 255;

      // The command's syntax is "0XXXYYYY", where XXX is a 3 byte string that indicates the radii ratio * 999
      num = read_number(3);
      //      SERIA.print(" ");
      //      SERIA.println(num);
      if (num < 0 || num > 999) {
        ERROR_CODE = RR_OUT_RANGE;
        break;
      }
      // For a circular test we must ensure that num == 0 to avoid the error, so the code for test is: C000, D000
      if (order[0] == 0x43 || order[0] == 0x44)
        if (num != 0) {
          ERROR_CODE = RR_OUT_RANGE;
          break;
        }
      radii_relation = (float) num / 999;

      /* The command's syntax is "0XXXYYYY" where YYYY is a 4 bytes string indicating the distance that the faster wheel should
         traverse. For this circular test the distance is given in cm (* 10 factor), as we will be traersing large distances to
         reduce the non-systematic and estimation errors when computing the number of pulses per turn */
      num = read_number(4);
      if (num < 0 || num > 9999) {
        ERROR_CODE = OUT_RANGE;
        break;
      }

      if (order[0] == 0x43 || order[0] == 0x44)
        PULSES_NUM = (unsigned int) ( 10 * num / mmperpulse);
      else
        PULSES_NUM = (unsigned int) (num / mmperpulse);

      // The is the minimum space to move
      if (PULSES_NUM > INERTIA_LIMIT) {
        switch (order[0]) {
          case '0':
            STATE = RIGHT_FASTER_STATE;
            dir_right = dir_left = 1;
            SPEED_INI_L = (int) (SPEED_INI_R * radii_relation * 0.8);
            move_motors();
            break;

          case '1':
            STATE = LEFT_FASTER_STATE;
            dir_right = dir_left = 1;
            SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
            move_motors();
            break;

          case 'C':
            STATE = CIRC_TEST_R_STATE;
            dir_right = dir_left = 1;
            SPEED_INI_L = (int) (SPEED_INI_R * radii_relation * 0.8);
            move_motors();
            break;

          case 'D':
            STATE = CIRC_TEST_L_STATE;
            dir_right = dir_left = 1;
            SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
            move_motors();
            break;

          case 'J':
            STATE = RIGHT_FASTER_STATE;
            dir_right = 2;
            dir_left = 0;
            SPEED_INI_L = (int) (SPEED_INI_R * radii_relation * 0.8);
            move_motors();
            break;

          case 'K':
            STATE = LEFT_FASTER_STATE;
            dir_right = 0;
            dir_left = 2;
            SPEED_INI_R = (int) (SPEED_INI_L * radii_relation * 0.8);
            move_motors();
        }
      }
      else
        ERROR_CODE = OUT_RANGE;
      break;

    // '2' -> Rotate counterclockwise with respect to the wheel's axis center
    // '3' -> Rotate clockwise with respect to the wheel's axis center
    case   0x32: // '2'
    case   0x33: // '3'
      encoderDER = 0;
      aux_encoderDER = 0 ;
      aux_encoderIZQ = 0 ;
      encoderIZQ = 0;
      SPEED_INI_R = SPEED_INI_L = 160;

      test_counter = 0;

      radii_relation = 1;
      // The command's syntax is "3XXX", where XXX is a 3 byte string indicating the rotation in degrees
      num = read_number(3);

      if (num && num < 361) {
        PULSES_NUM = c_factor * ((num * 3.1416 * WHEEL_DIST) / (360 * mmperpulse));
        if (order[0] == '2') {
          clockwise = dir_left = 0;
          STATE = ROTATE_CCW_STATE;
          dir_right = 1;
          move_motors();
        }
        else {
          clockwise = dir_left = 1;
          STATE = ROTATE_CW_STATE;
          dir_right = 0;
          move_motors();
        }
      }
      else
        ERROR_CODE = OUT_RANGE;
      break;
    /* ## Move forward ##
      Command: "4XXXX", where XXXX is a 4 byte string representing the distance in mm.
      ## Move backward ##
      Command: "5XXXX", where XXXX is a 4 byte string representing the distance in mm.
      We recomend to traverse small distances in backwards movement due to the lack of an US sensor at the rear...
      ## Move forward at a specific speed ##
      Command: "6XXXXYYY", where XXXX is a 4 byte string representing the distance in mm and YYY is the maximum speed (PWM Duty Cycle)
      ## Move backward at specific speed ##
      Command: "7XXXXYYY", where XXXX is a 4 byte string representing the distance in mm and YYY is the maximum speed (PWM Duty Cycle)
      We recomend to traverse small distances in backwards movement due to the lack of an US sensor at the rear... */
    case   0x34: // '4'
    case   0x35: // '5'
    case   0x36: // '6'
    case   0x37: // '7'
      encoderDER = 0;
      aux_encoderDER = 0 ;
      aux_encoderIZQ = 0 ;
      encoderIZQ = 0;
      SPEED_INI_R = SPEED_INI_L = 150;
      radii_relation = 1;
      num = read_number(4);
      if (num) {
        PULSES_NUM = num / mmperpulse;
        // The is the minimum space to move
        if (PULSES_NUM > INERTIA_LIMIT) {
          if (order[0] == 0x36 || order[0] == 0x37) {
            num = read_number(3);
            if (num && num < 256)
              SPEED_INI_R = SPEED_INI_L = num;
            else
              ERROR_CODE = SPEED_OUT_RANGE;
          }
          if (ERROR_CODE == NO_ERROR)
          {
            STATE = MOVE_STRAIGHT_STATE;
            if (order[0] == 0x34 ||
                order[0] == 0x36)
              dir_right = dir_left = 1;
            else
              dir_right = dir_left = 0;
            //      Serial.print("  TT Encoder:  ");
            //      Serial.print(encoderDER);
            //      Serial.print("  Pulses: ");
            //      Serial.println(PULSES_NUM);
            //            Serial.println("Comando movimiento");
            move_motors();
          }
        }
        else
          ERROR_CODE = INERTIA_LIMIT_ERROR;
      }
      else
        ERROR_CODE = OUT_RANGE;
      break;

    ////////////////////////////////////////
    //    Control por velocidad
    ////////////////////////////////////////
    case   'V':
      velDER = read_number(3);
      velIZQ = read_number(3);

      if (velDER == 255)
        velDER = 254;
      if (velIZQ == 255)
        velIZQ = 254;

      if (STATE == RESET_STATE)
      {
        flag_move_motor = 1;
      }

      if (velDER == 0  && velIZQ == 0 )
        STATE = RESET_STATE;
      else
        STATE = MOVE_DIF_SPEED;
      SPEED_INI_R = SPEED_INI_L = 255;
      movimientos_vel();

      /*      if (veloc_left>10 && veloc_left<281 && veloc_right>10 && veloc_right<281)
            {
              if (ERROR_CODE == NO_ERROR)
              {
                STATE = MOVE_DIF_SPEED;
                dir_right = dir_left = 1;
                update_speeds(1); // Reset the aux encoder variables
                move_motors();
              }
            }
            else
              ERROR_CODE = OUT_RANGE;
      */
      break;
    ////////////////////////////////////////
    //    Control por velocidad
    ////////////////////////////////////////

    // Patrol command
    case 'P':
      X = 2500;
      Y = 0;
      Theta = 0;
      //SERIA.println("Comand P");
      num = read_number(3);
      //SERIA.println("Comand P2");
      if (num == 0)
      {
        if (digitalRead(SW5_PIN) == LOW)
          Serial2.println("Comand P3");
        else
          Serial.println("Comand P3");
        STATE = PATROL_STATE;
        estado_patrol = PATROL_REPOSO;
        if (patrol() == 0)
          STATE = RESET_STATE;
      }
      else
        ERROR_CODE = OUT_RANGE;

      break;

    // Bumping and falling Sensors status
    case 0x39: // '9'
      for (int k = 0; k < 6; k++)
        str[k] = '1';
      str[6] = '\0';
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("Sensors :");
        Serial2.println(str);
      }
      else {
        Serial.print("Sensors :");
        Serial.println(str);
      }
      break;

    // Send back the last measurements from the US sensors
    case 0x3A: // ':'
      disp_lect_sensores();
      break;

    // Reset the global position
    case 0x3C: // '<'
      X = Y = Theta = 0;
      if (digitalRead(SW5_PIN) == LOW)
        Serial2.println(0x15);
      else
        Serial.println(0x15);
      break;

    // Send back the firmware's version
    case 0x3E: // '>'
      if (digitalRead(SW5_PIN) == LOW)
        Serial2.println(VERSION);
      else
        Serial.println(VERSION);
      break;

    // Stop motors and calcucompute the new positions
    case 0x3F: // '?'
      stop_motors();
      STATE = RESET_STATE;
      update_global_positions();
      /*
        sl = mmperpulse * (encoderIZQ - aux_encoderIZQ);
        sr = mmperpulse * (encoderDER - aux_encoderDER);
        if (dir_right == 0)
        sr = -sr;
        if (dir_left == 0)
        sl =- sl;
        Theta += (sr - sl) / WHEEL_DIST;
        s = (sr + sl) / 2;

        X += s * cos(Theta);
        Y += s * sin(Theta);
      */
      //      dep1();
      break;

    // Send back the global position variables. The position is in mm and the orientation in degrees
    case 0x41: // 'A'
      disp_global_pos();
      //      SERIA.print(" encoderDER ");
      //      SERIA.print(encoderDER);
      //      SERIA.print(" encoderIZQ ");
      //      SERIA.println(encoderIZQ);
      break;
    // Send back the state of the system. STATE is the variable and the different states are #define(d) at the beginning
    // The state is sent by adding 0x30
    case 0x42: // 'B'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("B");
        Serial2.write(0x30 + STATE);
        Serial2.println("");
      }
      else {
        Serial.print("B");
        Serial.write(0x30 + STATE);
        Serial.println("");
      }
      break;

    // The error is sent back by adding 0x30 to the error code
    case 0x45: // 'E'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.write(0x45);  // 'E'
        Serial2.write(0x30 + ERROR_CODE);
        Serial2.println("");
      }
      else {
        Serial.write(0x45);  // 'E'
        Serial.write(0x30 + ERROR_CODE);
        Serial.println("");
      }
      break;
    case 'Q': // 'Q'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("\nQ:");
        Serial2.print(encoderIZQ);
        Serial2.print(" ");
        Serial2.println(encoderDER);
      }
      else {
        Serial.print("\nQ:");
        Serial.print(encoderIZQ);
        Serial.print(" ");
        Serial.println(encoderDER);
      }
      break;

    case 'N': //'N'    Datos de los encoders en valor absoluto
      enviar_datos_encoder();
      break;

    case 0x46: // 'F'
      if (digitalRead(SW5_PIN) == LOW) {
        Serial2.print("Counter :");
        Serial2.println(test_counter);
        for (n = 1; n < test_counter; n++) {
          Serial2.print(" ");
          Serial2.print(TEST_pulses[n]);
          Serial2.print(" ");
          Serial2.print(TEST_distances[n]);
          Serial2.println(" ;... ");
          delay(50);
        }
        Serial2.println("END");
      }
      else {
        Serial.print("Counter :");
        Serial.println(test_counter);
        for (n = 1; n < test_counter; n++) {
          Serial.print(" ");
          Serial.print(TEST_pulses[n]);
          Serial.print(" ");
          Serial.print(TEST_distances[n]);
          Serial.println(" ;... ");
          delay(50);
        }
        Serial.println("END");
      }
      break;
    case 0x53: // 'S'
      parse_input();
      break;
    default:
      ERROR_CODE = NO_AVAILABLE;
  }
}  // End of analyze_order()


////////////////////////////////////////
//    Control por velocidad
////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// This function perform a PID for speeds control:
// The global variables with the objective speeds are:
//          veloc_right
//          veloc_left
//
// The global variables with the speed limits are:
//          SPEED_INI_R
//          SPEED_INI_L

// Input parameter:
//  unsigned char reset, if reset==1 the initial values are set in variables.
////////////////////////////////////////////////////////////////////////
void update_speeds(unsigned char reset)
{
  static unsigned int encI_aux = 0, encD_aux = 0;
  unsigned int temp_encDER, temp_encIZQ;
  float aux_floatDER, aux_floatIZQ;
  unsigned long t;
  long tiempo;

  float errorD, errorI;
  static float ant_errorD = 0, ant_errorI = 0;

  unsigned int real_vI, real_vD;

  static float KP = (float)0.3; //0.7
  static float KD = (float)0.5; // 1

  //  Serial.print("reset ");
  //  Serial.print(reset);


  if (reset == 1)
  {
    encI_aux = encoderIZQ;
    encD_aux = encoderDER;
    ant_errorD = 0;
    ant_errorI = 0;
    atr = micros();
    // velr=veloc_right;
    // vell=veloc_left;
    return;
  }


  tr = micros();
  tiempo = (long)tr - (long)atr;
  if (tiempo < 90000)
    return;
  atr = tr;

actualizar_encoder_abs();

  //Serial.print("tiempo  ");
  //Serial.print(tiempo);

  temp_encDER = encoderDER;
  temp_encIZQ = encoderIZQ;

  //Serial.print(" encI ");
  //Serial.print(encoderIZQ);
  //Serial.print(" encD ");
  //Serial.print(encoderDER);
  //Serial.print(" encI_aux ");
  //Serial.print(encI_aux);
  //Serial.print(" encD_aux ");
  //Serial.print(encD_aux);


  real_vD = (unsigned int) ((float)temp_encDER - (float)encD_aux) / ((float)tiempo / (float)1000000);
  real_vI = (unsigned int) ((float)temp_encIZQ - (float)encI_aux) / ((float)tiempo / (float)1000000);

  //  Serial.print(SPEED_INI_R);
  //  Serial.print(" ");
  
  
  //  Serial.print(" realvD ");
  //  Serial.print(real_vD);
  //  Serial.print(" real_vI ");
  //  Serial.print(real_vI);
  
  encD_aux = temp_encDER;
  encI_aux = temp_encIZQ;
  
/*    Serial.print(" velecR ");
    Serial.print(veloc_right);
    Serial.print(" velocL ");
    Serial.print(veloc_left);
  */
  aux_floatDER = (float)veloc_right - (float)real_vD;
  aux_floatIZQ = (float)veloc_left  - (float)real_vI;

  errorD = ant_errorD - aux_floatDER;
  errorI = ant_errorI - aux_floatIZQ;
  ant_errorD = aux_floatDER;
  ant_errorI = aux_floatIZQ;

  
   
  // Serial.print("  fDER ");
  //Serial.print(aux_floatDER);
  //Serial.print("  fIZQ ");
  //Serial.print(aux_floatIZQ);
 
  //Serial.print("    errorD  ");
  //Serial.print(errorD);
  //Serial.print("   ");
  //Serial.print(errorI);

  // Implement PID (just PD)
  aux_floatDER = (float) aux_floatDER * KP - (float) errorD * KD;
  aux_floatIZQ = (float) aux_floatIZQ * KP - (float) errorI * KD;

  //Serial.print("  inc ");
  //Serial.print(aux_floatDER);
  //Serial.print("   ");
  //Serial.print(aux_floatIZQ);

  if (aux_floatDER > 0)
    aux_floatDER += 0.5;
  if (aux_floatDER < 0)
    aux_floatDER -= 0.5;

  //Serial.print("   ");
  //Serial.print(aux_floatDER);

  if (aux_floatIZQ > 0)
    aux_floatIZQ += 0.5;
  if (aux_floatIZQ < 0)
    aux_floatIZQ -= 0.5;


  /*    if (aux_float > 0)
        aux_float += 0.5;
      if (aux_float < 0)
        aux_float -= 0.5;
  */

  //Serial.print("    velr ");
  //Serial.print(velr);
  //Serial.print(" velI ");
  //Serial.println(vell);

  if ((velr + aux_floatDER) < 0)
    velr = 0;
  else
    velr += (int)aux_floatDER;
  if ((vell + aux_floatIZQ) < 0)
    vell = 0;
  else
    vell += (int)aux_floatIZQ;

  /*  Serial.print(" velr ");
    Serial.print(velr);
    Serial.print(" velI ");
    Serial.println(vell);
  */
  if (velr < 0)
    velr = 0;
  if (vell < 0)
    vell = 0;
  if (velr > SPEED_INI_R)
    velr = SPEED_INI_R;
  if (vell > SPEED_INI_L)
    vell = SPEED_INI_L;

  if (STATE != CAN_STATE && STATE != BOTON_STATE)
    if (vell > SPEED_INI_L || velr > SPEED_INI_R)
    {
      //    Serial.println(" Speed Nor ");
      speed_normalization();
    }
  //Serial.print("  velr: ");
  //Serial.print(velr);
  //Serial.print("  vell: ");
  //Serial.print(vell);
  //Serial.println("  ");
  // Write, as PWM duty cycles, the speeds for each wheel
  analogWrite(MOT_R_PWM_PIN, velr);
  analogWrite(MOT_L_PWM_PIN, vell);

}  // fin de void update_speeds()

////////////////////////////////////////
//    Control por velocidad
////////////////////////////////////////

// OJO CAN
////////////////////////////////////////////////////////////////
void enviar_trama_periodica_can()
{
  struct can_frame canMsg1;
  unsigned char byte_auxiliar;

  canMsg1.can_id  = 0x102;
  canMsg1.can_dlc = 8;

  /* ORIGINAL
    byte_auxiliar=(encoderAAbs & 0xFF000000)>>24;
    canMsg1.data[0] = byte_auxiliar;
    byte_auxiliar=(encoderAAbs & 0x00FF0000)>>16;
    canMsg1.data[1] = byte_auxiliar;
    byte_auxiliar=(encoderAAbs & 0x0000FF00)>>8;
    canMsg1.data[2] = byte_auxiliar;
    byte_auxiliar=(encoderAAbs & 0x000000FF);
    canMsg1.data[3] = byte_auxiliar;
  */
  actualizar_encoder_abs();


  byte_auxiliar = (encoderAAbs & 0xFF000000) >> 24;
  canMsg1.data[3] = byte_auxiliar;
  byte_auxiliar = (encoderAAbs & 0x00FF0000) >> 16;
  canMsg1.data[2] = byte_auxiliar;
  byte_auxiliar = (encoderAAbs & 0x0000FF00) >> 8;
  canMsg1.data[1] = byte_auxiliar;
  byte_auxiliar = (encoderAAbs & 0x000000FF);
  canMsg1.data[0] = byte_auxiliar;

  tAabs = micros();
  byte_auxiliar = (tAabs & 0xFF000000) >> 24;
  canMsg1.data[7] = byte_auxiliar;
  byte_auxiliar = (tAabs & 0x00FF0000) >> 16;
  canMsg1.data[6] = byte_auxiliar;
  byte_auxiliar = (tAabs & 0x0000FF00) >> 8;
  canMsg1.data[5] = byte_auxiliar;
  byte_auxiliar = (tAabs & 0x000000FF);
  canMsg1.data[4] = byte_auxiliar;

  //  if (mcp2515.sendMessage(&canMsg1) != MCP2515::ERROR_OK)
  //    Serial.print("Msg1 TX error  ");
  mcp2515.sendMessage(&canMsg1);


  canMsg1.can_id  = 0x101;
  canMsg1.can_dlc = 8;

  byte_auxiliar = (encoderBAbs & 0xFF000000) >> 24;
  canMsg1.data[3] = byte_auxiliar;
  byte_auxiliar = (encoderBAbs & 0x00FF0000) >> 16;
  canMsg1.data[2] = byte_auxiliar;
  byte_auxiliar = (encoderBAbs & 0x0000FF00) >> 8;
  canMsg1.data[1] = byte_auxiliar;
  byte_auxiliar = (encoderBAbs & 0x000000FF);
  canMsg1.data[0] = byte_auxiliar;

  tBabs = micros();
  byte_auxiliar = (tBabs & 0xFF000000) >> 24;
  canMsg1.data[7] = byte_auxiliar;
  byte_auxiliar = (tBabs & 0x00FF0000) >> 16;
  canMsg1.data[6] = byte_auxiliar;
  byte_auxiliar = (tBabs & 0x0000FF00) >> 8;
  canMsg1.data[5] = byte_auxiliar;
  byte_auxiliar = (tBabs & 0x000000FF);
  canMsg1.data[4] = byte_auxiliar;

  //  if (mcp2515.sendMessage(&canMsg1) != MCP2515::ERROR_OK)
  //    Serial.print("Msg1 TX error  ");
  mcp2515.sendMessage(&canMsg1);


}  // fin de enviar_trama_periodica_can()
// OJO CAN

void actualizar_encoder_abs()
{
  if (dir_left == 0)
  {
    encoderIZQ_aux_can = (long)encoderIZQ_aux_can - (long)encoderIZQ;
    encoderAAbs = (long)encoderAAbs + (long)encoderIZQ_aux_can;

/*        
       Serial.print(encoderAAbs);
        Serial.print(" ");
        Serial.print(encoderIZQ_aux_can);
        Serial.print(" ");
        Serial.println(encoderIZQ);
*/
    encoderIZQ_aux_can = encoderIZQ;

  }
  else
  {
    encoderIZQ_aux_can = (long)encoderIZQ_aux_can - (long)encoderIZQ;
    encoderAAbs = (long)encoderAAbs - (long)encoderIZQ_aux_can;
 /*       Serial.print(encoderAAbs);
        Serial.print(" ");
        Serial.print(encoderIZQ_aux_can);
        Serial.print(" ");
        Serial.println(encoderIZQ);
*/
    encoderIZQ_aux_can = encoderIZQ;
  }
  if (dir_right == 0)
  {
    encoderDER_aux_can = (long)encoderDER_aux_can - (long)encoderDER;
    encoderBAbs = (long)encoderBAbs + (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
  }
  else
  {
    encoderDER_aux_can = (long)encoderDER_aux_can - (long)encoderDER;
    encoderBAbs = (long)encoderBAbs - (long)encoderDER_aux_can;
    encoderDER_aux_can = encoderDER;
  }



}  // fin actualizar_encoder_abs
////////////////////////////////////////////////////////////////


// OJO CAN RECEPCION DE COMANDOS
////////////////////////////////////////////////////////////////
void movimientos_vel()
{
  
  actualizar_encoder_abs();

  if (velDER == 0)
  {
    if (veloc_right != 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    veloc_right = 0;
    dir_right = 0;
  }
  else if (velDER < 128)
  {
    if (dir_right == 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //veloc_right=10+280*(unsigned int)velDER/128;
    veloc_right=(unsigned int)(PASOS_POR_VUELTA*velDER/(2*PI*Kdato)+0.5);

    dir_right = 1;
  }
  else
  {
    if (dir_right == 1)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //velr=255+127-velDER;
    velDER = 256 - velDER;
    //veloc_right=10+280*(unsigned int)velDER/128;
    veloc_right=(unsigned int)(PASOS_POR_VUELTA*velDER/(2*PI*Kdato)+0.5);

    dir_right = 0;
  }
/*  Serial.print("DEMO ");
  Serial.print(PASOS_POR_VUELTA);
  Serial.print(" ");
  Serial.print(velDER);
  Serial.print(" ");
  Serial.print(PI);
  Serial.print(" ");
  Serial.println(Kdato);
  */
  
  

  if (velIZQ == 0)
  {
    if (veloc_left != 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    veloc_left = 0;
    dir_left = 0;
  }
  else if (velIZQ < 128)
  {
    if (dir_left == 0)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //veloc_left=10+280*(unsigned int)velIZQ/128;
    veloc_left=(unsigned int)(PASOS_POR_VUELTA*velIZQ/(2*PI*Kdato)+0.5);
    dir_left = 1;
  }
  else
  {
    if (dir_left == 1)
    {
      stop_motors();
      flag_move_motor = 1;
    }
    //vell=255+127-velIZQ;
    velIZQ = 256 - velIZQ;
    //veloc_left=10+280*(unsigned int)velIZQ/128;
    veloc_left=(unsigned int)(PASOS_POR_VUELTA*velIZQ/(2*PI*Kdato)+0.5);
    dir_left = 0;
  }
  

  //Serial.println(trama_can_recibida.data[0]);
  //Serial.println(trama_can_recibida.data[1]);
  //Serial.println(velr);
  //Serial.println(vell);
  //Serial.println(dir_right);
  //Serial.println(dir_left);
  //Serial.println(veloc_right);

  if (veloc_left == 0)
    SPEED_INI_L = 0;
  //  else
  //    SPEED_INI_L=255;
  if (veloc_right == 0)
    SPEED_INI_R = 0;
  //  else
  //    SPEED_INI_R=255;

  if (flag_move_motor == 1)
  {
    move_motors();
    //    Serial.print(" Reset movimiento ");
    flag_move_motor = 0;
  }
  update_speeds(1);

  /*
    //Deactivate both motor's H-bridge
    PORTA &= 0xAA;

    if (!velr)
      PORTB |= 0x08;
    else
    {
      inhib_r |= 0x44;
      analogWrite(MOT_R_PWM_PIN, velr);
    }

    if (!vell)
      PORTB |= 0x10;
    else
    {
      inhib_l |= 0x11;
      analogWrite(MOT_L_PWM_PIN, vell);
    }

    // Programación de los pines en función de las cuatro
    // posibles combinaciones de direcciones de las ruedas
    if (dir_right && dir_left)
      PORTA |= 0x05 & inhib_r & inhib_l;
    else if (!dir_right && dir_left)
      PORTA |= 0x41 & inhib_r & inhib_l;
    else if (dir_right && !dir_left)
      PORTA |= 0x14 & inhib_r & inhib_l;
    else
      PORTA |= 0x50 & inhib_r & inhib_l;

  */

} // fin de movimientos_vel()

////////////////////////////////////////////////////////////////
// OJO CAN RECEPCION DE COMANDOS

/////////////////////////////////////////////////////////////////////
//   Enviar datos encoder
/////////////////////////////////////////////////////////////////////

void enviar_datos_encoder() {

  byte byte_auxiliar[4];

  actualizar_encoder_abs();
  tAabs = micros();
  tBabs = micros();

  byte_auxiliar[3] = (encoderAAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (encoderAAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (encoderAAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (encoderAAbs & 0x000000FF);

  Serial.println("N");   ///// Se manda una 'N'

  //Serial.print("N ");
  //Serial.print("  ");

  Serial.write(byte_auxiliar, 4);

   //DEBUG
  //Serial.print(" Dep EncoderAABS: ");
  //Serial.print(encoderAAbs);
  //Serial.print("  ");
  // FIN DEBUG

  byte_auxiliar[3] = (tAabs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (tAabs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (tAabs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (tAabs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);

   //DEBUG
  //Serial.print("  Dep tAABS: ");
  //Serial.print(tAabs);
  //Serial.print("  ");
  // FIN DEBUG

  byte_auxiliar[3] = (encoderBAbs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (encoderBAbs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (encoderBAbs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (encoderBAbs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);

  //DEBUG
  //Serial.print("  Dep EncoderBABS: ");
  // Serial.print(encoderBAbs);
  //Serial.print("  ");
 // FIN DEBUG

  byte_auxiliar[3] = (tBabs & 0xFF000000) >> 24;
  byte_auxiliar[2] = (tBabs & 0x00FF0000) >> 16;
  byte_auxiliar[1] = (tBabs & 0x0000FF00) >> 8;
  byte_auxiliar[0] = (tBabs & 0x000000FF);

  Serial.write(byte_auxiliar, 4);
   //DEBUG
  //Serial.print("  Dep tBABS: ");
  //Serial.print(tBabs);
  // FIN DEBUG
  Serial.println("P");   ///// Se manda una 'P'

}  //// Fin enviar los datos de los encoders

////////////////////////////////////////////////////////////////////////
//                            Lola                           //
// Loop function                                             //
////////////////////////////////////////////////////////////////////////
void Lola()
{
  static unsigned char us_sensor = 0;
  unsigned char aux, first_time = 1, flag = 1, dbg_counter = 0;
  static unsigned long time = 0, time1 = 0, time_can = 0;
  static unsigned int reference, last_pulses = 0, current_pulses;
  unsigned int auxiliar_uns_int;
  unsigned int msec_PID=0;

  static unsigned int flag_modo_pc;
  struct can_frame canMsg, canMsg2;

  canMsg.can_id  = 0x111;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x03;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;

  canMsg2.can_id  = 0x110;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x00;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x00;
  canMsg2.data[4] = 0x00;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0x03;

  mcp2515.sendMessage(&canMsg);
  mcp2515.sendMessage(&canMsg2);

  while (1)
  {

    if (digitalRead(SW3_PIN) == HIGH)
    {
      if (digitalRead(SW5_PIN) == LOW)
        Serial2.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
      else
        Serial.println("Seleccione mediante el SWITCH de la placa un modo de funcionamiento");
      break;
    }

    //    // OJO CAN
    //    ////////////////////////////////////////////////////////////////
    //    // De forma periódica se envían mensajes de control: cada 100 ms
    //    if((millis() - time_can) > 98)
    //    {
    //      enviar_trama_periodica_can();
    //
    ////      if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK)
    ////        Serial.print("Error trama modo PC  ");
    ////      mcp2515.sendMessage(&canMsg);
    //
    ////      if (mcp2515.sendMessage(&canMsg2) != MCP2515::ERROR_OK)
    ////        Serial.print("Error trama modo PC 2  ");
    ////      mcp2515.sendMessage(&canMsg2);
    //
    //      time_can=millis();
    //    }
    //    // OJO CAN
    //    ////////////////////////////////////////////////////////////////


    //    // OJO CAN RECEPCION DE COMANDOS
    //    ////////////////////////////////////////////////////////////////
    //    if (mcp2515.readMessage(&trama_can_recibida) == MCP2515::ERROR_OK)
    //      if (trama_can_recibida.can_id==0x120)
    //        if (trama_can_recibida.can_dlc==8)
    //        {
    //          velDER=trama_can_recibida.data[0];
    //          velIZQ=trama_can_recibida.data[1];
    //          if(velDER == 255)
    //            velDER = 254;
    //          if(velIZQ == 255)
    //            velIZQ = 254;
    //
    //          if (STATE==RESET_STATE)
    //          {
    //            flag_move_motor=1;
    //          }
    //
    //          if (velDER==0  && velIZQ==0 )
    //            STATE=RESET_STATE;
    //          else
    //            STATE=CAN_STATE;
    //
    //          SPEED_INI_R=SPEED_INI_L=255;
    //          movimientos_vel();
    //        }
    //
    //    ////////////////////////////////////////////////////////////////
    //    // OJO CAN RECEPCION DE COMANDOS

    //    US_sensor_read_sequence();
    /*
      // Sequence to read the ultra-sound sensors. In each iteration one sensor is read. We must take into account that we get a
      // delay by reading the ultra-sound sensor...
      if (us_sensor == 0)
      dist_us_sensor_central = us_range(F_US_TRIG, F_US_ECHO);
      else if (us_sensor == 2)
      dist_us_sensor_left = us_range(L_US_TRIG, L_US_ECHO);
      else if (us_sensor == 3)
      dist_us_sensor_right = us_range(R_US_TRIG, R_US_ECHO);
      else if (us_sensor == 4)
      dist_us_sensor_back = us_range(B_US_TRIG, B_US_ECHO);

      if (++us_sensor == 5)
      us_sensor = 0;
    */
    //  Read from the serial port if there is anything available
    if (digitalRead(SW5_PIN) == LOW)
    {
      if (Serial2.available() > 0)
      {
        order[0] = 'Z';
        theta_max = 0;
        //    Serial2.readBytes(order, 1);

        // Every time a command is going to be reveived the error is reset if it's not asking for an error code
        if (order[0] != 'E')
          ERROR_CODE = NO_ERROR;

        analyze_order();

        // Clean the buffer
        // No clean buffer to allow multiple order
        //while(SERIA.available())
        //  SERIA.readBytes(&aux, 1);
      }
    }
    else
    {
      if (Serial.available() > 0)
      {
        order[0] = 'Z';
        theta_max = 0;
        Serial.readBytes(order, 1);
        //    Serial.print(order[0]);
        // Every time a command is going to be reveived the error is reset if it's not asking for an error code
        if (order[0] != 'E')
          ERROR_CODE = NO_ERROR;
        //    Serial.print(order[0]);
        //    Serial.print("   ");
        //    Serial.println(STATE);
        analyze_order();
        //    Serial.print("  Recibidos  ");
        //    Serial.println(STATE);

        // Clean the buffer
        // No clean buffer to allow multiple order
        //while(SERIA.available())
        //  SERIA.readBytes(&aux, 1);
      }
    }

    
    // Each state performs a different movement
    switch (STATE)
    {
      case   CAN_STATE:
        update_speeds(0);
        break;
      case   RESET_STATE:

        RADIO = 1;
        vigila_patrol = 0;
        first_time = 1;
        breaking_period = 0;
        if ((millis() - time1) > 5000)
        {
          if (digitalRead(SW5_PIN) == LOW)
          {
            Serial2.print("B");
            Serial2.write(0x30 + STATE);
          }
          else
          {
            //Serial.print("B");
            //Serial.write(0x30 + STATE);
          }
          time1 = millis();
        }
        //Serial.print(".");
        // Just in case an error produced the movement of the motors
        stop_motors();
        break;

      // Movement where the right wheel should rotate faster. The right wheel will be the reference
      case RIGHT_FASTER_STATE:
      case LEFT_FASTER_STATE:
      case CIRC_TEST_R_STATE:
      case CIRC_TEST_L_STATE:
        // Take the reference of the faster wheel to compare distances
        if (STATE == RIGHT_FASTER_STATE || STATE == CIRC_TEST_R_STATE)
          reference = encoderDER;
        else
          reference = encoderIZQ;
        // Stop just a few pulses before because of the inertia
        if (reference < PULSES_NUM - INERTIA_LIMIT) {
          // Reduce speed in order to get objective pulses
          if (reference > PULSES_NUM - BREAK_PULSES) {
            breaking_period = 1;
            if (vell > 125)
              SPEED_INI_L = vell = 125;

            if (velr > 125)
              SPEED_INI_R = velr = 125;

            if (first_time == 1) {
              first_time = 0;
              // Write the speeds as a PWM duty cycle for each wheel
              analogWrite(MOT_R_PWM_PIN, velr);
              analogWrite(MOT_L_PWM_PIN, vell);
            }
          }
          if (millis() - time > PID_TIME) {
            time = millis();
            one_faster_dist();
            /*          dbg_counter++;
                      if (dbg_counter > 4)
                      {
                        dep1();
                        dbg_counter = 0;
                      }
            */
          }
        }
        else {
          stop_motors();
          // Make a delay in order to be sure that the wheels have stopped
          delay(1000);
          //        dep1();
          update_global_positions();
          STATE = RESET_STATE;
        }
        break;

      // Movement where both wheels should rotate the same distance
      case ROTATE_CCW_STATE:
      case ROTATE_CW_STATE:
      case MOVE_STRAIGHT_STATE:
    
        //      Serial.print("Encoder:  ");
        //      Serial.print(encoderDER);
        //      Serial.print("  Pulses: ");
        //      Serial.println(PULSES_NUM);
        if (encoderDER < PULSES_NUM - INERTIA_LIMIT)
        {
          // Reduce speed in order to get objective pulses
          if (encoderDER > PULSES_NUM - BREAK_PULSES)
          {
            breaking_period = 1;
            if (vell > 100)
              SPEED_INI_L  = 100;

            if (velr > 100)
              SPEED_INI_R  = 100;

            /*          if (first_time == 1)
                      {
                        first_time=0;
                        // Write the speeds as a PWM duty cycle for each wheel
                        analogWrite(MOT_R_PWM_PIN, velr);
                        if (vell == 0)
                        {
                          digitalWrite(MOT_L_A_PIN, LOW);
                          digitalWrite(MOT_L_B_PIN, LOW);
                          analogWrite(MOT_L_PWM_PIN, 255);
                        }
                        else
                        {
                          digitalWrite(MOT_L_A_PIN, HIGH);
                          digitalWrite(MOT_L_B_PIN, LOW);
                          analogWrite(MOT_L_PWM_PIN, vell);
                        }
                      }
            */
          }
          update_global_positions();

          if (millis()-msec_PID>100)
        {
        auxiliar_uns_int = SPEED_INI_L * 1.05;
        if (auxiliar_uns_int > 200)
          SPEED_INI_L = 200;
        else
          SPEED_INI_L = auxiliar_uns_int;
        SPEED_INI_R = SPEED_INI_L;
      msec_PID=millis();
          if (current_pulses = (unsigned int) (((encoderDER + encoderIZQ) / 2) + 0.4999) - last_pulses > PID_PULSES)
          {
            last_pulses = current_pulses;
            straigh_dist1();
            // delay(250);
            if (theta_max < abs(Theta))
              theta_max = abs(Theta);
            //          dep1();
          }
        }
        }
        else
        {
          //        Serial.println("Parar");
          stop_motors();
          //        dep1();
          // Make a delay in order to be sure that the wheels have stopped
          delay(1000);
          STATE = RESET_STATE;
          update_global_positions();
        }
        break;
      ////////////////////////////////////////
      //    Control por velocidad
      ////////////////////////////////////////
      case MOVE_DIF_SPEED:
        /*      auxiliar_uns_int=SPEED_INI_L*1.1;
              if (auxiliar_uns_int>SPEED_INI_L_LIM)
                SPEED_INI_L=SPEED_INI_L_LIM;
              else
                SPEED_INI_L=auxiliar_uns_int;
              SPEED_INI_R=SPEED_INI_L;
        */
    
        // PID for speeds update
        update_speeds(0);

        /*        if (vell==0 && velr==0)
                {
                  stop_motors();
                  delay(1000);
                  STATE = REST_STATE;
                }
        */        update_global_positions();
        break;
      ////////////////////////////////////////
      //    Control por velocidad
      ////////////////////////////////////////

      case PATROL_STATE:
        auxiliar_uns_int = SPEED_INI_L * 1.1;
        if (auxiliar_uns_int > SPEED_INI_L_LIM)
          SPEED_INI_L = SPEED_INI_L_LIM;
        else
          SPEED_INI_L = auxiliar_uns_int;
        SPEED_INI_R = SPEED_INI_L;
        //Serial.println(SPEED_INI_L);

        vigila_patrol = 1;
        if (patrol() == 0)
        {
          STATE = RESET_STATE;
          vigila_patrol = 0;
        }
        update_global_positions();
        break;
      default:
        break;
    }
  } // End of while(1)
} // End of loop
