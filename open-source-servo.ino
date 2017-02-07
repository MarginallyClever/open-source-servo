//--------------------------------------------------
// using a sensor to move a stepper, effectively making a servo
// dan@marginallyclever.com 2017-02-4
// Arduino UNO, a4988 driver, as5045 sensor
//--------------------------------------------------

//--------------------------------------------------
// CONSTANTS
//--------------------------------------------------

// AS5045 sensor bits, flags, and masks
#define SENSOR_PARITY_TEST_ON
#define SENSOR_TOTAL_BITS    (18)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (12)
#define SENSOR_ERROR_BITS    (5)
#define SENSOR_PARITY_BITS   (1)
#define SENSOR_STATUS_BITS   (SENSOR_ERROR_BITS+SENSOR_PARITY_BITS)
#define SENSOR_ANGLE_MASK    (0b111111111111000000)
#define SENSOR_ERROR_MASK    (0b000000000000111110)
#define SENSOR_PARITY_MASK   (0b000000000000000001)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)(1<<SENSOR_ANGLE_BITS))


//define this flag to set the chip into alignment mode, which indicates the how far the magnet is off-axis while turning.
//#define ALIGNMENT_MODE

// sensor pins
#define PIN_SENSOR_PGM    5
#define PIN_SENSOR_CSEL   4
#define PIN_SENSOR_CLK    3
#define PIN_SENSOR_SDOUT  2

// stepper defitions
#define STEPS_PER_TURN    400.0
#define MICROSTEPS        16.0
#define STEPS_TOTAL       (STEPS_PER_TURN * MICROSTEPS)
#define DEGREES_PER_STEP  (360.0/STEPS_TOTAL)

#define ERROR_TERM        2 // SENSOR_ANGLE_PER_BIT/DEGREES_PER_STEP = 0.9/0.6 = 1.5
#define PUSH_DELAY        50 // ms
// stepper pins
#define PIN_ENABLE        8
#define PIN_STEP          7
#define PIN_DIR           6

//--------------------------------------------------
// GLOBALS
//--------------------------------------------------

float previousAngle;
uint32_t previousD = 0;
char hasStepped;
int numberOfStepsTaken;
int pushDelay;
long previousTime;

//--------------------------------------------------
// METHODS
//--------------------------------------------------

void setup() {
  // open serial
  Serial.begin(57600);
  Serial.println(F("\n\nHELLO"));
  // setup sensor
  pinMode(PIN_SENSOR_CLK, OUTPUT);
  pinMode(PIN_SENSOR_CSEL, OUTPUT);
  pinMode(PIN_SENSOR_SDOUT, INPUT);
#ifdef ALIGNMENT_MODE
  pinMode(PIN_SENSOR_PGM, INPUT);
#endif
  previousAngle = -1;

  // setup stepper
  pinMode(PIN_ENABLE,OUTPUT);
  pinMode(PIN_DIR,OUTPUT);
  pinMode(PIN_STEP,OUTPUT);
  hasStepped=false;

  Serial.print("deg per step = ");  Serial.println(DEGREES_PER_STEP);
  Serial.print("deg per sense= ");  Serial.println(SENSOR_ANGLE_PER_BIT);
  delay(1000);
  
  // read starting sensor position
  uint32_t d;
  do {
    d = sensor_update(PIN_SENSOR_CSEL, PIN_SENSOR_SDOUT);
  } while(sensor_error(d, 1));
  
  float previousAngle = sensor_angle(d);

  numberOfStepsTaken=0;

  Serial.println(F("\n\nSTART"));
  previousTime=millis();
}



void loop() {
  int beVerbose=0;
  char hasMoved = false;
  float angleChange=0;

  // part 1 - sense the world
  
  long currentTime = millis();
  long dt = currentTime - previousTime;
  Serial.print(currentTime);
  Serial.print('\t');
  
#ifdef ALIGNMENT_MODE
  float bottom = 1 << (SENSOR_ANGLE_BITS + 1), top = 0;
  uint32_t d = sensor_alignment(PIN_SENSOR_CSEL, PIN_SENSOR_SDOUT);
#else
  uint32_t d = sensor_update(PIN_SENSOR_CSEL, PIN_SENSOR_SDOUT);
#endif
  if(previousD!=d)
  {
    // sensor reading has changed
    previousD = d;
    if (!sensor_error(d, beVerbose))
    {
      // no error
      float currentAngle = sensor_angle(d);

      if(previousAngle != currentAngle)
      {
        // sensor has detected a a change in position
        hasMoved = true;
        angleChange = currentAngle - previousAngle;
        previousAngle = currentAngle;
        if(beVerbose) Serial.print(' ');
        //if(beVerbose) Serial.print(currentAngle);
        //Serial.print('\t');
        //Serial.print(analogRead(A4));
        //Serial.print('\t');
        //Serial.print(analogRead(A5));
#ifdef ALIGNMENT_MODE
        if ( bottom > currentAngle ) bottom = currentAngle;
        if ( top < currentAngle ) top = currentAngle;
        if(beVerbose) Serial.print('\t');
        if(beVerbose) Serial.print(top);
        if(beVerbose) Serial.print('\t');
        if(beVerbose) Serial.print(bottom);

#endif
      }
    }
  }

  // part 2 - decide how to react
  Serial.print(" ");
  
  if(hasMoved) {
    Serial.print("m");
    if(hasStepped) {
      Serial.print("s");
      // has stepped and moved, no problem.
      numberOfStepsTaken=0;
    } else {
      Serial.print("_");
      if(angleChange>SENSOR_ANGLE_PER_BIT*3 && pushDelay==0) {
        // has not stepped and has moved.  definite interference (outside force? gravity?)
        outsideForceDetected();
      }
    }
  } else {
    Serial.print("_");
    if(hasStepped) {
      Serial.print("s");
      // has stepped and not moved.  step size < sensor range, so allow a few steps.
      if(numberOfStepsTaken>ERROR_TERM*2 && pushDelay==0) {
        // definite interference (outside force? gravity?)
        outsideForceDetected();
      } else {
        // not sure yet.
      }
    } else {
      // has not stepped and not moved - ok!
      Serial.print("_");
    }
  }

  // part 3 - act
  
  float target = 180;
  float margin = DEGREES_PER_STEP*3;
  if(pushDelay>0) {
    // we've been pushed, don't fight back for a moment.
    Serial.print("d");
    pushDelay -= dt;
    if(pushDelay<0) pushDelay=0;
  } else {
    if(previousAngle>target+margin) {
      Serial.print("-");
      moveStepper(-1);
    } else if(previousAngle<target-margin) {
      Serial.print("+");
      moveStepper(1);
    } else {
      Serial.print("d");
      stepperDisable();
      hasStepped=false;
    }
  }
  
  float diff = target - previousAngle;
  Serial.print(' ');
  Serial.print(diff);
  Serial.print('\t');
  Serial.print(numberOfStepsTaken);
  
  Serial.print('\n');

  
  //delay(1);
  previousTime = currentTime;
}


void outsideForceDetected() {
  pushDelay=PUSH_DELAY;
  stepperDisable();
  hasStepped=false;
  numberOfStepsTaken=0;
}

void stepperDisable() {
  digitalWrite(PIN_ENABLE,HIGH);
}


void stepperEnable() {
  digitalWrite(PIN_ENABLE,LOW);
}


void moveStepper(int dir) {
  stepperEnable();
  digitalWrite(PIN_DIR,(dir>0) ? HIGH:LOW);
  digitalWrite(PIN_STEP,LOW);
  digitalWrite(PIN_STEP,HIGH);
  hasStepped=true;
  numberOfStepsTaken += 1;
}


#ifdef ALIGNMENT_MODE
uint32_t sensor_alignment(int csel, int sdout) {
  uint32_t data = 0, inputStream;
  int x;

  // Sensor sends data when CLK goes high.
  // To choose a board, set the CSEL pin high, then tick the clock.
  digitalWrite(csel, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_SENSOR_PGM, HIGH);
  // We won't need CSEL again until the next sample, so set it low
  digitalWrite(csel, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SENSOR_PGM, LOW);


  digitalWrite(PIN_SENSOR_CLK, HIGH);
  delayMicroseconds(5);
  // Set the clock low.  On the next high sensor will start to deliver data.
  digitalWrite(PIN_SENSOR_CLK, LOW);

  for (x = 0; x < SENSOR_TOTAL_BITS; x++) {
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    // one bit of data is now waiting on sensor pin
    inputStream = digitalRead(sdout);
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
    digitalWrite(PIN_SENSOR_CLK, LOW);
  }
  return data;
}
#endif



// from http://www.madscientisthut.com/forum_php/viewtopic.php?f=11&t=7
uint32_t sensor_update(int csel, int sdout) {
  uint32_t data = 0, inputStream;
  int x;

  // Sensor sends data when CLK goes high.
  // To choose a board, set the CSEL pin high, then tick the clock.
  digitalWrite(csel, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  // We won't need CSEL again until the next sample, so set it low
  digitalWrite(csel, LOW);
  // Set the clock low.  On the next high sensor will start to deliver data.
  digitalWrite(PIN_SENSOR_CLK, LOW);

  for (x = 0; x < SENSOR_TOTAL_BITS; x++) {
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    // one bit of data is now waiting on sensor pin
    inputStream = digitalRead(sdout);
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
    digitalWrite(PIN_SENSOR_CLK, LOW);
  }
  return data;
}




/**
   @input data the raw sensor reading
   @return the angle in degrees
*/
float sensor_angle(uint32_t data) {
  uint32_t angle = data >> SENSOR_STATUS_BITS; // shift 18-digit angle right 6 digits to form 12-digit value
  angle &= SENSOR_ANGLE_MASK >> SENSOR_STATUS_BITS;  // mask the 18 bits that form the angle
#ifdef ALIGNMENT_MODE
  return angle;
#else
  return (angle * SENSOR_ANGLE_PER_BIT);
#endif
}




/**
   @input data the raw sensor reading
   @return the angle in degrees
*/
int sensor_error(uint32_t data, int beVerbose) {
#ifdef SENSOR_PARITY_TEST_ON
  // Parity test
  char parity = data & SENSOR_PARITY_MASK;
  int parity_test = 0;
  int x;
  uint32_t v = data >> SENSOR_PARITY_BITS;
  for (x = 0; x < (SENSOR_TOTAL_BITS - SENSOR_PARITY_BITS); ++x) {
    parity_test += (v & 0x1);
    v >>= 1;
  }
  if ( (parity_test & 0x1) != parity ) {
    if (beVerbose) {
      Serial.print("P");/*
      Serial.print(parity_test & 0x1, DEC);
      Serial.print('v');
      Serial.print(parity & 0x1, DEC);*/
      Serial.print(' ');
    }
    //return 1;
  } else {
    if (beVerbose) {
      Serial.print("- ");
    }
  }
#endif

  // status tests
  int statusBits = data & SENSOR_ERROR_MASK;
  char DECn = statusBits & 2; // goes high if magnet moved away from IC
  char INCn = statusBits & 4; // goes high if magnet moved towards IC
  char LIN = statusBits & 8; // goes high for linearity alarm
  char COF = statusBits & 16; // goes high for cordic overflow: data invalid
  char OCF = 32 - (statusBits & 32); // this is 1 when the chip startup is finished.

  if (DECn && INCn) {
    if (beVerbose) Serial.print('x'); //Serial.println("magnet moved out of range");
    //return (4 | 2);
  } else if (DECn) {
    if (beVerbose) Serial.print('A'); //Serial.println("magnet moved away from chip");
    //return 4;
  } else if (INCn) {
    if (beVerbose) Serial.print('T'); //Serial.println("magnet moved towards chip");
    //return 2;
  } else if (beVerbose) Serial.print('-');
  if (beVerbose) Serial.print(' ');

  if (LIN) {
    if (beVerbose) Serial.print('L'); //Serial.println("linearity alarm: magnet misaligned? Data questionable.");
    //    return 8;
  } else if (beVerbose) Serial.print('-');
  if (beVerbose) Serial.print(' ');

  if (COF) {
    if (beVerbose) Serial.print('C'); //Serial.println("cordic overflow: magnet misaligned? Data invalid.");
    //    return 16;
  } else if (beVerbose) Serial.print('-');
  if (beVerbose) Serial.print(' ');

  if (OCF) {
    if (beVerbose) Serial.print('O'); //Serial.println("Sensor not ready.");
    //    return 32;
  } else if (beVerbose) Serial.print('-');
  if (beVerbose) Serial.print(' ');

  return 0;
}

