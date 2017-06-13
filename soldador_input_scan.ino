/*      
 ____              _    __        __   _     _              ____            _             _ _           
/ ___| _ __   ___ | |_  \ \      / /__| | __| | ___ _ __   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __ 
\___ \| '_ \ / _ \| __|  \ \ /\ / / _ \ |/ _` |/ _ \ '__| | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
 ___) | |_) | (_) | |_    \ V  V /  __/ | (_| |  __/ |    | |__| (_) | | | | |_| | | (_) | | |  __/ |   
|____/| .__/ \___/ \__|    \_/\_/ \___|_|\__,_|\___|_|     \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|   
      |_|                                                                                              
      v-final                                            
 * Components used here: 4-digit 7-segment display, a 74hc595 shift-register IC to control the 7-segments of the display,
 * a 250V10A relay to control AC power to the spot welder transformer, a microwave keypad with its buttons 
 * charlieplexed/multiplexed/proprietaryplexed into arduino input pins. 
 * This "welder controller" has 2 power levels: first is a short-circuit of the AC line connecting the microwave transf.
 * primary coil directly to mains, which is very dangerous. This gives out about 7V on the secondary with max 200A. The
 * second power level is with BT151-800R triacs in series with a power resistor to halve that power. 
 * Also got a temp sensor to activate a fan when things get too hot; but this is unused for now since I ran out of 
 * output pins. 
 * I/O used: 0-8 keypad, 
 *           9 weld(power1) 
 *           10 weld(power2)
 *           11-13 74HC595 shiftregister (display segments), 
 *           A0-A3 display digits, 
 *           A4 - buzzer
 *           A5 - temperature sensor
 * Note that the shift-register is not used to control which DIGITS of the display are lit. That is controlled by 
 * arduino analog pins. IIRC my display is a common cathode so those analog pins go LOW when I want to light a 
 * certain digit. Pinout for the module is:
 * Segments on each digit: 
 *               A
 *              --- 
 *         f   |   |  b         
 *             g---             
 *         e   |   |  c     
 *              --- .         
 *               d    dp
 *       
 *       
 *       4-digit 7-segment module pintout: 6 pins top, 6 pins bottom. A-G are segments, D1-D4 are digit-enabling pins
 *      ===============================
 *      D1     A     F   D2   D3    B 
 *      -------------------------------
 *      12    11    10    9    8    7
 *      ===============================
 *       1     2     3    4    5    6
 *      -------------------------------
 *       E     D    dp    C    G    D4
 *      ===============================
 *      
 *      
 * The display segments' pins are connected to the 74HC595 output pins (Q0-7). I use a DIP IC with 16pins, the pinout is: 
 *     =============
 *   1-|Q1      VCC|-16           Q0-Q7    - output pins (connect to segment pins of the display)
 *   2-|Q2       Q0|-15           VCC, GND - obvious, 5V
 *   3-|Q3       DS|-14           DS       - data in
 *   4-|Q4       OE|-13           Q7'      - data out (if you daisy chain)
 *   5-|Q5    ST_CP|-12           ST_CP    - shift register latch
 *   6-|Q6    SH_CP|-11           SH_CP    - shift register clock
 *   7-|Q7       MR|-10           MR       - Master Reset (active low)
 *   8-|GND     Q7'|-09           OE       - Output Enable (active low)
 *     =============
 * 

 __        ___    ____  _   _ ___ _   _  ____ 
 \ \      / / \  |  _ \| \ | |_ _| \ | |/ ___|
  \ \ /\ / / _ \ | |_) |  \| || ||  \| | |  _   - Serial.print() needs pins 0 and 1
   \ V  V / ___ \|  _ <| |\  || || |\  | |_| |  - which are already in use here!!
    \_/\_/_/   \_\_| \_\_| \_|___|_| \_|\____|  - IT WONT WORK, DONT TRY!
                                             
*/

# include <SevenSegShift.h>

#define sinusMax_us   4583  // time required for a sine-wave at 60Hz to half-cycle once
                            // remember in europe it's 50Hz so this will change
                            
#define PIN_WELD1      9     
#define PIN_WELD2     10
#define PIN_BUZZ      A0
#define PIN_DATA      11
#define PIN_LATCH     12
#define PIN_SHIFT     13
#define PIN_TEMP      A5
const int           START            = 'S',
                    CANCEL           = 'X',
                    TEMP             = 'F',
                    DELAY            = 'D',
                    TIME             = 'T',
                    COOK             = 'O',
                    CLOCK            = 'C',
                    RECIPE           = 'R',
                    DEFROST          = 'E',
                    POWER            = 'P';
const int           pins[] = {0,       1,       2,       3,       4,       5,       6,       7,       8};
const int           keys[9][9] = { 
                 //          0        1        2        3        4        5        6        7        8
                 /*0*/    { 99,       5,      99,      99,      99,      99,       7,      99,      99},      
                 /*1*/    { 99,      99,      99,      99,      99,      99,      99,      99,      99},     
                 /*2*/    { 99,       1,      99,      99,      99,      99,       3,      99,      99},     
                 /*3*/    {  8,      99,       4,      99,      99,   POWER,      99,       0,    COOK},     
                 /*4*/    {  6,      99,       2,      99,      99, DEFROST,      99,   DELAY,    TIME},     
                 /*5*/    { 99,       9,      99,      99,      99,      99,  RECIPE,   POWER,      99},     
                 /*6*/    {  7,      99,       3,      99,      99,  RECIPE,      99,   CLOCK,    TEMP},     
                 /*7*/    { 99,   START,      99,      99,      99,      99,   CLOCK,      99,      99},     
                 /*8*/    { 99,  CANCEL,      99,      99,      99,      99,    TEMP,      99,      99}};
// s-start  p-power d-delay c-clock r-recipe o-cook e-defrost F-TEMP

const int             step_ms          = 125;

      int             preWeld_ms       = 250;
      int             weldPause_ms     = 500;
      int             weldTime_ms      = 375;
      int             powerLevel       = 1;
      
      int             temp_preWeld_ms       = 0;
      int             temp_weldPause_ms     = 0;
      int             temp_weldTime_ms      = 0;
      int             temp_powerLevel       = 1;
      
      int             maxpreWeld_ms    = 2500;
      int             maxweldPause_ms  = 5000;
      int             maxweldTime_ms   = 10000;
      int             maxpowerLevel    = 2;
      
      bool            continuously     = true;
      bool            onoff            = false;
      String          tempVal          = "" ;
      int             cfg_operation    = 0 ;

      byte            leds = 0;
const int             _numOfDigits = 4;
      int             _digitPins [ _numOfDigits ]={ A1, A2, A3, A4 };

      SevenSegShift   _disp (PIN_SHIFT, PIN_LATCH, PIN_DATA, MSBFIRST);

void setup() 
{
  delay(20000);
  for (int k=0; k<11; k++)
  {
    pinMode(pins[k], INPUT_PULLUP);
    digitalWrite(pins[k], HIGH);
  }
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_SHIFT, OUTPUT);  
  pinMode(PIN_DATA, OUTPUT);
  //Serial.begin(57600);
  //Serial.println("Welder bootup");
  
  for (int j = 0; j < 4; j++) 
  {
        tone(PIN_BUZZ, 3000, 100);
        delay(300); // prevent detecting too many key presses when i hold a key
        noTone(PIN_BUZZ);
  }
  _disp.setDigitPins ( _numOfDigits , _digitPins );
  _disp.setCommonCathode();
  _disp.setDigitDelay (2000) ;
  
  _disp.setTimer(2);
  _disp.startTimer();
  for ( int test = 0; test < 20; test++)
  {
    _disp.write(test);
    delay(500);
    _disp.write("    ");
  }
  //Serial.println("Started scanning...");
  //delay(1000);
}

void performCancellation()
{
  cfg_operation = 0;
  tempVal = "";
  _disp.write("    ");
}

void loop() 
{  
  for (int k=0; k<9; k++)
  {
    for (int j=0; j<9; j++)
    {
      if (j == k) continue;
      
      /*if (pins[k] == 3 && pins[j] == 6) continue; //these are always connected :-/ 
      if (pins[k] == 3 && pins[j] == 5) continue; //or multiple keys activate these combos
      if (pins[k] == 3 && pins[j] == 7) continue; //strange keypad
      if (pins[k] == 3 && pins[j] == 4) continue;*/

      
      pinMode(pins[k], OUTPUT);
      digitalWrite(pins[k], LOW);

  
      if ( !digitalRead(pins[j]) && keys[k][j]!=99) 
      {
        /*Serial.print("Got pin ");
        Serial.print(pins[k]);
        Serial.print(" ");
        Serial.print(pins[j]);
        Serial.print(" = ");
        Serial.println(keys[k][j]);
        delay(100);*/
        tone(PIN_BUZZ, 3000, 100);
        delay(300); // prevent detecting too many key presses when i hold a key
        noTone(PIN_BUZZ);
        
        if (cfg_operation == 0 && keys[k][j] == START) 
        { 
          weldCyclus();
          //cfg_operation = START;
          //Serial.println("WELDING");
        }
        // this condition is the equivalent of OK
        if (cfg_operation != 0 && keys[k][j] == START) 
        { 
          //Serial.println("OK");
          if ( cfg_operation == DELAY ) {
            /*Serial.print("weldpause:");
            Serial.print(weldPause_ms);
            Serial.println(" -> "+tempVal);*/
            weldPause_ms  = tempVal.toInt();
          }
          if ( cfg_operation == TIME  ) {
            /*Serial.print("weldTime:");
            Serial.print(weldTime_ms);
            Serial.println(" -> "+tempVal);*/
            weldTime_ms = tempVal.toInt();
          }
          if ( cfg_operation == COOK  ) {
            /*Serial.print("preWeld:");
            Serial.print(preWeld_ms);
            Serial.println(" -> "+tempVal);*/
            preWeld_ms = tempVal.toInt();
          }
          _disp.write(tempVal);
          performCancellation();
        }
        if ( keys[k][j] == CANCEL ) {
          //Serial.println("CANCEL");
          performCancellation();
        }

        
        if (keys[k][j] == DELAY) // weldPause_ms
        {
          //Serial.print("DELAY ");
          if ( cfg_operation == DELAY )
          {
            temp_weldPause_ms += step_ms;
            if (temp_weldPause_ms > maxweldPause_ms) 
              temp_weldPause_ms = step_ms; 
          }
          if ( cfg_operation == 0 )
          {
            cfg_operation = DELAY;
            temp_weldPause_ms = weldPause_ms;
          }
          //Serial.println(temp_weldPause_ms);
          _disp.write(temp_weldPause_ms);
        }

        
        if (keys[k][j] == TIME) // weldTime_ms
        {
          //Serial.print("TIME ");
          if ( cfg_operation == TIME )
          {
            temp_weldTime_ms += step_ms;
            if (temp_weldTime_ms > maxweldTime_ms) 
              temp_weldTime_ms = step_ms; 
          }
          if ( cfg_operation == 0 )
          {
            cfg_operation = TIME;
            temp_weldTime_ms = weldTime_ms;
          }
          //Serial.println(temp_weldTime_ms);
          _disp.write(temp_weldTime_ms);
        }

        
        if (keys[k][j] == COOK) 
        {
          //Serial.print("COOK ");
          if ( cfg_operation == COOK )
          {
            temp_preWeld_ms += step_ms;
            if (temp_preWeld_ms > maxpreWeld_ms) 
              temp_preWeld_ms = step_ms; 
          }
          if ( cfg_operation == 0 )
          {
            cfg_operation = COOK;
            temp_preWeld_ms = preWeld_ms;
          }
          //Serial.println(temp_preWeld_ms);
          _disp.write(temp_preWeld_ms);
        }

        
        if (keys[k][j] == POWER) 
        {
          //Serial.print("PWR");
          if ( cfg_operation == POWER )
          {
            temp_powerLevel += powerLevel;      // this section is a little bit special
            if (temp_powerLevel > maxpowerLevel)// because we only have 2 power levels.
              temp_powerLevel = powerLevel;     // dunno yet how this is controlled 
          }                                     // during a weld cycle. It related to
          if ( cfg_operation == 0 )             // activating the RELAY _OR_ the opto.
          {                                     // must make sure the opto is now with
            cfg_operation = POWER;              // with a curr limiting resistor so I
            temp_powerLevel = powerLevel;       // dont blow up my BTR like in winter
          }
          //Serial.println(temp_powerLevel);
          _disp.write(temp_powerLevel);
        }
        
        if (cfg_operation == 0 && keys[k][j] == CLOCK) 
        {
          cfg_operation = CLOCK;
          //Serial.println("CLOCK");
        }
        
        if (cfg_operation == 0 && keys[k][j] == TEMP) 
        {
          //cfg_operation = TEMP; this is not actually an operation, just display
          
          int value = analogRead(PIN_TEMP);
          float millivolts = (value / 1024.0) * 5000;
          float celsius = millivolts / 10;  // sensor output is 10mV per degree Celsius
          // Fahrenheit = (celsius * 9)/ 5 + 32 
          _disp.write(celsius);
          //Serial.println("TEMP");
        }
        
        if (cfg_operation == 0 && keys[k][j] == RECIPE) 
        {
          //cfg_operation = RECIPE;
          //TODO: switch through different recipes: allum, steel, inox, mag, etc
          //Serial.println("RECIPE");
        }
        
        if (cfg_operation == 0 && keys[k][j] == DEFROST) 
        {
          //cfg_operation = DEFROST;
          //Serial.println("DEFROST");
        }
        
        if ( cfg_operation != 0 ) {
        switch (keys[k][j])
        {
          case 0: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 1: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 2: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 3: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 4: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 5: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 6: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 7: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 8: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
          case 9: tempVal += String(keys[k][j]); _disp.write(tempVal); break;
        }
        }
      }
    }
    pinMode(pins[k], INPUT_PULLUP);
  }
  
}

void weldCyclus()
{ 
  for (int k = 1; k < 4; k++) {
    analogWrite(PIN_BUZZ, k*125);
    delay(1000);
    analogWrite(PIN_BUZZ, 0);
    delay(1000);
  }
  sinusMax();
  pulseWeld(preWeld_ms);
  delay(weldPause_ms);
  sinusMax();
  pulseWeld(weldTime_ms);
}


// Performs an intervalled operation of welding 
void pulseWeld(int ms)
{
  weld(1);
  delay(ms);
  weld(0);
}

// Connect welder to Mains (turn it on [actually welds])
void weld(bool b)
{
  digitalWrite(PIN_WELD1, b);
  digitalWrite(PIN_WELD2, b);
}

// Waits for a zerocrossing of AC line
void sinusMax()
{
  /*while(digitalRead(zeroCrossPin));
  while(!digitalRead(zeroCrossPin));*/
  delayMicroseconds(sinusMax_us); // to prevent inrush current, turn-on at the sinus max
}

ISR(TIMER2_COMPA_vect) 
{
  _disp.interruptAction();
}

