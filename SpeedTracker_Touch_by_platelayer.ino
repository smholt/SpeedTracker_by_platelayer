/* SpeedTracker_Touch_by_platelayer.ino by Svein-Martin Holt, mars 2024   http://www.platelayer.com

 Model Railroad Speed detector, using 2 IR-sensors to measure the time and a Touch Screen Shield for the Arduino Uno.
 The scale and speed units are selected on the TouchScreen scrolling through the scales and speed units
 New scale or speed unit are selected by pressing the TouchScreen.
 The last selected values are stored in EEPROM so they will be the default starting values.

 More info can be found on my homepage: http://www.platelayer.com/arduino-scale-speedometer.aspx

 The software can be found here: https://github.com/smholt/SpeedTracker_by_platelayer

 The 3D-model of the digits, can be found on my cults3d-page: https://cults3d.com/en/users/smholt
*/

// The software is based on an idea by Ruud Boar
// See his project homepage:
// https://rudysmodelrailway.wordpress.com/2018/03/29/a-e-6-model-train-speed-measurement-device-part-2-the-arduino-software/30
// Original version August 2019 by Ruud Boer
// Revised version September 2021

// Two optical sensors are used, spaced SENSOR_DISTANCE [um] apart
// [us] timer starts when one of the sensor inputs goes HIGH
// [us] timer stops when the other sensor goes HIGH
// m_per_s = distance / (stop_time - start_time)
// km_per_hr = 3.6 * m_per_s, mph = 2.23694 * m_per_s;

// Great resource for the LCD Touch screen
//  http://www.lcdwiki.com/2.4inch_Arduino_Display#Program_Download

#define prog_ver "v1.0, 2024.03.03"  // Program version and date, displayed at startup

#include <Adafruit_GFX.h>  // Adafruit Graphic library
#include <MCUFRIEND_kbv.h>

#include <Fonts/FreeSans18pt7b.h>

const char* compiledOn = __DATE__ ", " __TIME__;  // Get the compiled date and time

MCUFRIEND_kbv tft;
#include <TouchScreen.h>
// Define touchscreen pressure points
#define MINPRESSURE 200
#define MAXPRESSURE 1000

#include <EEPROM.h>  // EEPROM library

#define SENSOR_DISTANCE 100000  // [um] measured distance between the two IR beams
#define INIT_TRAIN_LENGTH 20    // [cm] New measurement only starts when train fully passed \
                                // Loco Length can be changed via keyboard input
#define SENSOR_L_PIN 11         // The left sensor is connected to digital pin 6
#define SENSOR_R_PIN 12         // The left sensor is connected to digital pin 7
//#define BUTTON_PIN 8            // The pushbutton is connected to digital pin 8
#define POWER_PIN A5  // Pin to measure voltage on Vin from battery

#define LED_PIN 13                // The ready LED is connected to digital pin 13
#define DEBOUNCE_DELAY 50         // Debounce delay 50 ms
#define LONG_PRESS_DELAY 1000     // Long press time 1 sec
#define SCALE_EEPROM_ADDR 0       // Saved scale to be used at startup
#define SPEED_UNIT_EEPROM_ADDR 1  // Saved speed unit to be used at startup
#define ROTATE_EEPROM_ADDR 2      // Saved rotation of display, 0=0 degrees, 1=180 degrees

/******************* UI details */
#define BUTTON_X 63
#define BUTTON_Y 250
#define BUTTON_W 100
#define BUTTON_H 70
#define BUTTON_SPACING_X 15
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// Color definitions
#define ILI9341_BLACK 0x0000       /*   0,   0,   0 */
#define ILI9341_NAVY 0x000F        /*   0,   0, 128 */
#define ILI9341_DARKGREEN 0x03E0   /*   0, 128,   0 */
#define ILI9341_DARKCYAN 0x03EF    /*   0, 128, 128 */
#define ILI9341_MAROON 0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE 0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE 0x7BE0       /* 128, 128,   0 */
#define ILI9341_LIGHTGREY 0xC618   /* 192, 192, 192 */
#define ILI9341_DARKGREY 0x7BEF    /* 128, 128, 128 */
#define ILI9341_BLUE 0x001F        /*   0,   0, 255 */
#define ILI9341_GREEN 0x07E0       /*   0, 255,   0 */
#define ILI9341_CYAN 0x07FF        /*   0, 255, 255 */
#define ILI9341_RED 0xF800         /* 255,   0,   0 */
#define ILI9341_MAGENTA 0xF81F     /* 255,   0, 255 */
#define ILI9341_YELLOW 0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE 0xFFFF       /* 255, 255, 255 */
#define ILI9341_ORANGE 0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5 /* 173, 255,  47 */
#define ILI9341_PINK 0xF81F


// Define pins for resistive touchscreen
// ALL Touch panels and wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
const int XP = 9, XM = A3, YP = A2, YM = 8;  //240x320 ID=0x9341
const int TS_LEFT = 98, TS_RT = 927, TS_TOP = 78, TS_BOT = 895;

// Define object for touchscreen
// Last parameter is X-Y resistance, measure or use 300 if unsure
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Define button array object
Adafruit_GFX_Button buttons[6];

// Define arrays with button text and colors
char buttonlabels[2][12] = { "SPEED", "SCALE" };
uint16_t buttoncolors[2] = { RED, BLUE };

byte l_r, state;
byte scale, scale_old, scale_local, rotate;
byte units, units_old = 99;  // 0=MPH, 1=km/hr, 99=startup dummy
int train_length, serialread;
unsigned long start_us, stop_us, measured_us, waittime;
float m_per_s;
float km_per_hr;
float mi_per_hr;

const int numValues = 6;
const int values[] = { 87, 45, 76, 120, 160, 220 };  // Different scales to scroll.

int currentValueIndex = 0;
int selectedValue = 0;
unsigned long buttonPressStartTime = 0;

// Define the pins for the IR sensors
const int sensor1Pin = 11;
const int sensor2Pin = 12;

// Variables to store the state of the sensors
bool sensor1Triggered = false;
bool sensor2Triggered = false;

// Variables to store the time of trigger
unsigned long sensor1Time = 0;
unsigned long sensor2Time = 0;

// Variable to store the start time for reset mechanism
unsigned long startTime = 0;
unsigned long readyMeasureTime = 5000;

int batteryLevel = 20;

int x_offset = 3;  //x_offset on screen of box hole need adjustement
int y_offset = 0;  //y_offset on screen of box hole need adjustement

void (*resetFunc)(void) = 0;  //declare reset function @ address 0

//---start setup----------------------------------------------------------------------------
void setup() {
  pinMode(SENSOR_L_PIN, INPUT_PULLUP);
  pinMode(SENSOR_R_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  delay(1000);
  uint16_t ID = tft.readID();
  Serial.print("TFT ID = 0x");
  Serial.println(ID, HEX);
  //Serial.println("Calibrate for your Touch Panel");
  if (ID == 0xD3D3) ID = 0x9486;  // write-only shield
  tft.begin(ID);
  tft.setRotation(2);  //PORTRAIT
  tft.fillScreen(BLACK);

  train_length = INIT_TRAIN_LENGTH;  // [cm]
  Serial.println();
  Serial.print("Speed Measurement ");
  Serial.println(prog_ver);
  Serial.print("Compiled on: ");
  Serial.println(compiledOn);
  Serial.println();
  Serial.print(F("Train length is set to "));
  Serial.print(train_length);
  Serial.println(F(" cm "));
  Serial.println(F("For longer trains change this by typing"));
  Serial.println(F("in the input field above and hit ENTER"));
  Serial.println();

  // Read saved scale and speed unit from EEPROM
  scale = EEPROM.read(SCALE_EEPROM_ADDR);
  Serial.println(scale);
  if (scale <= 0 or scale > 220) {  // reset if invalid value
    scale = 220;
    EEPROM.write(SCALE_EEPROM_ADDR, scale);  // Save the new scale to EEPROM
    Serial.print("scale reset: ");
    Serial.println(scale);
  }

  for (int currentValueIndex = 0; currentValueIndex <= 5; currentValueIndex++) {
    Serial.println(currentValueIndex);
    Serial.println(values[currentValueIndex]);
    if (values[currentValueIndex] == scale) {
      selectedValue = currentValueIndex;
      break;
    }
  }
  Serial.print("selectedValue: ");
  Serial.println(selectedValue);

  units = EEPROM.read(SPEED_UNIT_EEPROM_ADDR);

  read_write_scale();

  write_to_display();

  tft.fillRect(x_offset + 11, y_offset + 69, 10, 12, WHITE);
  tft.fillRect(x_offset + 220, y_offset + 69, 10, 12, WHITE);


  draw_button();

  write_waiting_for_train();
}
//---end setup----------------------------------------------------------------------------

void draw_button(void) {
  // Draw buttons
  uint8_t row = 0;
  //  for (uint8_t row = 0; row < 1; row++) {
  for (uint8_t col = 0; col < 2; col++) {
    buttons[col + row * 3].initButton(&tft, BUTTON_X + col * (BUTTON_W + BUTTON_SPACING_X),
                                      BUTTON_Y + row * (BUTTON_H + BUTTON_SPACING_Y),  // x, y, w, h, outline, fill, text
                                      BUTTON_W, BUTTON_H, WHITE, buttoncolors[col + row * 3], WHITE,
                                      buttonlabels[col + row * 3], BUTTON_TEXTSIZE);
    buttons[col + row * 3].drawButton();
  }
  // }
}

void check_button(void) {

  // digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  // digitalWrite(13, LOW);

  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // There is a minimum pressure that is consider valid
  // Pressure of 0 means no pressing

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {

    p.x = p.x + p.y;
    p.y = p.x - p.y;
    p.x = p.x - p.y;

    p.x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());  //.kbv makes sense to me
    p.y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
  }

  // Go thru all the buttons, checking if they were pressed
  for (uint8_t b = 0; b < 6; b++) {
    if ((buttons[b].contains(p.x, p.y)) && p.x > 10) {
      /*    Serial.print("Pressing: ");
      Serial.println(b);
      */
      buttons[b].press(true);  // tell the button it is pressed

      //Button has been pressed
      if (b == 0) {
        // Toggle Speed unit
        units = !units;
        write_to_display();
        EEPROM.write(SPEED_UNIT_EEPROM_ADDR, units);  // Save the new speed unit to EEPROM
      }
      if (b == 1) {
        // Scroll Scale
        scale = values[currentValueIndex];  // set current scale from array
        //write_to_display();
        EEPROM.write(SCALE_EEPROM_ADDR, scale);  // Save the new scale to EEPROM
        currentValueIndex++;
        if (currentValueIndex > 5) { currentValueIndex = 0; }  // reset currentValueIndex
      }

    } else {
      buttons[b].press(false);  // tell the button it is NOT pressed
    }
  }

  // now we can ask the buttons if their state has changed
  for (uint8_t b = 0; b < 6; b++) {
    if (buttons[b].justReleased()) {
      Serial.print("Released: ");
      Serial.println(b);
      buttons[b].drawButton();  // draw normal
    }

    if (buttons[b].justPressed()) {
      buttons[b].drawButton(true);  // draw invert!

      delay(100);  // UI debouncing
    }
  }
}

//---start loop--------------------------------------------------------------------
void loop() {

  read_write_scale();

  check_button();

  check_sensors();

  write_sensor_indicator();
}
//---end loop--------------------------------------------------------------------

void check_sensors2() {

  // Check if sensors are triggered by going LOW and if not already triggered
  if (digitalRead(SENSOR_L_PIN) == LOW && !sensor1Triggered) {
    if (!sensor2Triggered) {
      tft.fillRect(x_offset + 0, y_offset + 90, 240, 34, BLACK);
      tft.setCursor(x_offset + 20, y_offset + 112);
      tft.setTextSize(1);
      tft.setFont(&FreeSans18pt7b);
      tft.print("L >>>>>> R");
      tft.setFont();
    }
    sensor1Triggered = true;
    sensor1Time = millis();
    Serial.print("Sensor 1 triggered at: ");
    Serial.println(sensor1Time);
  }

  if (digitalRead(SENSOR_R_PIN) == LOW && !sensor2Triggered) {
    if (!sensor1Triggered) {
      tft.fillRect(x_offset + 0, y_offset + 90, 240, 34, BLACK);
      tft.setCursor(x_offset + 20, y_offset + 112);
      tft.setTextSize(1);
      tft.setFont(&FreeSans18pt7b);
      tft.print("R <<<<<< L");
      tft.setFont();
    }
    sensor2Triggered = true;
    sensor2Time = millis();
    Serial.print("Sensor 2 triggered at: ");
    Serial.println(sensor2Time);
  }

  // If both sensors have been triggered, calculate time difference and print
  if (sensor1Triggered && sensor2Triggered) {
    Serial.print("Time between triggers: ");
    if (sensor1Time > sensor2Time) {
      Serial.print(abs(sensor1Time - sensor2Time) / 1000.0);
      measured_us = abs(sensor1Time - sensor2Time - 10UL) * 1000;
    } else {
      Serial.print(abs(sensor2Time - sensor1Time) / 1000.0);
      measured_us = abs(sensor2Time - sensor1Time - 10UL) * 1000;
    }

    Serial.println(" seconds");
    Serial.println(measured_us);
    //  measured_us = sensor1Time - sensor2Time - 10UL;  // -10 to compensate for code delay
    //       measured_us = stop_us - start_us - 10UL;  // -10 to compensate for code delay
    // m_per_s = float(scale) * float(SENSOR_DISTANCE) / float(measured_us);
    m_per_s = float(SENSOR_DISTANCE) / float(measured_us);
    km_per_hr = 3.6 * m_per_s;
    mi_per_hr = 2.23694 * m_per_s;
    write_to_display();

    if (!units) {
      Serial.print(mi_per_hr);
      Serial.println(F(" MPH"));
    } else {
      Serial.print(km_per_hr);
      Serial.println(F(" km/hr"));
    }
    Serial.println();

    draw_button();

    // Determine direction
    if (sensor1Time < sensor2Time) {
      Serial.println("Direction: 1 -> 2");
    } else {
      Serial.println("Direction: 2 -> 1");
    }

    // Reset after printing
    sensor1Triggered = false;
    sensor2Triggered = false;
    startTime = millis();  // Reset the start time for the next measurement
  }

  // Reset mechanism after 10 seconds
  if (millis() - startTime > readyMeasureTime) {
    sensor1Triggered = false;
    sensor2Triggered = false;
    sensor1Time = 0;
    sensor2Time = 0;
    startTime = millis();  // Reset start time for a fresh start
    Serial.println("Ready for train...");
    write_waiting_for_train();
    //  state = 0;
    draw_button();
  }
}

void check_sensors() {

  switch (state) {
    case 0:  // initial state, ready to start measuring
      if (!digitalRead(SENSOR_L_PIN)) {
        start_us = micros();
        l_r = 0;
        state = 1;
      }
      if (!digitalRead(SENSOR_R_PIN)) {
        start_us = micros();
        l_r = 1;
        state = 1;
      }
      break;

    case 1:  // wait for the other sensor to be triggered
      tft.fillRect(x_offset + 0, y_offset + 90, 240, 34, BLACK);
      tft.setCursor(x_offset + 20, y_offset + 112);
      tft.setTextSize(1);
      tft.setFont(&FreeSans18pt7b);
      if (!l_r) {
        Serial.println(F("L >>> R"));
        tft.print("L >>>>>> R");
        while (digitalRead(SENSOR_R_PIN)) {}  // loop here until sensor R is triggered
      } else {
        Serial.println(F("L <<< R"));
        tft.print("L <<<<<< R");
        while (digitalRead(SENSOR_L_PIN)) {}  // loop here until sensor L is triggered
      }
      tft.setFont();
      stop_us = micros();
      state = 2;
      break;

    case 2:                                     // calculate and show speed values
      measured_us = stop_us - start_us - 10UL;  // -10 to compensate for code delay
      Serial.println(measured_us);
      //m_per_s = float(scale) * float(SENSOR_DISTANCE) / float(measured_us);

      m_per_s = float(SENSOR_DISTANCE) / float(measured_us);
      km_per_hr = 3.6 * m_per_s;
      mi_per_hr = 2.23694 * m_per_s;
      write_to_display();

      if (!units) {
        Serial.print(mi_per_hr * float(scale));
        Serial.println(F(" MPH"));
      } else {
        Serial.print(km_per_hr * float(scale));
        Serial.println(F(" km/hr"));
      }
      Serial.println();

      waittime = (int)(10UL * measured_us * (unsigned long)train_length / (unsigned long)SENSOR_DISTANCE + 1000UL);  // [ms]
                                                                                                                     // measured_us is needed for a move of SENSOR_DISTANCE um
                                                                                                                     // measured_us * train_length / SENSOR_DISTANCE) time is needed for train_length to pass

      Serial.print(F("Wait "));
      Serial.print(waittime);
      Serial.println(F(" ms"));

      // delay må erstattes med millis
      delay(waittime);
      write_waiting_for_train();
      state = 0;
      draw_button();
      break;
  }
}


void display_logo() {

  //tft.fillRect(0, 90, 240, 34, BLACK);
  tft.setCursor(x_offset + 10, y_offset + 185);
  tft.setTextSize(1);
  tft.setTextColor(RED);
  tft.setFont(&FreeSans18pt7b);
  tft.print("SpeedTracker");
  tft.setFont();
  tft.setCursor(x_offset + 20, y_offset + 200);
  tft.setTextColor(WHITE);
  tft.print("by platelayer,  ");
  tft.print(prog_ver);
}

void write_waiting_for_train(void) {
  tft.fillRect(x_offset + 0, y_offset + 90, 240, 34, GREEN);
  tft.setCursor(x_offset + 8, y_offset + 100);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("WAITING FOR TRAIN..");
  tft.setTextColor(WHITE);
}

void write_sensor_indicator(void) {
  if (!digitalRead(SENSOR_L_PIN)) {
    tft.fillRect(x_offset + 0, y_offset + 0, 14, 14, RED);
  } else {
    tft.fillRect(x_offset + 0, y_offset + 0, 14, 14, BLACK);
  }
  if (!digitalRead(SENSOR_R_PIN)) {
    tft.fillRect(x_offset + 218, y_offset + 0, 14, 14, RED);
  } else {
    tft.fillRect(x_offset + 218, y_offset + 0, 14, 14, BLACK);
  }
}

void write_to_display(void) {
  int intdel, rest;
  if (units) {  // units = 1: KMH
    intdel = int(km_per_hr * float(scale));
    rest = int((km_per_hr * float(scale) - intdel) * 100);
  } else {
    intdel = int(mi_per_hr * float(scale));
    rest = int((mi_per_hr * float(scale) - intdel) * 100);
  }

  tft.fillRect(x_offset + 0, y_offset + 0, 240, 90, BLACK);
  tft.fillRect(x_offset + 0, y_offset + 126, 240, 40, BLACK);

  tft.setFont(&FreeSans18pt7b);

  tft.setTextColor(GREEN);

  tft.setTextSize(2);
  if (intdel < 10) { tft.setCursor(x_offset + 85, y_offset + 60); }
  if (intdel >= 10 && intdel < 100) { tft.setCursor(x_offset + 45, y_offset + 60); }
  if (intdel >= 100) { tft.setCursor(x_offset + 9, y_offset + 60); }

  tft.print(intdel);
  tft.print(".");
  tft.setTextSize(1);
  tft.setCursor(x_offset + 140, y_offset + 60);
  if (rest < 10) { tft.print(0); }
  tft.print(rest);
  tft.setTextColor(WHITE);
  tft.setCursor(x_offset + 180, y_offset + 60);
  tft.setTextSize(1);
  if (!units) {  // units = 0: MPH
    tft.print(" MI");
  } else {
    tft.print("KM");
  }
  tft.setFont();
  if (!units) {  // units = 0: MPH

    if ((km_per_hr * scale) < 10) tft.setCursor(x_offset + 6, y_offset + 128);
    else tft.setCursor(x_offset + 0, y_offset + 128);
    tft.setTextSize(2);
    tft.print(km_per_hr * float(scale));
    tft.print(" km/h");
  } else {  // units = 1: km/hr


    if ((mi_per_hr * scale) < 10) tft.setCursor(x_offset + 6, y_offset + 128);
    else tft.setCursor(x_offset + 0, y_offset + 128);
    tft.setTextSize(2);
    tft.print(mi_per_hr * float(scale));
    tft.print(" MPH");
  }

  tft.setTextSize(2);
  if ((m_per_s) < 10) tft.setCursor(x_offset + 6, y_offset + 148);
  else tft.setCursor(x_offset + 0, y_offset + 148);
  tft.print(m_per_s);
  tft.print(" m/s");
  tft.setCursor(x_offset + 150, y_offset + 148);
  tft.print(int((measured_us + 500) / 1000));
  tft.print(" ms");
  tft.setCursor(x_offset + 129, y_offset + 128);
  display_scale();
  tft.print(" 1/");
  tft.print(scale);

  tft.drawRect(x_offset + 10, y_offset + 68, 220, 14, WHITE);
  tft.fillRect(x_offset + 12, y_offset + 70, 212, 11, BLACK);
  if (units) {
    if (km_per_hr * float(scale) > 212) {
      tft.fillRect(x_offset + 12, y_offset + 70, 217, 11, RED);
    } else {
      tft.fillRect(x_offset + 12, y_offset + 70, int(km_per_hr * float(scale) + 0.5), 11, RED);
    }
  } else {
    if (mi_per_hr * float(scale) > 212) {
      tft.fillRect(x_offset + 12, y_offset + 70, 217, 11, RED);
    } else {
      tft.fillRect(x_offset + 12, y_offset + 70, int(mi_per_hr * float(scale) + 0.5), 11, RED);
    }
  }

  // display_power_status();  // need some more work

  display_logo();
}

void display_scale() {
  if (scale == 45) tft.print(" O");
  if (scale == 76) tft.print("OO");
  if (scale == 87) tft.print("HO");
  if (scale == 120) tft.print("TT");
  if (scale == 160) tft.print(" N");
  if (scale == 220) tft.print(" Z");
}

void print_scale() {
  if (scale == 45) Serial.print("O");
  if (scale == 76) Serial.print("OO");
  if (scale == 87) Serial.print("HO");
  if (scale == 120) Serial.print("TT");
  if (scale == 160) Serial.print("N");
  if (scale == 220) Serial.print("Z");
}

void read_write_scale() {
  if (Serial.available()) {
    serialread = Serial.parseInt();
    if (serialread) { train_length = serialread; }
    Serial.print(F("Train length set to: "));
    Serial.print(train_length);
    Serial.println(F(" cm"));
    Serial.println(F("Waiting for train ..."));
    Serial.println();
  }

  if (scale != scale_old) {
    scale_old = scale;
    Serial.print(F("Scale set to: 1/"));
    Serial.println(scale);
    write_to_display();
  }
}

void display_power_status() {  // display battery power screen, voltage or warning low battery voltage

  // Read the raw ADC value from the voltage divider
  int rawValue = analogRead(POWER_PIN);

  // Map the raw ADC value to the voltage range (0-5V)
  float voltage = map(rawValue, 0, 760, 0, 9000) / 1000.0;

  /*
    Battery monitoring:
    Battery is connected via a diode to Vin. A 100 Kohm resistor is connected from Vin to A0.
    A 10 Kohm resistor is connected to A0 and GND. analogRead on A0.
    Mesuring with multimeter:
    Voltage battery 8,7V
    Voltage over diode 0,82Volt
    Voltage after diode 7,23Volt
    Voltage on A0: 4,96 Volt

                Diode
    Battery>----|>|--O-----> Vin
                     |
                  Resistor 100 Kohm
                     |
                     O-----> A0
                     |
                  Resistor 10 Kohm
                     |
                     |-----> GND
*/
  // tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(x_offset + 20, y_offset + 292);
  Serial.print("Raw: ");
  Serial.println(rawValue);
  Serial.print("Battery: ");
  Serial.println(voltage, 1);
  /*if (voltage < 3) {
    // Print the voltage value to the serial monitor

    Serial.print("Battery: ");
    Serial.print(voltage, 1);  // Display voltage with two decimal places
    Serial.println(" V");
*/
  //tft.print(F("Battery: "));
  tft.print(F("B: "));
  tft.print(voltage, 1);  // Display voltage with two decimal places
  tft.print("V  ");
  //   tft.println(rawValue);
  /*} else {
    tft.print(F("Power: OK"));
  }*/
  //delay(2000);
  batteryLevel = int(map(voltage, 0, 9, 0, 100));
  drawBatteryPercent();
}

void drawBatteryPercent() {

  // draw battery indicator 0-100%

  int x = 110 + x_offset;
  int y = 220 + y_offset;

  tft.fillRect(x + 11, y + 69, 100, 20, BLACK);

  //0-25%
  tft.drawRoundRect(x + 12, y + 69, 51, 20, 5, WHITE);
  tft.fillRoundRect(x + 62, y + 74, 4, 8, 3, WHITE);
  tft.fillRoundRect(x + 13, y + 70, 48, 18, 4, BLACK);
  tft.fillRect(x + 14, y + 72, 10, 14, RED);

  if (batteryLevel > 25) {
    //26-50%
    tft.fillRect(x + 14, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 26, y + 72, 10, 14, YELLOW);
  }

  if (batteryLevel > 50) {
    //51-75%
    tft.fillRect(x + 14, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 26, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 38, y + 72, 10, 14, YELLOW);
  }

  if (batteryLevel > 75) {
    //76-100%
    tft.fillRect(x + 14, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 26, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 38, y + 72, 10, 14, GREEN);
    tft.fillRect(x + 50, y + 72, 10, 14, GREEN);
  }

  tft.drawRoundRect(x + 13, y + 70, 48, 18, 4, BLACK);
  tft.fillRect(x + 69, y + 69, 50, 20, BLACK);
  tft.setCursor(x + 70, y + 72);
  tft.setTextSize(2);
  tft.setFont();
  tft.print(batteryLevel);
  tft.print("%");
}
