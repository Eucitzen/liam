/*
 Liam - DIY Robot Lawn Mower

 Display Library
 Changelog: 
 2024-09-18 - combined all display options in this library for ease

 ======================
  Licensed under GPLv3
 ======================
*/
/*
  Base class for displays.
  Default behavior is to print to serial port.

  Subclasses should implement the following functions:
  - size_t write(uint8_t)
  - void setCursor(int col, int row)
  - void clear()
  - void blink();

  If necesary, also override
  - boolean initialize()
  but make sure to also call MYDISPLAY::initialize()
*/

#include "MyDisplay.h"
//#include "myLcd.h"
#include <LiquidCrystal_I2C.h>

MYDISPLAY::MYDISPLAY(BATTERY* batt, WHEELMOTOR* left, WHEELMOTOR* right, CUTTERMOTOR* cut, BWFSENSOR* bwf, MOTIONSENSOR* comp, int* state)
{
  Battery = batt;
  leftMotor = left;
  rightMotor = right;
  cutter = cut;
  sensor = bwf;
  compass = comp;
  moverstate = state;
}

boolean MYDISPLAY::initialize()
{
  for (int i=0; i<3; i++)
    blink();

  clear();

  return true;
}

// Do NOT override. Implement basic commands here
void MYDISPLAY::update()
{
  // Row 1: Sensor status
  setCursor(0,0);
#if __MS9150__ || __MS5883L__ || __ADXL345__ || __MMA7455__
  print(F("Comp: "));
  print(compass->getHeading());
#else
  print(F("Sens: "));
  print(F("Disabled"));
#endif

  print(F("InOut: "));
  print(!sensor->isOutOfBounds(0));
  print(!sensor->isOutOfBounds(1));

  print("\n");
  // Row 2: Motor load
  print(F("LMoto: "));
  print(leftMotor->getLoad());
  print(F(" RMoto: "));
  print(rightMotor->getLoad());

  print("\n");
  // Row 3: Battery
  print(F("Battery: "));
  print(Battery->getVoltage());

  print("\n");
  // Row 4: State and Error data
  print(F("State: "));

  switch (*moverstate)
  {
    case MOWING:
      print(F("MOWING"));
      break;
    case LAUNCHING:
      print(F("LAUNCHING"));
      break;
    case DOCKING:
      print(F("DOCKING"));
      break;
    case CHARGING:
      print(F("CHARGING"));
      break;
    case LOOKING_FOR_BWF:
      print(F("LOOKING"));
      break;
      case IDLE:
      print(F("IDLE"));
      break;
  }
}


// DEVICE SPECIFIC FUNCTIONS

// Override this for each type of display
size_t MYDISPLAY::write(uint8_t s)
{
  // By default just write to serial port
  return Serial.write(s);
}

// Override this for each type of display
void MYDISPLAY::setCursor(int col, int row)
{
  // For a serial port, do nothing
}

// Override this for each type of display
void MYDISPLAY::clear()
{
  // For a serial port, do very little
  println();
}

// Override this for each type of display
void MYDISPLAY::blink()
{
  // For a serial port, do very little
  println("*");
}


/*
      LCD Display Library
 ======================
  Licensed under GPLv3
 ======================
*/



myLCD::myLCD(BATTERY* batt, WHEELMOTOR* left, WHEELMOTOR* right, CUTTERMOTOR* cut, BWFSENSOR* bwf, MOTIONSENSOR* comp, int* state) :
  MYDISPLAY(batt, left, right, cut, bwf, comp, state),
  // You may need to modify this line to work with your LCD controller
  lcd(LCD_I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE)
{
}

boolean myLCD::initialize()
{
  current_row = current_col = 0;
  lcd.begin(max_cols, max_rows);

  // Hide cursor and turn on backlight
  lcd.noCursor();
  lcd.backlight();

  return MYDISPLAY::initialize();
}


size_t myLCD::write(uint8_t s)
{
  current_col++;

  if(s == '\n' || current_col >= max_cols)
  {
    current_row++;
    current_col = 0;
    if(current_row >= max_rows)
      current_row = 0;
    lcd.setCursor(current_col, current_row);
  }

  lcd.write(s);

  // Uncomment to write to serial port also
  /* MYDISPLAY::write(s); */
}

void myLCD::setCursor(int col, int row)
{
  current_row = row;
  current_col = col;
  lcd.setCursor(col, row);
}

void myLCD::clear()
{
  lcd.clear();
  setCursor(0,0);
}

void myLCD::blink()
{
  // Flash backlight
  lcd.backlight();
  delay(100);
  lcd.noBacklight();
  delay(100);
  lcd.backlight();
}
