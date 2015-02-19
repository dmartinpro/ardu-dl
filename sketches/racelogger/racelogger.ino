#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <RunningAverage.h>
#include <Servo.h>
#include <string.h>
#include <EEPROMex.h>

#define VERSION 1.1

#define LCD_LEFT_ARROW 3
#define LCD_RIGHT_ARROW 4

#define SERVO_PIN 11
#define LCD_SHIELD_CMD_PIN 14 // A0
#define BRAKE_PIN 15          // A1
#define THROTTLE_PIN 16       // A2
#define LCD_PIN_1 8
#define LCD_PIN_2 9
#define LCD_PIN_3 4
#define LCD_PIN_4 5
#define LCD_PIN_5 6
#define LCD_PIN_6 7

#define ANALOGS_SIZE 8
#define NUNCHUK_VALUES_SIZE 7
#define NC_ACC_X_AXIS 0
#define NC_ACC_Y_AXIS 1
#define NC_ACC_Z_AXIS 2
#define NC_JOY_X_AXIS 3
#define NC_JOY_Y_AXIS 4
#define NC_C_BUTTON 5
#define NC_Z_BUTTON 6

#define ANA_ACC_X_AXIS 0
#define ANA_THROTTLE 1
#define ANA_BRAKE 2

#define INTERVAL_NUNCHUK 0
#define INTERVAL_BLUETOOTH 1
#define INTERVAL_SERVO 2
#define INTERVAL_LCD 3

#define INTERVAL_DURATION 0
#define INTERVAL_LAST 1

#define SERVO_ANGLE_THRESHOLD 2 // minimum difference angle to send a rotation signal to the servo

// ID of the settings block
#define CONFIG_VERSION "AL1"
// Tell it where to store your config data in EEPROM
#define EEPROM_BASE 32
// How many writes allowed?
#define EEPROM_MAX_ALLOWED_WRITES 10

RunningAverage avg_angle(10);
RunningAverage avg_angle_cfg(5);
RunningAverage avg_brake(3);
RunningAverage avg_throttle(3);

// #######################################################

const char *app_name = "RaceLogger";
const char *app_author = " by David MARTIN";
char app_screen_name[16];

// LCD shield
LiquidCrystal lcd(LCD_PIN_1, LCD_PIN_2, LCD_PIN_3, LCD_PIN_4, LCD_PIN_5, LCD_PIN_6);
// LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// LCD shield buttons values
enum {
  BP_NONE,
  BP_SELECT,
  BP_LEFT,
  BP_UP,
  BP_DOWN,
  BP_RIGHT
};

byte lcd_button = BP_NONE;
byte prev_button = BP_NONE;
unsigned long prev_button_timestamp = 0;
unsigned long last_none_null_button_timestamp = 0;
const unsigned long BUTTONS_MAX_INACTIVITY = 5000; // After this delay, the configuration menu is replaced with the running screen

uint8_t custom_hex0[8] = {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}; // full square
uint8_t custom_hex1[8] = {0x02,0x06,0x0E,0x1E,0x1E,0x0E,0x06,0x02}; // square with right empty line
uint8_t custom_hex2[8] = {0x08,0x0C,0x0E,0x0F,0x0F,0x0E,0x0C,0x08}; // square with left empty line

const int BUTTONS_DELAY = 200; // minimum milliseconds between two buttons events

// ############ CONFIGURATION MENU VALUES ######################

boolean calibration_mode = false;
unsigned int calibration_item = 0;

int menu_items_number = 13;
const char* line1_options[3] = {"Angle", "Throttle", "Brake"};
#define LINE1_OPTS_NB (sizeof(line1_options) / sizeof(char*))
const char* line2_options[2] = {"Throt-Brake", "Angle"};
#define LINE2_OPTS_NB (sizeof(line2_options) / sizeof(char*))

// non blocking ops times
unsigned long execution_intervals[4][2]; // 4 intervals(servo bluetooth, drawscreen & nunchuk), with 2 values each(duration, last)

// #######################################################

// ############ DISPLAY MODE VALUES ######################

const int LINE1_MODE_ANGLE = 0;
const int LINE1_MODE_THROTTLE = 1;
const int LINE1_MODE_BRAKE = 2;
const int LINE2_MODE_BRAKE_THROTTLE = 0;
const int LINE2_MODE_ANGLE = 1;


// ################# RaceLogger configuration ############
struct ConfigurationStruct {
  char version[4];
  byte line1_display_mode, line2_display_mode; // normal display modes
  int servo_offset_angle; // correction offset to set the cam horizontal
  boolean servo_rotation_clockwise; // rotation direction: true: clockwise, false: counterclockwise
  boolean raw_angle; // consider raw or smoothed bike angle value (more or less jitter)
  unsigned int min_brake, max_brake; // calibration value: brake voltages when no brake, maximum braking
  unsigned int min_throttle, max_throttle; // calibration value: throttle voltage when no throttle, throttle fully open
  float brake_ratio, throttle_ratio; // calibration values: brake & throttle voltage ratios
  boolean servo_enabled;
} settings = {
  CONFIG_VERSION,
  0,0,
  0,
  true,
  false,
  0, 1023,
  0, 1023,
  0,0,
  false
};

boolean unsaved_settings = true;

// ################ EEPROM related variables #############
unsigned int settings_address = 0;
boolean settings_loaded = false;

// ########## BT related variables #######################

unsigned int timeout=0;
unsigned char bt_state=0;

// ########## RaceChrono related variables #######################
// $RC2,<time>,<count>,<xacc>,<yacc>,<zacc>,<rpm/d1>,<d2>,<a1>,<a2>,<a3>,<a4>,<a5>,<a6>,<a7>,<a8>*checksum
const char *pattern1 = "RC2,,"; // everything before analogic fields
const char *pattern2 = ",,,,,"; // everything before analogic fields (there is a comma missing, this is due to the way the final string is built)
unsigned int rc2_msg_counter = 0; // this is "<count>"

// ############### LOGGING related variables #####################
int analogs[ANALOGS_SIZE];

int last_acc_x_axis;
int nunchuk_values[NUNCHUK_VALUES_SIZE];
enum {
  NC_BTN_A,
  NC_BTN_B,
  NC_ACC_X,
  NC_ACC_Y,
  NC_ACC_Z,
  NC_JOY_X,
  NC_JOY_Y
};

byte buffer[6]; // Buffer containing the 6 bytes coming from the nunchuk
byte cnt = 0; // Current buffer index. NOTE: maybe doesn't need to be global...
int acc_horiz_x_axis = 0;
int acc_cw90_x_axis = 0;
int acc_acw90_x_axis = 0;
float acc2servo_ratio = 0;

// ############# Servo related variables ###################
Servo cam_servo;
int last_servo_angle = 0; // last known position


// Timer2 Service (bluetooth related)
ISR(TIMER2_OVF_vect) {
  TCNT2 = 0;
  timeout++;
  if (timeout > 61) {
    bt_state=1;
    timeout=0;
  }
}

void init_timer2(void) {
  TCCR2A |=(1 << WGM21) |(1 << WGM20);
  TCCR2B |= 0x07;   // by clk/1024
  ASSR |=(0<<AS2);  // Use internal clock - external clock not used in Arduino
  TIMSK2 |= 0x01;   // Timer2 Overflow Interrupt Enable
  TCNT2 = 0;
  sei();
}

void cleantime() {
  timeout = 0;
  bt_state = 0;
}

boolean load_EEPROM_settings() {
  EEPROM.readBlock(settings_address, settings);
  return (settings.version == CONFIG_VERSION);
}

void save_EEPROM_settings() {
   EEPROM.writeBlock(settings_address, settings);
}

void handshake() {
  Wire.beginTransmission(0x52);
  Wire.write(0x00);
  Wire.endTransmission();
}

/*
 * Send a read command to the nunchuk and fill the nunchuk_values array
 */
void read_nunchuk() {
  Wire.requestFrom(0x52, 6);
  while (Wire.available()) {
    buffer[cnt] = Wire.read();
    cnt++;
  }

  if (cnt >= 5) {
    parse_nunchuk_response();
  }
  cnt = 0;
  handshake();
}

void parse_nunchuk_response() {
  byte joy_x_axis = buffer[0]; // joystick X axe (0-255)
  byte joy_y_axis = buffer[1]; // joystick Y axe (0-255)
  int accel_x_axis = buffer[2] * 4; // acc x
  int accel_y_axis = buffer[3] * 4; // acc y
  int accel_z_axis = buffer[4] * 4; // acc z

  byte z_button = 0; // bouton Z
  byte c_button = 0; // bouton c

  if ((buffer[5] >> 0) & 1)
    z_button = 1;

  if ((buffer[5] >> 1) & 1)
    c_button = 1;

  if ((buffer[5] >> 2) & 1)
    accel_x_axis += 2;

  if ((buffer[5] >> 3) & 1)
    accel_x_axis += 1;

  if ((buffer[5] >> 4) & 1)
    accel_y_axis += 2;

  if ((buffer[5] >> 5) & 1)
    accel_y_axis += 1;

  if ((buffer[5] >> 6) & 1)
    accel_z_axis += 2;

  if ((buffer[5] >> 7) & 1)
    accel_z_axis += 1;

  nunchuk_values[NC_ACC_X_AXIS] = accel_x_axis;
  nunchuk_values[NC_ACC_Y_AXIS] = accel_y_axis;
  nunchuk_values[NC_ACC_Z_AXIS] = accel_z_axis;
  nunchuk_values[NC_JOY_X_AXIS] = joy_x_axis;
  nunchuk_values[NC_JOY_Y_AXIS] = joy_y_axis;
  nunchuk_values[NC_C_BUTTON] = c_button;
  nunchuk_values[NC_Z_BUTTON] = z_button;

}

/**
 * GPS checksum
 */
char checksum(char* input) {
  char c;
  char checksum = 0;
  unsigned short position = 0;

  while (input[position] != '\0') {
    c = input[position]; // TODO input[position++]
    checksum = checksum ^ c;
    position++;
  }
  return checksum;
}

/**
 * Build the RaceChrono datalogger Bluetooth message
 * NMEA 0183 type checksum, with two uppercase hexadecimal digits (one byte)
 */
char* get_RC2_message(int analog_values[]) {

  char core_value[150];
  strcpy(core_value, pattern1);
  char count_buffer[5];
//  itoa(rc2_msg_counter++, count_buffer, 10);
  sprintf(count_buffer, "%i", rc2_msg_counter++); // TODO verify if it works as expected with itoa
  strcat(core_value, count_buffer);
  strcat(core_value, pattern2);

  char buffer[6];
  for(int i = 0; i < ANALOGS_SIZE; i++) {
    strcat(core_value, ",");
//    itoa(analog_values[i], buffer, 10);
    sprintf(buffer, "%i", analog_values[i]);
    strcat(core_value, buffer);
  }

  char chk[2];
  sprintf(chk, "%X", checksum(core_value));
  char output[150];
  // build the final string
  strcpy(output, "$");
  strcat(output, core_value);
  strcat(output, "*");
  if (strlen(chk) == 1) {
    strcat(output, "0");
  }
  strcat(output, chk);
  strcat(output, "\r\n"); // must be terminated with CR LF

  return output;
}

int get_brake_value() {
  int pin_value = analogRead(BRAKE_PIN);
  if (settings.brake_ratio == 0) {
    set_brake_ratio();
  }
  if (pin_value < settings.min_brake) {
    pin_value = settings.min_brake;
  } else if (pin_value > settings.max_brake) {
    pin_value = settings.max_brake;
  }
  float value = ((float)(pin_value - settings.min_brake)) * settings.brake_ratio;
  return (int)(value+0.5);
}

int get_throttle_value() {
  int pin_value = analogRead(THROTTLE_PIN);
  if (settings.throttle_ratio == 0) {
    set_throttle_ratio();
  }
  if (pin_value < settings.min_throttle) {
    pin_value = settings.min_throttle;
  } else if (pin_value > settings.max_throttle) {
    pin_value = settings.max_throttle;
  }
  float value = ((float)(pin_value - settings.min_throttle)) * settings.throttle_ratio;
  return (int)(value+0.5);
}

int get_bike_angle() {
  if (settings.raw_angle) {
    return get_raw_bike_angle();
  } else {
    return get_avg_bike_angle();
  }
}

/**
 * Return the right and raw bike angle depending on the nunchuk read value
 */
int get_raw_bike_angle() {
  float angle = ((float)(last_acc_x_axis - acc_horiz_x_axis)) * acc2servo_ratio + 90;

  if (settings.servo_rotation_clockwise) {
    return angle >= 0 ?(int)(angle+0.5) :(int)(angle-0.5);
  } else {
    return 180 - (angle >= 0 ?(int)(angle+0.5) :(int)(angle-0.5));
  }
}

/**
 * Return the right smoothed (avg) bike angle depending on the nunchuk read value
 */
int get_avg_bike_angle() {
  float angle = ((float)(avg_angle.avg() - acc_horiz_x_axis)) * acc2servo_ratio + 90;

  if (settings.servo_rotation_clockwise) {
    return angle >= 0 ?(int)(angle+0.5) :(int)(angle-0.5);
  } else {
    return 180 - (angle >= 0 ? (int)(angle+0.5) :(int)(angle-0.5));
  }
}

void display() {
  char line1[16];
  char line2[16];
  char value[4];

  strcpy(line1, "");
  strcpy(line2, "");

  switch (settings.line1_display_mode) {
    case LINE1_MODE_ANGLE:
      strcpy(line1, "Angle: ");
      itoa(analogs[ANA_ACC_X_AXIS], value, 10);
      strcat(line1, value);
      strcat(line1, " deg");
      break;
    case LINE1_MODE_THROTTLE:
      strcpy(line1, "Throttle: ");
      itoa(analogs[ANA_THROTTLE], value, 10);
      strcat(line1, value);
      strcat(line1, " %");
      break;
    case LINE1_MODE_BRAKE:
      strcpy(line1, "Brake: ");
      itoa(analogs[ANA_BRAKE], value, 10);
      strcat(line1, value);
      strcat(line1, " %  ");
      break;
  }

// #####

  if (settings.line2_display_mode == LINE2_MODE_BRAKE_THROTTLE) {
    int brake_value = analogs[ANA_BRAKE];
    int throttle_value = analogs[ANA_THROTTLE];

    int brake_display_value = 0;
    int throttle_display_value = 0;

    brake_display_value = (int) (((float) brake_value)*8/100+0.5);
    throttle_display_value = (int) (((float) throttle_value)*8/100+0.5);

    for(int i = 7; i >= 0; i--) {
      if (i < brake_display_value) {
        strcat(line2, "\x03");
      } else {
        strcat(line2, "\x14");
      }
    }

    for(int i = 0; i < 8; i++) {
      if (i < throttle_display_value) {
        strcat(line2, "\x04");
      } else {
        strcat(line2, "\x14");
      }
    }
  } else {
    int angle_value = (analogs[ANA_ACC_X_AXIS] * 100) / 90; // angle is in range -90;90, just convert it to -100;100
    angle_value = 2*angle_value; // And consider 50% (45° as the maximum a conventional driver can lean)
    int angle_display_value = 0;
    if (angle_value < 0) {
      angle_display_value = (int) (((float) angle_value)*8/100-0.5);
    } else {
      angle_display_value = (int) (((float) angle_value)*8/100+0.5);
    }

    if (angle_display_value < 0) {
      for(int i = 7; i >= 0; i--) {
        if (i < (-1*angle_display_value)) {
          strcat(line2, "\x03");
        } else {
          strcat(line2, "\x14");
        }
      }
      strcat(line2, "        ");
    } else {
      strcat(line2, "        ");
      for(int i = 0; i < 8; i++) {
        if (i < angle_display_value) {
          strcat(line2, "\x04");
        } else {
          strcat(line2, "\x14");
        }
      }
    }
  }

  draw(line1, line2);
}

void display_values() {
  int pause = 2000;
  char screen[32];
  char value[5];

  strcpy(screen, "Brk:");
  itoa(settings.min_brake, value, 10);
  strcat(screen, value);
  strcat(screen, "-");
  itoa(settings.max_brake, value, 10);
  strcat(screen, value);

  strcat(screen, "\n");

  strcat(screen, "Thr:");
  itoa(settings.min_throttle, value, 10);
  strcat(screen, value);
  strcat(screen, "-");
  itoa(settings.max_throttle, value, 10);
  strcat(screen, value);

  draw(screen, pause);

  strcpy(screen, "Servo: ");
  if (settings.servo_enabled) {
    strcat(screen, "ON");
  } else {
    strcat(screen, "OFF");
  }
  if (settings.raw_angle) {
    strcat(screen, " (RAW)");
  } else {
    strcat(screen, " (AVG)");
  }
  strcat(screen, "\n");
  strcat(screen, "Rotation: ");
  if (settings.servo_rotation_clockwise) {
    strcat(screen, "CW");
  } else {
    strcat(screen, "CCW");
  }

  draw(screen, pause);

  strcpy(screen, "Horiz:");
  itoa(avg_angle_cfg.avg(), value, 10);
  strcat(screen, value);
  strcat(screen, "\n");
  strcat(screen, "Offset: ");
  itoa(settings.servo_offset_angle, value, 10);
  strcat(screen, value);

  draw(screen, pause);
}

/**
 * Read the Shield button and return a MENWIZ compliant button code
 * NOTE: ESCAPE button is not mapped. Should add a new analog button.
 */
byte read_shield_button(void) {

  /* No button detected value */
  byte current_button = BP_NONE;

  /* Read analog input A0 (LCD_SHIELD_CMD_PIN) */
  unsigned int val = analogRead(LCD_SHIELD_CMD_PIN);

  /* Test against a set of values */
  if (val < 30) { // RIGHT (0)
    current_button = BP_RIGHT;
  } else if (val < 210) { // UP (132)
    current_button = BP_UP;
  } else if (val < 390) { // DOWN (306)
    current_button = BP_DOWN;
  } else if (val < 560) { // LEFT (478)
    current_button = BP_LEFT;
  } else if (val < 800) { // SELECT (720)
    current_button = BP_SELECT;
  }

  if ((millis() - prev_button_timestamp) < BUTTONS_DELAY) {
    return BP_NONE;
  }

  if (current_button != BP_NONE) {
    last_none_null_button_timestamp = millis();
  }

  prev_button_timestamp = millis();
  prev_button = current_button;

  return current_button;
}

void set_horizon_value() {
  for (int i = 0; i < 10; i++) {
    read_nunchuk();
    avg_angle_cfg.add(nunchuk_values[NC_ACC_X_AXIS]);
    delay(10);
  }
  acc_horiz_x_axis = avg_angle_cfg.avg();
  unsaved_settings = true;
}

void set_minimum_brake() {
  settings.min_brake = analogRead(BRAKE_PIN);
  set_brake_ratio();
  unsaved_settings = true;
}

void set_maximum_brake() {
  settings.max_brake = analogRead(BRAKE_PIN);
  set_brake_ratio();
  unsaved_settings = true;
}

void set_minimum_throttle() {
  settings.min_throttle = analogRead(THROTTLE_PIN);
  set_throttle_ratio();
  unsaved_settings = true;
}

void set_maximum_throttle() {
  settings.max_throttle = analogRead(THROTTLE_PIN);
  set_throttle_ratio();
  unsaved_settings = true;
}

void set_servo_activation() {
  settings.servo_enabled = !settings.servo_enabled;
  unsaved_settings = true;
}

void set_servo_rotation_direction() {
  settings.servo_rotation_clockwise = !settings.servo_rotation_clockwise;
  unsaved_settings = true;
}

void set_calibrate_servo_horiz_offset() {
  if (lcd_button == BP_LEFT) {
    settings.servo_offset_angle--;
  } else if (lcd_button == BP_RIGHT){
    settings.servo_offset_angle++;
  }
  execution_intervals[INTERVAL_SERVO][INTERVAL_LAST] = 0;
  drive_servo();
  unsaved_settings = true;
}

void set_raw_servo() {
  settings.raw_angle = !settings.raw_angle;
  unsaved_settings = true;
}

void set_line1_display_mode() {
  if (lcd_button == BP_LEFT) {
    if (settings.line1_display_mode <= 0) {
      settings.line1_display_mode = LINE1_OPTS_NB-1;
    } else {
      settings.line1_display_mode--;
    }
  } else if (lcd_button == BP_RIGHT){
    if (settings.line1_display_mode >= LINE1_OPTS_NB-1) {
      settings.line1_display_mode = 0;
    } else {
      settings.line1_display_mode++;
    }
  }
  unsaved_settings = true;
}

void set_line2_display_mode() {
  if (lcd_button == BP_LEFT) {
    if (settings.line2_display_mode <= 0) {
      settings.line2_display_mode = LINE2_OPTS_NB-1;
    } else {
      settings.line2_display_mode--;
    }
  } else if (lcd_button == BP_RIGHT){
    if (settings.line2_display_mode >= LINE2_OPTS_NB-1) {
      settings.line2_display_mode = 0;
    } else {
      settings.line2_display_mode++;
    }
  }
  unsaved_settings = true;
}

void set_save_settings() {
  save_EEPROM_settings();
  unsaved_settings = false;
}

void set_brake_ratio() {
  float delta =(float)(settings.max_brake - settings.min_brake);
  float ratio = 100/delta;
  if (ratio == 0) {
    settings.brake_ratio = 1.0;
  } else if (ratio < 0) {
    settings.brake_ratio = -1.0*ratio;
  } else {
    settings.brake_ratio = ratio;
  }
}

void set_throttle_ratio() {
  float delta =(float)(settings.max_throttle - settings.min_throttle);
  float ratio = 100/delta;
  if (ratio == 0) {
    settings.throttle_ratio = 1.0;
  } else if (ratio < 0) {
    settings.throttle_ratio = -1.0*ratio;
  } else {
    settings.throttle_ratio = ratio;
  }
}

/*
 * Log data, send them through BT and drive the servo
 */
void acquire_data() {
  // Read throttle
  analogs[ANA_THROTTLE] = get_throttle_value();
  // delay 10ms to let the ADC recover:
  delay(10);
  // Read brake
  analogs[ANA_BRAKE] = get_brake_value();
  // delay 10ms to let the ADC recover:
  delay(10);
  // Read running avg angle, and set it up relative to 90°
  analogs[ANA_ACC_X_AXIS] = 90-get_bike_angle();

  // Get nunchuk data every 'execution_intervals[INTERVAL_NUNCHUK][INTERVAL_DURATION]' ms
  if (millis() - execution_intervals[INTERVAL_NUNCHUK][INTERVAL_LAST] > execution_intervals[INTERVAL_NUNCHUK][INTERVAL_DURATION]) {
    execution_intervals[INTERVAL_NUNCHUK][INTERVAL_LAST] = millis();

    read_nunchuk();
    last_acc_x_axis = nunchuk_values[NC_ACC_X_AXIS];
    avg_angle.add(nunchuk_values[NC_ACC_X_AXIS]);
  }

}

/*
 * Send the BT message to RaceChrono
 */
void send_rc2_message() {
  if (millis() - execution_intervals[INTERVAL_BLUETOOTH][INTERVAL_LAST] > execution_intervals[INTERVAL_BLUETOOTH][INTERVAL_DURATION]) {
    execution_intervals[INTERVAL_BLUETOOTH][INTERVAL_LAST] = millis();

    // Verify bluetooth connection status
    if (bt_state == 1) {
      Serial.print(get_RC2_message(analogs));
    }
  }

}

void reset_avg_values() {
  avg_angle.clr();
  avg_angle_cfg.clr();
  avg_brake.clr();
  avg_throttle.clr();
}

/*
 * Drive the servo.
 * If the previous angle measure is older than the defined interval (INTERVAL_DURATION), then
 * read the angle of the bike and if it has changed more than the THRESHOLD, move the servo accordingly.
 */
void drive_servo() {
  if (!settings.servo_enabled) {
    return;
  }

  if (millis() - execution_intervals[INTERVAL_SERVO][INTERVAL_LAST] > execution_intervals[INTERVAL_SERVO][INTERVAL_DURATION]) {
    execution_intervals[INTERVAL_SERVO][INTERVAL_LAST] = millis();
    int angle = get_bike_angle();
    if (last_servo_angle - angle > SERVO_ANGLE_THRESHOLD || -1*(last_servo_angle - angle)> SERVO_ANGLE_THRESHOLD) {
      cam_servo.write(angle + settings.servo_offset_angle);
      last_servo_angle = angle;
    }
  }
}


// ################ LCD related #####################################

void draw(char* screen, int wait) {
  char line1[16];
  char line2[16];

  if (strlen(screen) >= 32) {
    for (int i = 0; i < 32; i++) {
      if (i < 16) {
        strcat(line1, &screen[i]);
      } else {
        strcat(line2, &screen[i]);
      }
    }
  } else {
    strcpy(line1, "ERROR");
    strcpy(line2, "Can't print line");
  }
  draw(line1, line2, wait);
}

/* Draw the two lines of the LCD shield */
void draw(char* line1, char* line2) {
  draw(line1, line2, 0);
}

void draw(char* line1, char* line2, long wait) {
  if (millis() - execution_intervals[INTERVAL_LCD][INTERVAL_LAST] > execution_intervals[INTERVAL_LCD][INTERVAL_DURATION]) {
    execution_intervals[INTERVAL_LCD][INTERVAL_LAST] = millis();

    lcd.setCursor(0, 0);
    lcd.print(line1);
    int line_length = strlen(line1);
    for(int i = 0; i < 16 - line_length; ++i) {
      lcd.write(" ");
    }
    lcd.setCursor(0, 1);
    lcd.print(line2);
    line_length = strlen(line2);
    for(int i = 0; i < 16 - line_length; ++i) {
      lcd.write(" ");
    }

    execution_intervals[INTERVAL_LCD][INTERVAL_DURATION] = wait;
  }
}

void draw_horizon_value() {
  read_nunchuk();

  char line2[16];
  char num_value[5];
  strcpy(line2, "Value: ");
  itoa(nunchuk_values[NC_ACC_X_AXIS], num_value, 10);
  strcat(line2, num_value);
  strcat(line2, " (");
  itoa(acc_horiz_x_axis, num_value, 10);
  strcat(line2, num_value);
  strcat(line2, ")");
  draw("1. Config horiz.", line2);
}

void draw_minimum_brake() {
  char line2[16];
  char num_value[5];
  strcpy(line2, "Min brake: ");
  itoa(settings.min_brake, num_value, 10);
  strcat(line2, num_value);
  draw("2. Config brake", line2);
}

void draw_maximum_brake() {
  char line2[16];
  char num_value[5];
  strcpy(line2, "Max brake: ");
  itoa(settings.max_brake, num_value, 10);
  strcat(line2, num_value);
  draw("3. Config brake", line2);
}

void draw_minimum_throttle() {
  char line2[16];
  char num_value[5];
  strcpy(line2, "Min. : ");
  itoa(settings.min_throttle, num_value, 10);
  strcat(line2, num_value);
  draw("4. Conf throttle", line2);
}

void draw_maximum_throttle() {
  char line2[16];
  char num_value[5];
  strcpy(line2, "Max. : ");
  itoa(settings.max_throttle, num_value, 10);
  strcat(line2, num_value);
  draw("5. Conf throttle", line2);
}

void draw_servo_activation() {
  char line2[16];
  strcpy(line2, "Activated: ");
  strcat(line2, (settings.servo_enabled) ? "Y":"N");
  draw("6. Conf servo", line2);
}

void draw_servo_rotation_direction() {
  char line2[16];
  strcpy(line2, "Dir: ");
  strcat(line2, (settings.servo_rotation_clockwise) ? "<-" : "->");
  draw("7. Config servo", line2);
}

void draw_calibrate_servo_horiz_offset() {
  char line2[16];
  char num_value[5];
  strcpy(line2, "Offset: ");
  itoa(settings.servo_offset_angle, num_value, 10);
  strcat(line2, num_value);
  draw("8. Config servo", line2);
}

void draw_raw_servo() {
  char line2[16];
  strcpy(line2, "Raw/Avg: ");
  strcat(line2, (settings.raw_angle) ? "RAW" : "AVG");
  draw("9. Config servo", line2);
}

void draw_line1_display_mode() {
  char line2[16];
  strcpy(line2, "Line1:");
  strcat(line2, line1_options[settings.line1_display_mode]);
  draw("10. Options", line2);
}

void draw_line2_display_mode() {
  char line2[16];
  strcpy(line2, "Line2:");
  strcat(line2, line2_options[settings.line2_display_mode]);
  draw("11. Options", line2);
}

void draw_save_settings() {
  char line1[16];
  char line2[16];
  strcpy(line1, "12. Save conf");
  strcat(line1, (unsaved_settings) ? " * " : "   ");
  strcpy(line2, "Lft=No Right=Yes");
  draw(line1, line2);
}

void draw_about_menu_item() {
  char line1[16];
  char line2[16];
  strcpy(line1, "13. About");
  strcpy(line2, " Press Right -->");
  draw(line1, line2);
}

void draw_about_screen() {
  char line2[16];
  strcpy(line2, app_author);
  draw(app_screen_name, line2, 1500);
}

void draw_menu() {
  lcd_button = read_shield_button();

  if (prev_button == BP_NONE && lcd_button == BP_NONE && (millis()-last_none_null_button_timestamp) > BUTTONS_MAX_INACTIVITY) {
    calibration_mode = false;
  } else if (lcd_button == BP_SELECT) {
    calibration_mode = !calibration_mode;
  }

  if (calibration_mode) {
    if (lcd_button == BP_DOWN) {
      if (calibration_item < menu_items_number) {
        calibration_item++;
      } else {
        calibration_item = 1;
      }
    } else if (lcd_button == BP_UP) {
      if (calibration_item > 1) {
        calibration_item--;
      } else {
        calibration_item = menu_items_number;
      }
    }

    switch (calibration_item) {
      case 1 :
        if (lcd_button == BP_RIGHT) {
          set_horizon_value();
        }
        draw_horizon_value();
        break;
      case 2 :
        if (lcd_button == BP_RIGHT) {
          set_minimum_brake();
        }
        draw_minimum_brake();
        break;
      case 3 :
        if (lcd_button == BP_RIGHT) {
          set_maximum_brake();
        }
        draw_maximum_brake();
        break;
        if (lcd_button == BP_RIGHT) {
          set_minimum_throttle();
        }
      case 4 :
        if (lcd_button == BP_RIGHT) {
          set_minimum_throttle();
        }
        draw_minimum_throttle();
        break;
      case 5 :
        if (lcd_button == BP_RIGHT) {
          set_maximum_throttle();
        }
        draw_maximum_throttle();
        break;
      case 6 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_servo_activation();
        }
        draw_servo_activation();
        break;
      case 7 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_servo_rotation_direction();
        }
        draw_servo_rotation_direction();
        break;
      case 8 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_calibrate_servo_horiz_offset();
        }
        draw_calibrate_servo_horiz_offset();
        break;
      case 9 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_raw_servo();
        }
        draw_raw_servo();
        break;
      case 10 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_line1_display_mode();
        }
        draw_line1_display_mode();
        break;
      case 11 :
        if (lcd_button == BP_LEFT || lcd_button == BP_RIGHT) {
          set_line2_display_mode();
        }
        draw_line2_display_mode();
        break;
      case 12 :
        if (lcd_button == BP_RIGHT) {
          set_save_settings();
        }
        draw_save_settings();
        break;
      case 13 :
        if (lcd_button == BP_RIGHT) {
          draw_about_screen();
        }
        draw_about_menu_item();
        break;
      default :
        calibration_item = 1;
      }
  } else {
    display();
  }

}

/**
 * Argh, Arduino's sprintf doesn't support float numbers
 */
char *ftoa(char *a, double f, int precision) {
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

// ###############################################################


void setup() {

  strcpy(app_screen_name, app_name);
  strcat(app_screen_name, " ");
  char _version[5];
  ftoa(_version, VERSION, 1);
  strcat(app_screen_name, _version);

  lcd.begin(16, 2);

  lcd.createChar(3, custom_hex1);
  delay(40);
  lcd.createChar(4, custom_hex2);
  delay(40);
  lcd.clear();

  draw(app_screen_name, "Warming up...");

  // EEPROM stuff
  settings_address  = EEPROM.getAddress(sizeof(ConfigurationStruct)); // Size of settings object
  EEPROM.setMemPool(EEPROM_BASE, EEPROMSizeUno); //Set memorypool base to EEPROM_BASE, assume Arduino Uno board
  EEPROM.setMaxAllowedWrites(EEPROM_MAX_ALLOWED_WRITES);
  settings_loaded = load_EEPROM_settings();


  // Serial(used by BT Shield)
  Serial.begin(38400);

  // Nunchuk init:
  Wire.begin();
  Wire.beginTransmission(0x52);
  Wire.write(0x40);
  Wire.write(0x00);
  Wire.endTransmission();

// Bluetooth init:
  pinMode(13, OUTPUT);
  attachInterrupt(0, cleantime, FALLING);
  init_timer2();

  // Set up Nunchuk dependent values
  read_nunchuk();
  // Angular sensor values (min, med, max)
  acc_cw90_x_axis = 180; // cw: clockwise, hardcoded for now
  acc_horiz_x_axis = ((nunchuk_values[NC_ACC_X_AXIS] - 395) > 100 || (nunchuk_values[NC_ACC_X_AXIS] - 395) < -100) ? 395 : nunchuk_values[NC_ACC_X_AXIS]; // ~90° (horizontal)
  acc_acw90_x_axis = 610; // acw: anti clockwise, hardcoded for now

  float delta =(float)(acc_cw90_x_axis - acc_acw90_x_axis);
  if (delta != 0) {
    acc2servo_ratio = 180/delta;
    if (acc2servo_ratio < 0) {
      acc2servo_ratio = -1.0*acc2servo_ratio;
    }
  }

  // Servo init:
  last_servo_angle = 90;
  cam_servo.attach(SERVO_PIN);
  cam_servo.write(last_servo_angle);  // set servo to an approximative mid course position

  // Analog data init:
  pinMode(LCD_SHIELD_CMD_PIN, INPUT); // LCD commands pin
  pinMode(THROTTLE_PIN, INPUT); // Throttle pin
  pinMode(BRAKE_PIN, INPUT); // Brake pin

  last_acc_x_axis = 0;

  reset_avg_values();

  // Time references init:
  execution_intervals[INTERVAL_NUNCHUK][INTERVAL_DURATION] = 25; // poll nunchuk every n ms
  execution_intervals[INTERVAL_NUNCHUK][INTERVAL_LAST] = 0;

  execution_intervals[INTERVAL_SERVO][INTERVAL_DURATION] = 40; // write servo position every n ms
  execution_intervals[INTERVAL_SERVO][INTERVAL_LAST] = 0;

  execution_intervals[INTERVAL_BLUETOOTH][INTERVAL_DURATION] = 100; // send rc2 message every n ms
  execution_intervals[INTERVAL_BLUETOOTH][INTERVAL_LAST] = 0;

  execution_intervals[INTERVAL_LCD][INTERVAL_DURATION] = 20; // wait n ms before writing on the screen again
  execution_intervals[INTERVAL_LCD][INTERVAL_LAST] = 0;

  set_horizon_value();

  draw(app_screen_name, "Running...", 1500);
}

/* Main loop */
void loop() {

  draw_menu();

  if (!calibration_mode) {
    acquire_data();
    send_rc2_message();
    drive_servo();
  }

}
