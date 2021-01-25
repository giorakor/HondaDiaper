#include <Arduino.h>
#include <Servo.h>
// pins allocation
#define PIN_RIGHT 3
#define PIN_LEFT 5
#define PIN_SW1 4
#define PIN_SW2 7
#define PIN_REDLED 8
#define PIN_GRNLED 10
#define PIN_A_FB 0
#define PIN_A_SPEED 2
#define PIN_A_slider 1
#define PIN_LEFT_FWD 2
#define PIN_LEFT_BACK 9
#define PIN_RIGHT_FWD 11
#define PIN_RIGHT_BACK 12

// motors
#define servo_zero 90

#define cycle_time 1 // milisec

Servo RIGHT;
Servo LEFT;

int A_FB, A_Speed, A_slider;
int LEFT_VAL, RIGHT_VAL,usr_power;
long last_time, lastGRNblink, lastREDblink, last_changed_dir;
bool GRN_LED, RED_LED;
char one_PID_0_PWR, SW1_raw, SW2_raw, SW1, SW2, SW1_count, SW2_count, SW1_prev_raw, SW2_prev_raw, SW1_prev, SW2_prev;
char lb, lb_raw, lb_count, lb_prev_raw, lb_prev;
char lf, lf_raw, lf_count, lf_prev_raw, lf_prev;
char rb, rb_raw, rb_count, rb_prev_raw, rb_prev;
char rf, rf_raw, rf_count, rf_prev_raw, rf_prev;
bool walk;
void readPIN_Inputs()
{
  A_Speed = 1023 - analogRead(PIN_A_SPEED);
  A_slider = analogRead(PIN_A_slider) - 512;
  SW1_prev_raw = SW1_raw;
  SW2_prev_raw = SW2_raw;
  SW1_prev = SW1;
  SW2_prev = SW2;
  SW1_raw = 1 - digitalRead(PIN_SW1);
  SW2_raw = 1 - digitalRead(PIN_SW2);
  if (SW1_raw == SW1_prev_raw)
    SW1_count += 1;
  else
    SW1_count = 0;
  if (SW2_raw == SW2_prev_raw)
    SW2_count += 1;
  else
    SW2_count = 0;
  if (SW1_count > 50)
  {
    SW1 = SW1_raw;
    SW1_count = 51;
  }
  if (SW2_count > 50)
  {
    SW2 = SW2_raw;
    SW2_count = 51;
  }

  lb_prev_raw = lb_raw;
  lb_prev = lb;
  lb_raw = 1 - digitalRead(PIN_LEFT_BACK);
  if (lb_raw == lb_prev_raw)
    lb_count += 1;
  else
    lb_count = 0;
  if (lb_count > 10)
  {
    lb = lb_raw;
    lb_count = 11;
  }

  lf_prev_raw = lf_raw;
  lf_prev = lf;
  lf_raw = 1 - digitalRead(PIN_LEFT_FWD);
  if (lf_raw == lf_prev_raw)
    lf_count += 1;
  else
    lf_count = 0;
  if (lf_count > 10)
  {
    lf = lf_raw;
    lf_count = 11;
  }

  rb_prev_raw = rb_raw;
  rb_prev = rb;
  rb_raw = 1 - digitalRead(PIN_RIGHT_BACK);
  if (rb_raw == rb_prev_raw)
    rb_count += 1;
  else
    rb_count = 0;
  if (rb_count > 10)
  {
    rb = rb_raw;
    rb_count = 11;
  }

  rf_prev_raw = rf_raw;
  rf_prev = rf;
  rf_raw = 1 - digitalRead(PIN_RIGHT_FWD);
  if (rf_raw == rf_prev_raw)
    rf_count += 1;
  else
    rf_count = 0;
  if (rf_count > 10)
  {
    rf = rf_raw;
    rf_count = 11;
  }
}

void print_tele()
{
  //Serial.print("valT ");
  //Serial.print(val_tilter);
  //Serial.print("TW ");
  //Serial.println(tilt_W);
}
void toggle(bool &flag)
{
  flag = 1 - flag;
}
float clipF(float val, float lower, float upper)
{
  if (val > upper)
    val = upper;
  if (val < lower)
    val = lower;
  return (val);
}
void toggle_GRNLED(uint32_t delay_time)
{
  if (millis() - lastGRNblink > delay_time)
  {
    toggle(GRN_LED);
    lastGRNblink = millis();
    digitalWrite(PIN_GRNLED, GRN_LED);
  }
}
void toggle_REDLED(uint32_t delay_time)
{
  if (millis() - lastREDblink > delay_time)
  {
    toggle(RED_LED);
    lastREDblink = millis();
    digitalWrite(PIN_REDLED, RED_LED);
  }
}
void control_LEDs()
{
  if (GRN_LED)
    toggle_GRNLED(1);
  else
    toggle_GRNLED(50 + (1023 - A_Speed) / 2);
  if (RED_LED)
    toggle_REDLED(1);
  else
    toggle_REDLED(1000 - 800 * walk);
}
void control_motors()
{
  LEFT.write(servo_zero + LEFT_VAL);
  RIGHT.write(servo_zero + RIGHT_VAL);
}

void wait_for_cycle()
{
  while (millis() - last_time < cycle_time)
  {
  }
  last_time = millis();
}
void setup()
{
  Serial.begin(9600);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_LEFT_FWD, INPUT_PULLUP);
  pinMode(PIN_LEFT_BACK, INPUT_PULLUP);
  pinMode(PIN_RIGHT_FWD, INPUT_PULLUP);
  pinMode(PIN_RIGHT_BACK, INPUT_PULLUP);

  pinMode(PIN_REDLED, OUTPUT);
  pinMode(PIN_GRNLED, OUTPUT);
  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);

  LEFT.attach(PIN_LEFT);
  RIGHT.attach(PIN_RIGHT);
}

void loop()
{
  wait_for_cycle();
  readPIN_Inputs();
  LEFT_VAL = servo_zero;
  RIGHT_VAL = servo_zero;
  usr_power = A_Speed / 15;
  if (!SW1 && SW1_prev)
    toggle(walk);
  if (walk)
  {
    LEFT_VAL = int (float(usr_power) * float(A_slider)/512);
    RIGHT_VAL =- int (float(usr_power) * float(A_slider)/512);;
  }
  else
  {
    LEFT_VAL = 0;
    RIGHT_VAL = 0;
  }

  control_motors();
  control_LEDs();
  // print_tele();
}
