#define BAUD_RATE 115200
#define HEADER_LENGTH 2
#define DATA_LENGTH 2
#include <avr/io.h>
#include <util/delay.h>

#define CHA_PWM 5
#define CHA_LPWM 7
#define CHA_RPWM 8

#define CHB_PWM 6
#define CHB_LPWM 9
#define CHB_RPWM 10
#define VBAT A0

long last_update = 0;
//                      A    B
int8_t new_values[2] = {10, 10};
int8_t old_values[2] = {10, 10};
int8_t act_values[2] = {10, 10};

void pwm_tca0_dual_init(void)
{
  // Set PA0 and PA1 as output (WO0 and WO1)
  // PA0=D2
  // PA1=D7
  PORTA.DIRSET = PIN0_bm | PIN1_bm;

  // Set single-slope PWM, enable WO0 and WO1
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm | // Enable WO0
                      TCA_SINGLE_CMP1EN_bm;                                     // Enable WO1
  TCA0.SINGLE.PER = 127;                                                        // CNT TOP VALUE
  TCA0.SINGLE.CMP0 = 0;                                                         // PWM VALUE
  TCA0.SINGLE.CMP1 = 0;                                                         // PWM VALUE
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

void setup()
{
  // pwm_tca0_dual_init();
  Serial.begin(BAUD_RATE);
  Serial.println("hello");
  pinMode(CHA_PWM, OUTPUT);
  pinMode(CHA_LPWM, OUTPUT);
  pinMode(CHA_RPWM, OUTPUT);
  pinMode(CHB_PWM, OUTPUT);
  pinMode(CHB_LPWM, OUTPUT);
  pinMode(CHB_RPWM, OUTPUT);
  pinMode(A0, INPUT);
}

int slew_rate_limit(int new_value, int old_value, int slew_limit)
{
  int delta = new_value - old_value;

  if (delta > slew_limit)
  {
    delta = slew_limit;
  }
  if (delta < -slew_limit)
  {
    delta = -slew_limit;
  }
  old_value+=delta;
  return old_value;
}

long last_bat_update = 0;

void loop()
{


  // read_packet(values);
  if (Serial.available() > 1)
  {
    new_values[0] = Serial.read();
    new_values[1] = Serial.read();

    act_values[0]=slew_rate_limit(new_values[0], old_values[0], 10);
    act_values[1]=slew_rate_limit(new_values[1], old_values[1], 10);

    old_values[0]=act_values[0];
    old_values[1]=act_values[1];

    last_update = millis();
  }

  if (millis() - last_update < 1000)
  {
    digitalWrite(CHA_LPWM, act_values[0] > 0);
    digitalWrite(CHA_RPWM, !(act_values[0] > 0));
    analogWrite(CHA_PWM, abs(act_values[0]) * 2);

    digitalWrite(CHB_LPWM, act_values[1] > 0);
    digitalWrite(CHB_RPWM, !(act_values[1] > 0));
    analogWrite(CHB_PWM, abs(act_values[1]) * 2);
  }
  else
  {
    digitalWrite(CHA_LPWM, 0);
    digitalWrite(CHA_RPWM, 0);
    digitalWrite(CHB_LPWM, 0);
    digitalWrite(CHB_RPWM, 0);
    analogWrite(CHA_PWM, 0);
    analogWrite(CHB_PWM, 0);
  }

  if (millis() - last_bat_update > 20)
  {
    last_bat_update = millis();
    float vbat = (analogRead(VBAT) * 5.0 * 3.0) / 100.0;
    uint8_t vbat_byte = vbat;
    Serial.write(vbat_byte);
  }
}
