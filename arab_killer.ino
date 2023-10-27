#include <Servo.h>

#define PWM_INPUT_CYCLE_TIME_US (20000.0)
#define PWM_INPUT_LOW_US (1000.0)
#define PWM_INPUT_HIGH_US (2000.0)

#define SERVO_MIN (0)
#define SERVO_MID (90)
#define SERVO_MAX (180)

#define SERVO_CLOCKWISE (79)
#define SERVO_COUNTERCLOCKWISE (107)
#define SERVO_STOP (93)

#define RELOAD_DELAY (1550)

#define MIN_AMMO (1)
#define MAX_AMMO (3)

typedef enum pwm {
  PWM_IN_0 = 0,
  PWM_IN_1,
  PWM_IN_2,  
  PWM_OUT_3,
  PWM_IN_4,  
  PWM_OUT_5,
  PWM_OUT_6,
  PWM_IN_7,  
  PWM_IN_8,  
  PWM_OUT_9,
  PWM_OUT_10,
  PWM_OUT_11,
  // PWM_OUT_12,
  PWM_IN_13 = 13,

  PWM_IN_CHANNEL_1 = PWM_IN_2,
  PWM_IN_CHANNEL_2 = PWM_IN_4,
  PWM_IN_CHANNEL_3 = PWM_IN_7,
  PWM_IN_CHANNEL_4 = PWM_IN_8,
  // PWM_IN_CHANNEL_5 = PWM_IN_13,

  PWM_IN_ARM = PWM_IN_CHANNEL_1,
  PWM_IN_FIRE = PWM_IN_CHANNEL_2,
  PWM_IN_RELOAD = PWM_IN_CHANNEL_3,
  PWM_IN_RESET = PWM_IN_CHANNEL_4,

  PWM_OUT_ARM = PWM_OUT_3,
  PWM_OUT_FIRE = PWM_OUT_5,
  PWM_OUT_RELOAD = PWM_OUT_6,

  DIGITAL_IN_SENSOR = 12,
}pwm_t;

double get_pwm_value(int pin)
{
  return pulseIn(pin, HIGH) / PWM_INPUT_CYCLE_TIME_US;
}

bool get_pwm_state(double pulse_width)
{
  double threshhold = (PWM_INPUT_HIGH_US/PWM_INPUT_CYCLE_TIME_US + PWM_INPUT_LOW_US/PWM_INPUT_CYCLE_TIME_US) / 2.0;
  return pulse_width > threshhold;
}

int get_pwm(bool pwm_state)
{
  return pwm_state ? 255 : 0;
}

Servo arm_servo;
Servo fire_servo;
Servo reload_servo;
int ammo_place = MIN_AMMO;

void arm_system(bool do_arm)
{
  Serial.write("arm state: ");
  if (do_arm)
  {
    arm_servo.write(SERVO_MAX);
    Serial.println("ARMED!");
  }
  else
  {
    arm_servo.write(SERVO_MIN);
    Serial.println("not armed");
  }
}

void fire_system(bool do_fire)
{
  Serial.print("fire state: ");

  if (do_fire)
  {
    if (arm_servo.read() != SERVO_MAX)
    {
      Serial.println("cant fire, arm not in armed mode");
      return;
    }
    fire_servo.write(SERVO_MAX);
    Serial.println("FIRE!!!");
  }
  else
  {
    if (arm_servo.read() != SERVO_MIN)
    {
      Serial.println("cant fire, arm not in reset mode");
      return;
    }
    fire_servo.write(SERVO_MID);
    Serial.println("hold fire");
  }
}

bool rotate_ammo(int servo_power)
{
  Serial.print("rotate ammo: ");

  if ((ammo_place == 1 && servo_power == SERVO_COUNTERCLOCKWISE) || (ammo_place == 3 && servo_power == SERVO_CLOCKWISE))
  {
    Serial.println("can't rotate, extreme position");
    return false;
  }

  if (arm_servo.read() != SERVO_MIN || fire_servo.read() != SERVO_MID)
  {
    Serial.println("arm or fire not in reset mode");
    return false;
  }

  if (servo_power == SERVO_CLOCKWISE)
  {
    ammo_place++;
  }
  else if (servo_power == SERVO_COUNTERCLOCKWISE)
  {
    ammo_place--;
  }
  else
  {
    Serial.println("can't rotate, bad servo power");
    return false;
  }

  reload_servo.attach(PWM_OUT_RELOAD);

  reload_servo.write(servo_power);
  // while (digitalRead(DIGITAL_IN_SENSOR) == LOW) {}
  delay(RELOAD_DELAY);
  // while (digitalRead(DIGITAL_IN_SENSOR) == HIGH) {}
  reload_servo.write(SERVO_STOP);
  delay(500);
  reload_servo.detach();

  return true;
}

void reload_system(bool do_reload)
{
  Serial.print("reload system: ");
  if (do_reload && ammo_place < MAX_AMMO)
  {
    Serial.println("reloading");

    if (!rotate_ammo(SERVO_CLOCKWISE))
    {
      return;
    }

    Serial.println("did reload!");
    while (get_pwm_state(get_pwm_value(PWM_IN_RELOAD)) == HIGH) {}
  }
  else if (do_reload && ammo_place == MAX_AMMO)
  {
    Serial.println("can't reload");
  }
  else
  {
    Serial.println("no reload");
  }
  reload_servo.detach();
}

void reset_system(bool do_reset)
{
  Serial.print("reset system: ");
  if (!do_reset)
  {
    Serial.println("no reset");
    return;
  }

  if (ammo_place == MIN_AMMO)
  {
    Serial.println("can't reset");
    return;
  }
  else
  {
    while (ammo_place != MIN_AMMO)
    {
      if (!rotate_ammo(SERVO_COUNTERCLOCKWISE))
      {
        return;
      }
      Serial.println("resetting");
    }
  }
}

void setup() {
  Serial.begin(9600);

  arm_servo.attach(PWM_OUT_ARM);
  fire_servo.attach(PWM_OUT_FIRE);
  // reload_servo.attach(PWM_OUT_RELOAD);

  pinMode(DIGITAL_IN_SENSOR, INPUT_PULLUP);

  arm_system(false);
}

void loop() {
  int pwm_arm = get_pwm(get_pwm_state(get_pwm_value(PWM_IN_ARM)));
  int pwm_fire = get_pwm(get_pwm_state(get_pwm_value(PWM_IN_FIRE)));
  int pwm_reload = get_pwm(get_pwm_state(get_pwm_value(PWM_IN_RELOAD)));
  int pwm_reset = get_pwm(get_pwm_state(get_pwm_value(PWM_IN_RESET)));

  Serial.print("pwm_arm: ");
  Serial.print(pwm_arm);

  Serial.print("\tpwm_fire: ");
  Serial.print(pwm_fire);
  
  Serial.print("\tpwm_reload: ");
  Serial.print(pwm_reload);

  Serial.print("\t pwm_reset: ");
  Serial.println(pwm_reset);

  arm_system(get_pwm_state(get_pwm_value(PWM_IN_ARM)));
  fire_system(get_pwm_state(get_pwm_value(PWM_IN_FIRE)));
  reload_system(get_pwm_state(get_pwm_value(PWM_IN_RELOAD)));
  reset_system(get_pwm_state(get_pwm_value(PWM_IN_RESET)));

  Serial.print("current ammo number: ");
  Serial.println(ammo_place);
}