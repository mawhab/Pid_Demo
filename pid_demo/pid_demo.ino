#define BAUD_RATE 115200
#define CPR 540

// Encoder pins
const int PIN_ENCODER_M[] = {PA6, PA6}; // B15, B14     a6, a7      a, b
const int PIN_ENCODER_P[] = {PA7, PA15, PA15};
const int PIN_ENCODER_I[] = {PA7, PA15, PA15};
const int PIN_ENCODER_D[] = {PA7, PA15, PA15};
// Motor pins
const int PIN_MOTOR[] = {PA3, PB1}; // b3, b9     a3, a2      dir, pwm

// Define TTL pins
#define PIN_UART3_RX PB11
#define PIN_UART3_TX PB10
HardwareSerial Serial3(PIN_UART3_RX, PIN_UART3_TX);

long long pwm = 0, counter = 0;
long long counterp = 0, counteri = 0, counterd = 0;
double setpoint = 0;
long long prev_time, prev = 0;

float Kp;
float Ki;
float Kd;

double p,i=0,d;


void encoderISR_MA(void)
{
  counter += digitalRead(PIN_ENCODER_M[0]) == digitalRead(PIN_ENCODER_M[1]) ? -1 : 1;
}
void encoderISR_MB(void)
{
  counter += digitalRead(PIN_ENCODER_M[0]) != digitalRead(PIN_ENCODER_M[1]) ? -1 : 1;
}

void encoderISR_AP(void)
{
  counterp += digitalRead(PIN_ENCODER_P[0]) != digitalRead(PIN_ENCODER_P[1]) ? -1 : 1;
}
void encoderISR_BP(void)
{
  counterp += digitalRead(PIN_ENCODER_P[0]) == digitalRead(PIN_ENCODER_P[1]) ? -1 : 1;
}

void encoderISR_AI(void)
{
  counteri += digitalRead(PIN_ENCODER_I[0]) != digitalRead(PIN_ENCODER_I[1]) ? -1 : 1;
}
void encoderISR_BI(void)
{
  counteri += digitalRead(PIN_ENCODER_I[0]) == digitalRead(PIN_ENCODER_I[1]) ? -1 : 1;
}

void encoderISR_AD(void)
{
  counterd += digitalRead(PIN_ENCODER_D[0]) != digitalRead(PIN_ENCODER_D[1]) ? -1 : 1;
}
void encoderISR_BD(void)
{
  counterd += digitalRead(PIN_ENCODER_D[0]) == digitalRead(PIN_ENCODER_D[1]) ? -1 : 1;
}

void reset_P(void){
  counterp = 0;
}

void reset_I(void){
  counteri = 0;
}

void reset_D(void){
  counterd = 0;
}

void initEncoders()
{
  pinMode(PIN_ENCODER_M[0], INPUT_PULLUP);
  pinMode(PIN_ENCODER_M[0], INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_M[0], encoderISR_MA, CHANGE);
  attachInterrupt(PIN_ENCODER_M[0], encoderISR_MB, CHANGE);

  pinMode(PIN_ENCODER_P[0], INPUT_PULLUP);
  pinMode(PIN_ENCODER_P[1], INPUT_PULLUP);
  pinMode(PIN_ENCODER_P[2], INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_P[0], encoderISR_AP, CHANGE);
  attachInterrupt(PIN_ENCODER_P[1], encoderISR_BP, CHANGE);
  attachInterrupt(PIN_ENCODER_P[2], reset_P, CHANGE);

  pinMode(PIN_ENCODER_I[0], INPUT_PULLUP);
  pinMode(PIN_ENCODER_I[1], INPUT_PULLUP);
  pinMode(PIN_ENCODER_I[2], INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_I[0], encoderISR_AI, CHANGE);
  attachInterrupt(PIN_ENCODER_I[1], encoderISR_BI, CHANGE);
  attachInterrupt(PIN_ENCODER_I[2], reset_I, CHANGE);

  pinMode(PIN_ENCODER_D[0], INPUT_PULLUP);
  pinMode(PIN_ENCODER_D[1], INPUT_PULLUP);
  pinMode(PIN_ENCODER_D[2], INPUT_PULLUP);
  attachInterrupt(PIN_ENCODER_D[0], encoderISR_AD, CHANGE);
  attachInterrupt(PIN_ENCODER_D[1], encoderISR_BD, CHANGE);
  attachInterrupt(PIN_ENCODER_D[2], reset_D, CHANGE);
}

void controlAction(void)
{
    // motor
    if (pwm > 0)
        digitalWrite(PIN_MOTOR[0], HIGH);
    else if (pwm < 0)
        digitalWrite(PIN_MOTOR[0], LOW);
    analogWrite(PIN_MOTOR[1], abs(pwm));
}

void initMotorDrivers()
{
  pinMode(PIN_MOTOR[0], OUTPUT);
  pinMode(PIN_MOTOR[1], OUTPUT);
  digitalWrite(PIN_MOTOR[0], LOW);
  digitalWrite(PIN_MOTOR[1], LOW);
}

void debug()
{
    Serial1.print("PWM: ");
    Serial1.print(pwm);
    Serial1.print(", ");
    Serial1.print("Counts: ");
    Serial1.println(counter);
    Serial1.print("P counts: ");
    Serial1.println(counterp);
    Serial1.print("I counts: ");
    Serial1.println(counteri);
    Serial1.print("D counts: ");
    Serial1.println(counterd);
}

void pid(double sp){
  double dt = (millis() - prev) / 1000;
  double error = sp - ((counter / 540) / dt);
  p = error * Kp;
  i += error * Ki * dt;
  d = error * Kd / dt;

  pwm = p + i + d;
  
  controlAction();
}

void setup() {
  Serial1.begin(BAUD_RATE);
  Serial3.begin(BAUD_RATE);

  // PWM setup
  analogWriteFrequency(2000);
  analogWriteResolution(16);

  initEncoders();
  initMotorDrivers();
  
  //  Start scheduler and LED indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  prev_time = millis();
}

void loop() {
  if(Serial1.available())
    {
        setpoint = Serial1.readString().toInt();    // Parse an Integer from Serial
        i = 0;
    }
    if(millis() - prev_time >= 15){
      pid(setpoint);
      prev_time = millis();
    }
    debug();

}
