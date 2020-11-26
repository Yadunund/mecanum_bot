
#define NUM_PINS 4
//   ^
//w1---w2
//|     |
//|     |
//w3---w4

int dir_pins[NUM_PINS] = {2, 4, 7, 12};
int pwm_pins[NUM_PINS] = {3, 5, 6, 11};
int dir_correction[NUM_PINS] = {1, -1, 1, -1};
String data;

void setup()
{
  Serial.begin(115200);
  // Set dir and pwm pins as output
  for (int i = 0; i < NUM_PINS; i++)
  {
    pinMode(dir_pins[i], OUTPUT);
    pinMode(pwm_pins[i], OUTPUT);

    digitalWrite(pwm_pins[i], 0);
  }
}

void loop()
{
  data = "";
  while(Serial.available())
  {
    delay(1);
    if (Serial.available() > 0)
    {
      char c = Serial.read();
      data += c;
    }
  }

  if (data.length() > 0)
  {
    for (int i=0;i<data.length(); i++)
    {
      Serial.print("Char:");
      Serial.println(data[i]);
    }
    int val = data.toInt();
    Serial.print("Received data of length ");
    Serial.print(data.length());
    Serial.println(" with content "+ data);
    Serial.print("Integer value: ");
    Serial.println(val);

    for (int i = 0; i < NUM_PINS; i++)
    {
      int corrected_val = val * dir_correction[i];
      if (corrected_val < 0)
        digitalWrite(dir_pins[i], 0);
      else
        digitalWrite(dir_pins[i], 1);
      analogWrite(pwm_pins[i], abs(val));
    }
  }
}
