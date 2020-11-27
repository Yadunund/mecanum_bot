
#define NUM_MOTORS 4
//   ^
//w1---w2
//|     |
//|     |
//w3---w4

int dir_pins[NUM_MOTORS] = {2, 4, 7, 12};
int pwm_pins[NUM_MOTORS] = {3, 5, 6, 11};
int dir_correction[NUM_MOTORS] = {1, -1, 1, -1};
String data;

void setup()
{
  Serial.begin(115200);
  // Set dir and pwm pins as output
  for (int i = 0; i < NUM_MOTORS; i++)
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
  // We expect input data to be in the form
  // "[w1, w2, w3, w4]"
  if (data.length() > 0)
  {
    // Remove the brackets
    data = data.substring(1, data.length() - 1);
    
    int comma_indices[NUM_MOTORS -1] = {0, 0, 0};
    int comma_count = 0;
    int wheel_speeds[NUM_MOTORS] = {0, 0, 0, 0};
    
    for (int i = 0; i < data.length() - 1; i++)
    {
      if (data[i] == ',' && comma_count < 3)
      {
        comma_indices[comma_count] = i;
        comma_count++;
      }
    }

    Serial.print("Comma count: ");
    Serial.println(comma_count);
    for (int i = 0; i < NUM_MOTORS; i++)
    {
      String raw_data = "";
      if (i == 0)
        raw_data = data.substring(0, comma_indices[i]);
        
      else if (i == NUM_MOTORS - 1)
        raw_data = data.substring(comma_indices[i -1] + 1, data.length());

      else
        raw_data = data.substring(comma_indices[i-1] + 1, comma_indices[i]);

      Serial.println(raw_data);
      wheel_speeds[i] = raw_data.toInt();
    }


    for (int i = 0; i < NUM_MOTORS; i++)
    {
      int corrected_val = wheel_speeds[i] * dir_correction[i];
      if (corrected_val < 0)
        digitalWrite(dir_pins[i], 0);
      else
        digitalWrite(dir_pins[i], 1);
      analogWrite(pwm_pins[i], abs(corrected_val));
    }
  }
}
