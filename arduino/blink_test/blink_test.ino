int val = 0;
String data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello: inside begin");
  pinMode(11, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  data = "";
  while(Serial.available())
  {
    delay(10);
    if (Serial.available() > 0)
    {
      char c = Serial.read();
      data += c;
    }
  }

  if (data.length() > 0)
  {
    Serial.print("Received data of length ");
    Serial.print(data.length());
    Serial.println(" with content "+ data);
    Serial.print("Integer value: ");
    Serial.println(data.toInt());

    analogWrite(11, data.toInt());
    delay(1000);
    analogWrite(11, 0);

  }
  
}
