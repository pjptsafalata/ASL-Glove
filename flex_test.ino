
int flex=A0;
int data=0;

void setup() {
 Serial.begin(9600);
 pinMode(flex,INPUT);

}

void loop() {
  data=analogRead(flex);
  Serial.println(data);
  delay(1000);

}
