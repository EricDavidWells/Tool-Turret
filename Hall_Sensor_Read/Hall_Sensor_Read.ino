#define HALLPIN1 A2
#define HALLPIN2 A4

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
int a = analogRead(HALLPIN1);
int b = analogRead(HALLPIN2);

Serial.print("hall sensor 1 value: ");Serial.print(a);
Serial.print(" hall sensor 2 value: ");Serial.println(b);
delay(100);
}
