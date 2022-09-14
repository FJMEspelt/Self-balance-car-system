int motorW1pin1 = 2;
int motorW1pin2 = 3;

int motorW2pin1 = 5;
int motorW2pin2 = 4;

int motorS1pin1 = 6;
int motorS1pin2 = 7;

int motorS2pin1 = 8;
int motorS2pin2 = 9;

int motorS3pin1 = 10;
int motorS31pin2 = 11;

int motorS4pin1 = 12;
int motorS4pin2 = 13;


void setup() {
  // put your setup code here, to run once:
  pinMode(motorW1pin1, OUTPUT);
  pinMode(motorW1pin2, OUTPUT);
  pinMode(motorW2pin1, OUTPUT);
  pinMode(motorW2pin2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   
  digitalWrite(motorW1pin1, HIGH);
  digitalWrite(motorW1pin2, LOW);

  digitalWrite(motorW2pin1, HIGH);
  digitalWrite(motorW2pin2, LOW);
  delay(5000);

}
