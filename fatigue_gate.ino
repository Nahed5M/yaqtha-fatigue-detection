

int ledGreen = 9;   
int ledRed   = 5;   
int buzzer   = 10;  

void setup() {
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Serial.begin(9600);


  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, LOW);
  digitalWrite(buzzer, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'G') {

      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, LOW);
      noTone(buzzer);
    }

    else if (cmd == 'R') {

      digitalWrite(ledGreen, LOW);
      digitalWrite(ledRed, HIGH);

      tone(buzzer, 1000);
      delay(300);
      noTone(buzzer);
    }
  }
}
