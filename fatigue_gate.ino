//تكامل نظام كشف الإرهاق مع بوابة المصنع (نموذج مبدئي)
// Arduino جزء الإشارات البصرية والصوتية

int ledGreen = 8;   // LED أخضر - سائق سليم
int ledRed   = 9;   // LED أحمر - سائق مرهق
int buzzer   = 10;  // جرس إنذار

void setup() {
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(buzzer, OUTPUT);

  //تشغيل السيريال لاستقبال الأوامر من البايثون
  Serial.begin(9600);

  // افتراضياً: سائق سليم
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, LOW);
  digitalWrite(buzzer, LOW);
}

void loop() {
  //إذا وصل أمر من البايثون
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'G') {
      // G = Good / Green (سائق سليم)
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, LOW);
      digitalWrite(buzzer, LOW);
    }

    if (cmd == 'R') {
      // R = Risk / Red (سائق مرهق)
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledRed, HIGH);

      //نبضة إنذار صوتي بسيطة
      tone(buzzer, 1000);   // 1000 Hz
      delay(300);
      noTone(buzzer);
    }
  }
}
