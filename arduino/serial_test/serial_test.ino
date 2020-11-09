#define led1 8
#define led2 9
#define potential A0


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(potential,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    String data = Serial.readStringUntil('\n');
    if (data=="led1"){
      digitalWrite(led1,HIGH);
      digitalWrite(led2,LOW);
    } else {
      digitalWrite(led2,HIGH);
      digitalWrite(led1,LOW);
    }
    int value = analogRead(potential);
    Serial.println(value);
    delay(10);
  }
}
