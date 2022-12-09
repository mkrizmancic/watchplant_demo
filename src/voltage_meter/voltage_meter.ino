const int voltageSensor = 4;
const int totalLeds = 6;
int ledPins[totalLeds] = {16,27,32,33,13,12};

int sensorValue = 0;
int level = 0;

void ledRange(int num, int total) {
    for (int i = 0; i < num; i++) {
        digitalWrite(ledPins[i], HIGH);
    }
    for (int i = num; i < total; i++) {
        digitalWrite(ledPins[i], LOW);
    }
}

void setup()
{
  pinMode(voltageSensor, INPUT);
  for (int i = 0; i <= totalLeds; i++)
  {
      pinMode(ledPins[i], OUTPUT);
  }

  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);

  Serial.begin(115200);
}

void loop()
{
    sensorValue = analogRead(voltageSensor);
    Serial.print(sensorValue);
    Serial.print(" ");

    // level = (level + 1) % (totalLeds+1);

    int level = map(sensorValue, 0, 4095, 1, totalLeds);
    Serial.println(level);

    ledRange(level, totalLeds); 
    
    delay(1000);
}
