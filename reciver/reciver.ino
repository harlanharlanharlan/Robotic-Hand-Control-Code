#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define RFM69_CS    4
#define RFM69_INT   3
#define RFM69_RST   2
#define SERVOMIN  150
#define SERVOMAX  600
#define USMIN  600
#define USMAX  2400
#define SERVO_FREQ 50

RH_RF69 rf69(RFM69_CS, RFM69_INT);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int16_t packetnum = 0;
uint8_t servonum = 0;

void setup() {

  /* SERIAL INITIALIZE */
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  /* RF INITIALIZE */
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  Serial.println("Feather RFM69 RX Test!");
  Serial.println();
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };
  rf69.setEncryptionKey(key);
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  pinMode(RFM69_RST, OUTPUT);

  /* PWM INITIALIZE */
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Serial.println("PWM initialized");
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;
  pulselength /= SERVO_FREQ;
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  if (rf69.available()) {
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      for (int x = 0; x < 5; x++) {
        int num = (buf[x]);
        if (num > 195) num = 0;
        if (num < 20) num = 0;
        Serial.print (num);
        Serial.print (";");
        num = map(num, 0, 180, 1000, 2000);
        pwm.writeMicroseconds(x, num);
        delay (1);
      }
      Serial.println();
    }
  }
}
