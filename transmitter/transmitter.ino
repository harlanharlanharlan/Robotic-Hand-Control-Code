#include <SPI.h>
#include <RH_RF69.h>

#define RF69_FREQ 915.0
#define RFM69_CS    4
#define RFM69_INT   3
#define RFM69_RST   2

RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;
char txt[] = {72, 73, 74, 74, 0 };
int x;

void setup() {
  Serial.begin(115200);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

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
}

void loop() {
  for (x = 0; x < 5; x++) {
    txt[x] = analogRead(x);
//    if (int y >= 200; y = 0; y++) {
//      num = map(y, 200, 180, 0, 180);
//    }
  }
  rf69.send((uint8_t *) txt, sizeof(txt));
  rf69.waitPacketSent();
  delay(100);
}
