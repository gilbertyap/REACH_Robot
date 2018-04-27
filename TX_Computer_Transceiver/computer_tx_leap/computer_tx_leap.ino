// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem
// configuration

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Where to send packets to!
#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     2

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup()
{
  Serial.begin(115200);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = "ReGameVrLab";
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

// Dont put this on the stack:
char serial_packet[10];
char old_serial_packet[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  // Number of bytes in the packet
  int packet_length = 10;

  // Check for
  // Serial.println(Serial.peek());
  if (Serial.peek() == 240) {
    for (int i = 0; i < packet_length; i++) {
      serial_packet[i] = Serial.read();
      Serial.println(serial_packet[i]);
    }
  }

  // Make it so that a message is sent ever 10 ms
  if (millis() % 10 != 0) {
    if (!samePackets(old_serial_packet, serial_packet)) {

      Serial.print("Sending "); Serial.println(serial_packet);

      // Send a message to the DESTINATION!
      if (rf69_manager.sendtoWait((uint8_t *)serial_packet, strlen(serial_packet), DEST_ADDRESS)) {
        // Now wait for a reply from the server
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
          buf[len] = 0; // zero out remaining string

          Serial.print("Got reply from #"); Serial.print(from);
          Serial.print(" [RSSI :");
          Serial.print(rf69.lastRssi());
          Serial.print("] : ");
          Serial.println((char*)buf);

          // Give old_serial_packet the value of the newest packet
          for (int i = 0; i < packet_length; i++) {
            old_serial_packet[i] = serial_packet[i];
          }

        } else {
          Serial.println("No reply, is anyone listening?");
        }
      } else {
        Serial.println("Sending failed (no ack)");
      }
    }
  } else {
    delay(2);
  }
}

bool samePackets(char old_packet[], char new_packet[]) {
  if ((old_packet[2] == new_packet[2]) && (old_packet[4] == new_packet[4]) &&
      (old_packet[6] == new_packet[6]) && (old_packet[8] == new_packet[8])) {
    return true;
  } else {
    return false;
  }
}

