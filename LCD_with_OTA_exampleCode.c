#include <Update.h>
#include <TFT_eSPI.h>

TFT_eSPI tft;

#define HELLO       "AAA"
#define HELLO_ACK   "BBB"
#define OTA_START   "SSS"
#define OTA_OK      "KKK"
#define CHUNK_ACK   "ACK"
#define CHUNK_NACK  "NAK"
#define RESEND_REQ  "RSN"

uint32_t totalSize = 0;
uint16_t totalChunks = 0;
uint32_t receivedBytes = 0;
uint32_t lastDraw = 0;

uint8_t calculateChunkCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

void waitForToken(const char* expected) {
  size_t len = strlen(expected);
  char buf[8] = {0};
  size_t idx = 0;

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') continue;
      
      buf[idx++] = c;
      if (idx == len) {
        if (memcmp(buf, expected, len) == 0) return;
        memmove(buf, buf + 1, len - 1);
        idx = len - 1;
      }
    }


  }
}

void setup() {
  delay(2000);
  
  Serial.begin(115200, SERIAL_8E1);
  Serial.setRxBufferSize(256);
  Serial.setRxBufferSize(8192); 
  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  
  // Step 1: Wait for HELLO from ESP32 (ESP32 initiates)
  tft.setCursor(10, 50);
  tft.print("1. Waiting HELLO...");
  waitForToken(HELLO);
  
  tft.fillRect(0, 50, 320, 30, TFT_BLACK);
  tft.setCursor(10, 50);
  tft.print("1. HELLO Received!");
  delay(200);
  
  //2: BBB
  tft.setCursor(10, 80);
  tft.print("2. Sending ACK...");
  Serial.write(HELLO_ACK);
  Serial.flush();
  delay(100);
  
  //3. SSS
  tft.setCursor(10, 110);
  tft.print("3. Waiting START...");
  waitForToken(OTA_START);
  
  tft.fillRect(0, 110, 320, 30, TFT_BLACK);
  tft.setCursor(10, 110);
  tft.print("3. START Received!");
  delay(100);
  
  //4. KKK
  tft.setCursor(10, 140);
  tft.print("4. Sending OK...");
  Serial.write(OTA_OK);
  Serial.flush();
  delay(100);
  
  // Step 5: Receive SIZE (4 bytes) and TOTAL CHUNKS (2 bytes)
  tft.fillScreen(TFT_BLUE);
  tft.setCursor(10, 50);
  tft.print("5. Receiving SIZE...");
  
  uint8_t sizeBytes[4];
  while (Serial.available() < 4) delay(10);
  Serial.readBytes(sizeBytes, 4);
  
  totalSize = (uint32_t)sizeBytes[0] | 
              ((uint32_t)sizeBytes[1] << 8) | 
              ((uint32_t)sizeBytes[2] << 16) | 
              ((uint32_t)sizeBytes[3] << 24);
  
  // Receive total chunks
  uint8_t chunkBytes[2];
  while (Serial.available() < 2) delay(10);
  Serial.readBytes(chunkBytes, 2);
  
  totalChunks = (uint16_t)chunkBytes[0] | ((uint16_t)chunkBytes[1] << 8);
  
  tft.setCursor(10, 80);
  tft.print("SIZE: ");
  tft.print(totalSize);
  tft.print(" bytes");
  tft.setCursor(10, 110);
  tft.print("CHUNKS: ");
  tft.print(totalChunks);
  delay(5000);

  Serial.write(OTA_OK);
  Serial.flush();
  delay(100);

  // Step 6: Start Update
  tft.fillScreen(TFT_CYAN);
  tft.setCursor(10, 50);
  tft.print("6. Starting Update...");
  
  if (!Update.begin(totalSize)) {
    tft.fillScreen(TFT_RED);
    tft.setCursor(10, 100);
    tft.print("UPDATE BEGIN FAILED!");
    while(1) delay(1000);
  }

  receivedBytes = 0;
  lastDraw = millis();
  uint16_t expectedChunkId = 1;
  const size_t CHUNK_SIZE = 128;
  uint8_t buffer[CHUNK_SIZE + 3];  // [ChunkID_Low][ChunkID_High][Data...][CRC]
  uint16_t successfulChunks = 0;
  uint16_t failedChunks = 0;
  uint16_t retries = 0;


  // Chunk loop
  while (successfulChunks < totalChunks) {
    size_t remaining = totalSize - receivedBytes;
    size_t expectedDataSize = min((size_t)CHUNK_SIZE, remaining);
    size_t expectedBytes = expectedDataSize + 3;
    
    // Wait for chunk with timeout
    uint32_t chunkStart = millis();
    while (Serial.available() < expectedBytes) {
      if (millis() - chunkStart > 15000) {
        tft.fillScreen(TFT_RED);
        tft.setTextSize(2);
        tft.setCursor(10, 80);
        tft.print("CHUNK TIMEOUT!");
        tft.setCursor(10, 110);
        tft.print("ID: ");
        tft.print(expectedChunkId);
        tft.setCursor(10, 140);
        tft.print("Avail: ");
        tft.print(Serial.available());
        tft.print("/");
        tft.print(expectedBytes);
        while(1);
      }
      yield();
    }
    
    // Read chunk
    int bytesRead = Serial.readBytes(buffer, expectedBytes);
    
    if (bytesRead != expectedBytes) {
      Serial.write(CHUNK_NACK);
      Serial.flush();
      delay(100);
while (Serial.available()) Serial.read();
      retries++;
      continue;
    }
    
    uint16_t rxChunkId = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    uint8_t rxCRC = buffer[bytesRead - 1];
    uint8_t calcCRC = calculateChunkCRC8(buffer, bytesRead - 1);
    
    // CRC check
    if (calcCRC != rxCRC) {
      Serial.write(CHUNK_NACK);
      Serial.flush();
      delay(100);
      while (Serial.available()) Serial.read();
      retries++;
      continue;
    }
    
    // ID check
    if (rxChunkId != expectedChunkId) {
      uint8_t req[5];
      memcpy(req, RESEND_REQ, 3);
      req[3] = (uint8_t)(expectedChunkId & 0xFF);
      req[4] = (uint8_t)((expectedChunkId >> 8) & 0xFF);
      Serial.write(req, 5);
      Serial.flush();
      delay(100);
      while (Serial.available()) Serial.read();
      retries++;
      continue;
    }
    
    // Write to flash
    size_t written = Update.write(buffer + 2, expectedDataSize);
    if (written != expectedDataSize) {
      Serial.write(CHUNK_NACK);
      Serial.flush();
      delay(100);
      while (Serial.available()) Serial.read();
      retries++;
      continue;
    }
    
    receivedBytes += expectedDataSize;
    successfulChunks++;
    expectedChunkId++;
    
    // Send ACK
    Serial.write(CHUNK_ACK);
    Serial.flush();
    
    // Update display every 5 chunks (less frequent to avoid blocking)
    if (successfulChunks % 5 == 0 || successfulChunks == totalChunks) {
      uint32_t percent = (successfulChunks * 100) / totalChunks;
      tft.fillRect(0, 60, 320, 180, TFT_BLACK);
      
      tft.setTextSize(6);
      tft.setCursor(80, 90);
      tft.print(percent);
      tft.print("%");
      
      tft.setTextSize(2);
      tft.setCursor(10, 170);
      tft.print(successfulChunks);
      tft.print("/");
      tft.print(totalChunks);
      
      tft.setCursor(10, 200);
      tft.print("Retry: ");
      tft.print(retries);
    }
  }



  // Step 8: Finish Update
  tft.fillScreen(TFT_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(10, 100);
  tft.print("FINISHING...");
  
  if (Update.end(true)) {
    tft.fillScreen(TFT_GREEN);
    tft.setTextSize(4);
    tft.setCursor(50, 100);
    tft.print("SUCCESS!");
    tft.setTextSize(2);
    tft.setCursor(10, 150);
    tft.print("Total: ");
    tft.print(successfulChunks);
    tft.print(" chunks");
    // tft.setCursor(10, 180);
    // tft.print("Retries: ");
    // tft.print(failedChunks);
    tft.setCursor(10, 180);
    tft.print("Retries: ");
    tft.print(retries);

    delay(3000);
    ESP.restart();
  } else {
    tft.fillScreen(TFT_RED);
    tft.setTextSize(3);
    tft.setCursor(60, 100);
    tft.print("FAILED!");
    while(1) delay(1000);
  }
}

void loop() {
  // Empty
}