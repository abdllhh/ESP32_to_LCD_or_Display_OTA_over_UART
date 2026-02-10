#include "esp32_lcd_ota_uart.h"

ESP32_LCD_OTA_UART::ESP32_LCD_OTA_UART() 
    : firmwareBuffer(nullptr),
      firmwareSize(0),
      otaInProgress(false),
      caCert(nullptr),
      insecureMode(false) {
    
    // Initialize statistics
    memset(&stats, 0, sizeof(ota_stats_t));
    
    // Configure default UART settings
    uartConfig.baud_rate = OTA_UART_BAUD_RATE;
    uartConfig.data_bits = UART_DATA_8_BITS;
    uartConfig.parity = UART_PARITY_EVEN;
    uartConfig.stop_bits = UART_STOP_BITS_1;
    uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uartConfig.source_clk = UART_SCLK_APB;
}

ESP32_LCD_OTA_UART::~ESP32_LCD_OTA_UART() {
    cleanup();
}

void ESP32_LCD_OTA_UART::cleanup() {
    if (firmwareBuffer != nullptr) {
        free(firmwareBuffer);
        firmwareBuffer = nullptr;
    }
    firmwareSize = 0;
    otaInProgress = false;
}

void ESP32_LCD_OTA_UART::setCACertificate(const char* ca_cert) {
    caCert = ca_cert;
}

void ESP32_LCD_OTA_UART::setInsecure(bool insecure) {
    insecureMode = insecure;
}

bool ESP32_LCD_OTA_UART::isOTAInProgress() const {
    return otaInProgress;
}

const ota_stats_t& ESP32_LCD_OTA_UART::getStats() const {
    return stats;
}

bool ESP32_LCD_OTA_UART::downloadFirmware(const char* url, const char* username, const char* password) {
    if (url == nullptr) {
        Serial.println("[OTA] Error: URL is null");
        return false;
    }
    
    HTTPClient http;
    WiFiClientSecure client;
    
    // Configure SSL/TLS
    if (insecureMode) {
        client.setInsecure();
    } else if (caCert != nullptr) {
        client.setCACert(caCert);
    } else {
        Serial.println("[OTA] Warning: No CA certificate set. Consider calling setCACertificate() or setInsecure()");
    }
    
    // Begin HTTP connection
    if (!http.begin(client, url)) {
        Serial.println("[OTA] Error: Failed to begin HTTP connection");
        return false;
    }
    
    // Set authentication if provided
    if (username != nullptr && password != nullptr) {
        http.setAuthorization(username, password);
    }
    
    // Perform GET request
    int httpCode = http.GET();
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[OTA] HTTP GET failed with code: %d\n", httpCode);
        http.end();
        return false;
    }
    
    // Get firmware size
    firmwareSize = http.getSize();
    if (firmwareSize <= 0) {
        Serial.printf("[OTA] Invalid firmware size: %d\n", firmwareSize);
        http.end();
        return false;
    }
    
    Serial.printf("[OTA] Firmware size: %d bytes\n", firmwareSize);
    
    // Free old buffer if exists
    if (firmwareBuffer != nullptr) {
        free(firmwareBuffer);
    }
    
    // Allocate PSRAM buffer
    firmwareBuffer = (uint8_t*)ps_malloc(firmwareSize);
    if (firmwareBuffer == nullptr) {
        Serial.printf("[OTA] Failed to allocate %d bytes for firmware buffer\n", firmwareSize);
        http.end();
        return false;
    }
    
    // Download firmware
    uint8_t buff[1280];
    WiFiClient* stream = http.getStreamPtr();
    
    int offset = 0;
    while (http.connected() && offset < firmwareSize) {
        size_t sizeAvail = stream->available();
        if (sizeAvail > 0) {
            size_t bytes_to_read = min(sizeAvail, sizeof(buff));
            size_t bytes_read = stream->readBytes(buff, bytes_to_read);
            memcpy(firmwareBuffer + offset, buff, bytes_read);
            offset += bytes_read;
            
            // Print progress every 10KB
            if (offset % 10240 == 0 || offset == firmwareSize) {
                Serial.printf("[OTA] Downloaded: %d/%d bytes (%d%%)\n", 
                             offset, firmwareSize, (offset * 100) / firmwareSize);
            }
        }
    }
    
    http.end();
    
    if (offset == firmwareSize) {
        Serial.printf("[OTA] Firmware downloaded successfully: %d bytes\n", offset);
        return true;
    } else {
        Serial.printf("[OTA] Firmware download incomplete: %d/%d bytes\n", offset, firmwareSize);
        cleanup();
        return false;
    }
}

bool ESP32_LCD_OTA_UART::initUART() {
    // Flush and end Arduino Serial
    Serial.flush();
    delay(500);
    Serial.end();
    delay(1000);
    
    // Delete any existing UART driver
    uart_driver_delete(OTA_UART_NUM);
    delay(200);
    
    // Apply UART configuration
    esp_err_t err = uart_param_config(OTA_UART_NUM, &uartConfig);
    if (err != ESP_OK) {
        Serial.begin(115200);
        Serial.printf("[OTA] Error: uart_param_config failed: %d\n", err);
        return false;
    }
    
    // Set UART pins
    err = uart_set_pin(OTA_UART_NUM, OTA_UART_TX_PIN, OTA_UART_RX_PIN, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        Serial.begin(115200);
        Serial.printf("[OTA] Error: uart_set_pin failed: %d\n", err);
        return false;
    }
    
    // Install UART driver
    err = uart_driver_install(OTA_UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        Serial.begin(115200);
        Serial.printf("[OTA] Error: uart_driver_install failed: %d\n", err);
        return false;
    }
    
    // Flush input buffer multiple times
    for (int i = 0; i < 3; i++) {
        uart_flush_input(OTA_UART_NUM);
        delay(100);
    }
    
    return true;
}

bool ESP32_LCD_OTA_UART::deinitUART() {
    uart_driver_delete(OTA_UART_NUM);
    delay(100);
    
    // Re-attach Serial
    Serial.begin(115200);
    delay(100);
    
    return true;
}

void ESP32_LCD_OTA_UART::flushUARTInput() {
    uart_flush_input(OTA_UART_NUM);
}

bool ESP32_LCD_OTA_UART::sendData(const uint8_t* data, size_t length) {
    int written = uart_write_bytes(OTA_UART_NUM, (const char*)data, length);
    if (written != length) {
        return false;
    }
    
    esp_err_t err = uart_wait_tx_done(OTA_UART_NUM, 1000 / portTICK_PERIOD_MS);
    return (err == ESP_OK);
}

bool ESP32_LCD_OTA_UART::sendString(const char* str) {
    return sendData((const uint8_t*)str, strlen(str));
}

bool ESP32_LCD_OTA_UART::waitForToken(const char* expected, uint32_t timeoutMs) {
    size_t len = strlen(expected);
    char buf[8] = {0};
    size_t idx = 0;
    
    uint32_t startTime = millis();
    
    while (true) {
        // Check timeout
        if (timeoutMs > 0 && (millis() - startTime > timeoutMs)) {
            return false;
        }
        
        uint8_t c;
        int bytes = uart_read_bytes(OTA_UART_NUM, &c, 1, 100 / portTICK_PERIOD_MS);
        
        if (bytes > 0) {
            if (c == '\n' || c == '\r') continue;
            
            buf[idx++] = c;
            if (idx == len) {
                if (memcmp(buf, expected, len) == 0) {
                    return true;
                }
                memmove(buf, buf + 1, len - 1);
                idx = len - 1;
            }
        }
    }
}

uint8_t ESP32_LCD_OTA_UART::calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

bool ESP32_LCD_OTA_UART::performHandshake() {
    Serial.println("[OTA] Starting handshake...");
    
    // Step 1: Send HELLO
    if (!sendString(HELLO)) {
        Serial.println("[OTA] Failed to send HELLO");
        return false;
    }
    delay(200);
    
    // Step 2: Wait for HELLO_ACK
    if (!waitForToken(HELLO_ACK, 5000)) {
        Serial.println("[OTA] Timeout waiting for HELLO_ACK");
        return false;
    }
    Serial.println("[OTA] Received HELLO_ACK");
    delay(100);
    
    // Step 3: Send OTA_START
    if (!sendString(OTA_START)) {
        Serial.println("[OTA] Failed to send OTA_START");
        return false;
    }
    delay(100);
    
    // Step 4: Wait for OTA_OK
    if (!waitForToken(OTA_OK, 5000)) {
        Serial.println("[OTA] Timeout waiting for OTA_OK");
        return false;
    }
    Serial.println("[OTA] Received OTA_OK - handshake complete");
    delay(100);
    
    return true;
}

bool ESP32_LCD_OTA_UART::sendFirmwareMetadata() {
    // Send firmware size (4 bytes, little-endian)
    uint8_t sizeBytes[4];
    sizeBytes[0] = (uint8_t)(firmwareSize & 0xFF);
    sizeBytes[1] = (uint8_t)((firmwareSize >> 8) & 0xFF);
    sizeBytes[2] = (uint8_t)((firmwareSize >> 16) & 0xFF);
    sizeBytes[3] = (uint8_t)((firmwareSize >> 24) & 0xFF);
    
    if (!sendData(sizeBytes, 4)) {
        Serial.println("[OTA] Failed to send firmware size");
        return false;
    }
    delay(100);
    
    // Calculate and send chunk count (2 bytes, little-endian)
    uint16_t totalChunks = (firmwareSize + CHUNK_SIZE - 1) / CHUNK_SIZE;
    uint8_t chunkCountBytes[2];
    chunkCountBytes[0] = (uint8_t)(totalChunks & 0xFF);
    chunkCountBytes[1] = (uint8_t)((totalChunks >> 8) & 0xFF);
    
    if (!sendData(chunkCountBytes, 2)) {
        Serial.println("[OTA] Failed to send chunk count");
        return false;
    }
    delay(100);
    
    // Wait for acknowledgment
    if (!waitForToken(OTA_OK, 5000)) {
        Serial.println("[OTA] Timeout waiting for metadata ACK");
        return false;
    }
    
    Serial.printf("[OTA] Metadata sent - Size: %d bytes, Chunks: %d\n", firmwareSize, totalChunks);
    
    // Store in stats
    stats.totalBytes = firmwareSize;
    stats.totalChunks = totalChunks;
    
    // Give target device time to prepare
    delay(2000);
    
    return true;
}

bool ESP32_LCD_OTA_UART::sendChunk(uint16_t chunkId, const uint8_t* data, size_t dataSize) {
    // Build chunk packet: [ChunkID:2][Data:0-128][CRC:1]
    uint8_t chunkPacket[CHUNK_SIZE + 3];
    
    chunkPacket[0] = (uint8_t)(chunkId & 0xFF);
    chunkPacket[1] = (uint8_t)((chunkId >> 8) & 0xFF);
    memcpy(chunkPacket + 2, data, dataSize);
    
    // Calculate CRC over chunk ID + data
    uint8_t crc = calculateCRC8(chunkPacket, dataSize + 2);
    chunkPacket[dataSize + 2] = crc;
    
    // Send chunk packet
    return sendData(chunkPacket, dataSize + 3);
}

bool ESP32_LCD_OTA_UART::waitForChunkResponse(char* response, uint32_t timeoutMs) {
    memset(response, 0, 8);
    size_t idx = 0;
    uint32_t startTime = millis();
    
    while (millis() - startTime < timeoutMs) {
        uint8_t c;
        int bytes = uart_read_bytes(OTA_UART_NUM, &c, 1, 100 / portTICK_PERIOD_MS);
        
        if (bytes > 0) {
            response[idx++] = c;
            
            if (idx >= 3) {
                // Check for known responses
                if (memcmp(response, CHUNK_ACK, 3) == 0 ||
                    memcmp(response, CHUNK_NACK, 3) == 0 ||
                    memcmp(response, RESEND_REQ, 3) == 0) {
                    return true;
                }
                
                // Shift buffer if no match
                if (idx >= 7) {
                    idx = 0;
                    memset(response, 0, 8);
                } else {
                    memmove(response, response + 1, idx - 1);
                    idx--;
                }
            }
        }
    }
    
    return false;
}

bool ESP32_LCD_OTA_UART::transferFirmware() {
    Serial.println("[OTA] Starting firmware transfer...");
    
    uint32_t sentBytes = 0;
    uint16_t currentChunkId = 1;
    uint16_t totalRetries = 0;
    uint16_t consecutiveFailures = 0;
    
    uint32_t startTime = millis();
    
    while (sentBytes < firmwareSize) {
        size_t dataSize = min((size_t)CHUNK_SIZE, (size_t)(firmwareSize - sentBytes));
        
        // Send chunk
        if (!sendChunk(currentChunkId, firmwareBuffer + sentBytes, dataSize)) {
            Serial.printf("[OTA] Failed to send chunk %d\n", currentChunkId);
            return false;
        }
        
        // Wait for LCD to process (flash write takes time)
        delay(200);
        
        // Wait for response
        char response[8];
        bool gotResponse = waitForChunkResponse(response, CHUNK_TIMEOUT_MS);
        
        if (!gotResponse) {
            Serial.printf("[OTA] Timeout on chunk %d\n", currentChunkId);
            return false;
        }
        
        // Handle response
        if (memcmp(response, CHUNK_ACK, 3) == 0) {
            // Success - move to next chunk
            sentBytes += dataSize;
            currentChunkId++;
            consecutiveFailures = 0;
            stats.sentChunks++;
            
            // Print progress
            if (currentChunkId % 10 == 0 || sentBytes == firmwareSize) {
                Serial.printf("[OTA] Progress: %d/%d chunks (%d%%)\n", 
                             currentChunkId - 1, stats.totalChunks, 
                             (sentBytes * 100) / firmwareSize);
            }
            
            delay(50);  // Normal inter-chunk delay
            
        } else if (memcmp(response, CHUNK_NACK, 3) == 0) {
            // NACK - retry same chunk
            totalRetries++;
            consecutiveFailures++;
            
            Serial.printf("[OTA] NACK on chunk %d (retry %d/%d)\n", 
                         currentChunkId, consecutiveFailures, MAX_CONSECUTIVE_FAILURES);
            
            // Check limits
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                Serial.printf("[OTA] Max consecutive failures on chunk %d\n", currentChunkId);
                return false;
            }
            
            if (totalRetries >= MAX_TOTAL_RETRIES) {
                Serial.printf("[OTA] Max total retries exceeded\n");
                return false;
            }
            
            flushUARTInput();
            delay(300);  // Longer delay before retry
            
        } else if (memcmp(response, RESEND_REQ, 3) == 0) {
            // Specific chunk resend request
            uint8_t reqChunkBytes[2];
            uart_read_bytes(OTA_UART_NUM, reqChunkBytes, 2, 500 / portTICK_PERIOD_MS);
            
            uint16_t reqChunkId = (uint16_t)reqChunkBytes[0] | ((uint16_t)reqChunkBytes[1] << 8);
            
            Serial.printf("[OTA] Resend request for chunk %d\n", reqChunkId);
            
            sentBytes = (reqChunkId - 1) * CHUNK_SIZE;
            currentChunkId = reqChunkId;
            totalRetries++;
            consecutiveFailures++;
            
            flushUARTInput();
            delay(200);
        }
    }
    
    stats.sentBytes = sentBytes;
    stats.totalRetries = totalRetries;
    stats.transferTimeMs = millis() - startTime;
    
    Serial.printf("[OTA] Transfer complete - %d bytes in %d ms\n", 
                 sentBytes, stats.transferTimeMs);
    Serial.printf("[OTA] Total retries: %d\n", totalRetries);
    
    return true;
}

ota_result_t ESP32_LCD_OTA_UART::performOTAUpdate() {
    if (firmwareBuffer == nullptr || firmwareSize == 0) {
        Serial.println("[OTA] Error: No firmware loaded. Call downloadFirmware() first.");
        return OTA_ERROR_INVALID_FIRMWARE;
    }
    
    Serial.println("\n========== LCD OTA UPDATE START ==========");
    
    otaInProgress = true;
    
    // Initialize UART
    if (!initUART()) {
        otaInProgress = false;
        return OTA_ERROR_UART_INIT;
    }
    
    // Wait for target device to be ready
    delay(7000);
    
    // Perform handshake
    if (!performHandshake()) {
        deinitUART();
        otaInProgress = false;
        return OTA_ERROR_HANDSHAKE_TIMEOUT;
    }
    
    // Send firmware metadata
    if (!sendFirmwareMetadata()) {
        deinitUART();
        otaInProgress = false;
        return OTA_ERROR_TRANSFER_TIMEOUT;
    }
    
    // Transfer firmware
    bool success = transferFirmware();
    
    delay(500);
    
    // Cleanup
    cleanup();
    deinitUART();
    otaInProgress = false;
    
    Serial.println("========== LCD OTA UPDATE COMPLETE ==========\n");
    
    return success ? OTA_SUCCESS : OTA_ERROR_TRANSFER_TIMEOUT;
}