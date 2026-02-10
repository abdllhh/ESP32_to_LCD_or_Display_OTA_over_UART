#ifndef ESP32_LCD_OTA_UART_H
#define ESP32_LCD_OTA_UART_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "driver/uart.h"

// Protocol Constants
#define HELLO       "AAA"    ///< Initial handshake from ESP32
#define HELLO_ACK   "BBB"    ///< Handshake acknowledgment from target
#define OTA_START   "SSS"    ///< OTA session start command
#define OTA_OK      "KKK"    ///< Ready to receive firmware
#define CHUNK_ACK   "ACK"    ///< Chunk received OK
#define CHUNK_NACK  "NAK"    ///< Chunk failed, resend
#define RESEND_REQ  "RSN"    ///< Request specific chunk resend

// Configuration Constants
#define CHUNK_SIZE  128      ///< Bytes per chunk (default: 128)
#define MAX_CONSECUTIVE_FAILURES 5   ///< Max retries for same chunk
#define MAX_TOTAL_RETRIES 100        ///< Max total retries across all chunks
#define CHUNK_TIMEOUT_MS 15000       ///< Timeout per chunk in milliseconds
#define UART_BUFFER_SIZE 4096        ///< UART RX/TX buffer size

// UART Configuration
#define OTA_UART_NUM UART_NUM_0      ///< UART port number
#define OTA_UART_TX_PIN 1            ///< UART TX pin (GPIO1)
#define OTA_UART_RX_PIN 3            ///< UART RX pin (GPIO3)
#define OTA_UART_BAUD_RATE 115200    ///< UART baud rate

/**
 * @brief OTA result codes
 */
typedef enum {
    OTA_SUCCESS = 0,                  ///< OTA completed successfully
    OTA_ERROR_DOWNLOAD_FAILED = 1,    ///< Firmware download failed
    OTA_ERROR_NO_MEMORY = 2,          ///< Insufficient memory
    OTA_ERROR_UART_INIT = 3,          ///< UART initialization failed
    OTA_ERROR_HANDSHAKE_TIMEOUT = 4,  ///< Handshake timeout
    OTA_ERROR_TRANSFER_TIMEOUT = 5,   ///< Chunk transfer timeout
    OTA_ERROR_MAX_RETRIES = 6,        ///< Maximum retries exceeded
    OTA_ERROR_INVALID_FIRMWARE = 7    ///< Invalid firmware size/format
} ota_result_t;

/**
 * @brief OTA statistics structure
 */
typedef struct {
    uint32_t totalBytes;              ///< Total firmware size in bytes
    uint32_t sentBytes;               ///< Bytes successfully sent
    uint16_t totalChunks;             ///< Total number of chunks
    uint16_t sentChunks;              ///< Chunks successfully sent
    uint16_t totalRetries;            ///< Total retry count
    uint32_t transferTimeMs;          ///< Transfer duration in milliseconds
} ota_stats_t;

/**
 * @brief ESP32 LCD OTA UART Class
 * 
 * Main class for performing OTA updates to LCD/display modules over UART.
 * Handles firmware download, UART communication, and error recovery.
 */
class ESP32_LCD_OTA_UART {
public:
    /**
     * @brief Constructor
     */
    ESP32_LCD_OTA_UART();
    
    /**
     * @brief Destructor - cleans up allocated resources
     */
    ~ESP32_LCD_OTA_UART();
    
    /**
     * @brief Download firmware from HTTPS server
     * 
     * Downloads firmware binary from specified URL and stores in PSRAM buffer.
     * Requires WiFiClientSecure to be configured with proper certificates.
     * 
     * @param url HTTPS URL of the firmware binary
     * @param username Optional: HTTP basic auth username (nullptr if not needed)
     * @param password Optional: HTTP basic auth password (nullptr if not needed)
     * @return true on success, false on failure
     * 
     * @note You must configure SSL certificates before calling this:
     *       WiFiClientSecure client;
     *       client.setCACert(your_root_ca);
     */
    bool downloadFirmware(const char* url, const char* username = nullptr, const char* password = nullptr);
    
    /**
     * @brief Perform OTA update to target device via UART
     * 
     * Executes complete OTA update process:
     * 1. Detaches Arduino Serial library from UART0
     * 2. Configures UART0 hardware directly
     * 3. Performs handshake with target device
     * 4. Transfers firmware in chunks with CRC verification
     * 5. Handles retries and errors
     * 6. Reattaches Serial library when complete
     * 
     * @return OTA result code (OTA_SUCCESS or error code)
     * 
     * @warning This function blocks until OTA completes or fails.
     *          Serial.print() calls from other code will be ignored during update.
     */
    ota_result_t performOTAUpdate();
    
    /**
     * @brief Get OTA statistics from last transfer
     * 
     * @return Reference to statistics structure
     */
    const ota_stats_t& getStats() const;
    
    /**
     * @brief Check if OTA is currently in progress
     * 
     * @return true if OTA update is running
     */
    bool isOTAInProgress() const;
    
    /**
     * @brief Free firmware buffer and reset state
     * 
     * Call this to free memory after OTA completes if you need the PSRAM back.
     * Automatically called by destructor.
     */
    void cleanup();
    
    /**
     * @brief Set custom CA certificate for HTTPS downloads
     * 
     * @param ca_cert PEM-encoded CA certificate string
     */
    void setCACertificate(const char* ca_cert);
    
    /**
     * @brief Set to skip certificate validation (insecure, for testing only)
     * 
     * @param insecure true to skip validation (NOT recommended for production)
     */
    void setInsecure(bool insecure);

private:
    // Firmware buffer management
    uint8_t* firmwareBuffer;          ///< PSRAM buffer for firmware
    uint32_t firmwareSize;            ///< Size of firmware in bytes
    bool otaInProgress;               ///< OTA active flag
    
    // Statistics
    ota_stats_t stats;                ///< Transfer statistics
    
    // UART configuration
    uart_config_t uartConfig;         ///< UART hardware configuration
    
    // Security
    const char* caCert;               ///< CA certificate for HTTPS
    bool insecureMode;                ///< Skip certificate validation
    
    /**
     * @brief Initialize UART hardware
     * 
     * Configures UART0 with proper settings for OTA transfer:
     * - 115200 baud (configurable)
     * - 8 data bits
     * - Even parity
     * - 1 stop bit
     * 
     * @return true on success, false on failure
     */
    bool initUART();
    
    /**
     * @brief Deinitialize UART and restore Serial
     * 
     * @return true on success, false on failure
     */
    bool deinitUART();
    
    /**
     * @brief Wait for specific token from target device
     * 
     * Blocks until expected token is received via UART.
     * Filters out newline/carriage return characters.
     * 
     * @param expected Token string to wait for
     * @param timeoutMs Timeout in milliseconds (0 = infinite)
     * @return true if token received, false on timeout
     */
    bool waitForToken(const char* expected, uint32_t timeoutMs = 0);
    
    /**
     * @brief Send data over UART and wait for transmission complete
     * 
     * @param data Pointer to data buffer
     * @param length Number of bytes to send
     * @return true on success, false on failure
     */
    bool sendData(const uint8_t* data, size_t length);
    
    /**
     * @brief Send string over UART
     * 
     * @param str Null-terminated string to send
     * @return true on success, false on failure
     */
    bool sendString(const char* str);
    
    /**
     * @brief Perform handshake with target device
     * 
     * Executes handshake sequence:
     * ESP32 → LCD: HELLO
     * LCD → ESP32: HELLO_ACK
     * ESP32 → LCD: OTA_START
     * LCD → ESP32: OTA_OK
     * 
     * @return true on success, false on timeout or invalid response
     */
    bool performHandshake();
    
    /**
     * @brief Send firmware metadata (size and chunk count)
     * 
     * @return true on success, false on failure
     */
    bool sendFirmwareMetadata();
    
    /**
     * @brief Transfer firmware in chunks with error recovery
     * 
     * Sends firmware data in CHUNK_SIZE blocks with:
     * - CRC8 verification
     * - ACK/NACK/RESEND protocol
     * - Automatic retries
     * 
     * @return true on success, false on failure
     */
    bool transferFirmware();
    
    /**
     * @brief Send single firmware chunk
     * 
     * Chunk format: [ChunkID:2][Data:0-128][CRC:1]
     * 
     * @param chunkId Chunk identifier (1-based)
     * @param data Pointer to chunk data
     * @param dataSize Size of chunk data (up to CHUNK_SIZE)
     * @return true on success, false on failure
     */
    bool sendChunk(uint16_t chunkId, const uint8_t* data, size_t dataSize);
    
    /**
     * @brief Calculate CRC8 checksum
     * 
     * Uses simple XOR-based CRC8 algorithm for data verification.
     * 
     * @param data Pointer to data buffer
     * @param length Number of bytes to process
     * @return 8-bit CRC value
     */
    uint8_t calculateCRC8(const uint8_t* data, size_t length);
    
    /**
     * @brief Wait for chunk acknowledgment
     * 
     * Waits for ACK, NACK, or RESEND_REQ from target device.
     * 
     * @param response Output buffer for response (min 8 bytes)
     * @param timeoutMs Timeout in milliseconds
     * @return true if valid response received, false on timeout
     */
    bool waitForChunkResponse(char* response, uint32_t timeoutMs);
    
    /**
     * @brief Flush UART input buffer
     */
    void flushUARTInput();
};

#endif // ESP32_LCD_OTA_UART_H