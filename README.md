# ESP32_to_LCD_or_Display_OTA_over_UART
 firmware update library for ESP32 to LCD/display modules using UART communication with CRC verification and error recovery.

Implements a reliable Over-The-Air (OTA) firmware update mechanism for LCD displays or other microcontrollers connected to an ESP32 via UART. It features:

-Direct UART hardware access using ESP-IDF drivers for reliable communication
-CRC8 verification for data integrity on every chunk
-ACK/NACK protocol with automatic retransmission
-Chunk-based transfer 
-Comprehensive error handling with configurable retry limits
-Non-blocking firmware download from HTTPS servers
-PSRAM support for large firmware files
-Even parity UART configuration for noise-resistant communication
