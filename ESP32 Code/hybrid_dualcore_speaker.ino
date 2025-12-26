#include "BluetoothA2DPSink.h"
#include <SPI.h>

// --- USER CONFIGURATION ---
BluetoothA2DPSink a2dp_sink;

// --- PIN DEFINITIONS (VSPI Default) ---
// SCK  = GPIO 18  (Connect to STM32 PC10)
// MOSI = GPIO 23  (Connect to STM32 PC12)
// CS   = GPIO 5   (Dummy, but needed for SPI initialization)
#define CS_PIN    5 

// Handshake Pin (Connect to STM32 PA3)
// We read this to know if STM32 is ready for more food!
#define READY_PIN 4 

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  
  // 1. Setup Handshake Pin
  pinMode(READY_PIN, INPUT); 

  // 2. Setup SPI (Master Mode)
  // 2MHz speed is safe for breadboards.
  // SPI_MODE0 matches STM32 (CPOL=0, CPHA=1 Edge)
  SPI.begin(); 
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // 3. Setup Bluetooth Audio
  // We use a "Stream Reader" to intercept audio data
  a2dp_sink.set_stream_reader(read_data_stream, false);
  
  // Start Bluetooth! Name will show as "Jai_Hybrid_Speaker"
  Serial.println("Starting Bluetooth...");
  a2dp_sink.start("Jai_Hybrid_Speaker");
}

// --- MAIN LOOP ---
void loop() {
  // A2DP runs in the background. keep loop clean.
  // If you disconnect, this keeps it alive.
}

// --- THE MAGIC FUNCTION (Audio Callback) ---
// This runs automatically whenever phone sends audio data.
void read_data_stream(const uint8_t *data, uint32_t length) {
  
  // The 'data' comes as 16-bit Stereo bytes: 
  // [L_Low, L_High, R_Low, R_High, L_Low, L_High...]
  
  // We need to process 4 bytes at a time to get 1 sample for STM32
  // We will take only Left Channel High Byte (8-bit conversion)
  
  int16_t *samples = (int16_t*) data; // Treat bytes as 16-bit numbers
  uint32_t sample_count = length / 2; // Total 16-bit samples (L+R)

  for (int i = 0; i < sample_count; i += 2) { // Step by 2 to skip Right channel (Mono)
    
    // 1. Get Left Channel Sample
    int16_t sample = samples[i]; 
    
    // 2. Convert Signed 16-bit (-32768 to 32767) to Unsigned 8-bit (0 to 255)
    // Add 32768 to make it positive, then divide by 256 (shift right 8)
    uint8_t audio_byte = (sample + 32768) >> 8;

    // 3. FLOW CONTROL (Crucial!)
    // Check if STM32 is saying "STOP! My stomach is full!" (Pin LOW)
    while(digitalRead(READY_PIN) == LOW) {
       // Wait here until STM32 eats the buffer.
       // Don't wait too long or audio will stutter, but SPI is fast enough.
       delayMicroseconds(10); 
    }

    // 4. Send Clean Byte to STM32
    digitalWrite(CS_PIN, LOW); // Select Slave
    SPI.transfer(audio_byte);  // Push Data
    digitalWrite(CS_PIN, HIGH);// Deselect
  }
}