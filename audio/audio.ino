#include <LittleFS.h>
#include <driver/dac.h>
//#define LittleFS LITTLEFS

// Buffer for WAV-data
#define BUFFER_SIZE 512
uint8_t buffer[BUFFER_SIZE];

// WAV-fil header struktur
typedef struct {
  char riff[4]; // "RIFF"
  uint32_t chunkSize;
  char wave[4]; // "WAVE"
  char fmt[4]; // "fmt "
  uint32_t subchunk1Size;
  uint16_t audioFormat;
  uint16_t numChannels;
  uint32_t sampleRate;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char data[4]; // "data"
  uint32_t dataSize;
} WavHeader;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialiser LittleFS
  if (!LittleFS.begin(true)) { // true = formater hvis nødvendig
    Serial.println("❌ LittleFS init failed!");
    return;
  }
  Serial.println("✅ LittleFS initialisert");
  Serial.print("Total bytes: ");
  Serial.println(LittleFS.totalBytes());
  Serial.print("Used bytes: ");
  Serial.println(LittleFS.usedBytes());

  // List filer
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("📄 ");
    Serial.print(file.name());
    Serial.print(" - ");
    Serial.print(file.size());
    Serial.println(" bytes");
    file = root.openNextFile();
  }

  // Åpne WAV-fil
  File wavFile = LittleFS.open("/farlig.wav", "r");
  if (!wavFile) {
    Serial.println("❌ Kunne ikke åpne farlig.wav!");
    return;
  }

  // Les WAV-header
  WavHeader header;
  wavFile.read((uint8_t*)&header, sizeof(WavHeader));
  if (strncmp(header.riff, "RIFF", 4) != 0 || strncmp(header.wave, "WAVE", 4) != 0) {
    Serial.println("❌ Ugyldig WAV-fil!");
    wavFile.close();
    return;
  }

  Serial.print("Sample rate: ");
  Serial.println(header.sampleRate);
  Serial.print("Channels: ");
  Serial.println(header.numChannels);
  Serial.print("Bits per sample: ");
  Serial.println(header.bitsPerSample);

  // Konfigurer DAC (GPIO25)
  dac_output_enable(DAC_CHANNEL_1); // GPIO25
  dac_output_voltage(DAC_CHANNEL_1, 128); // Start med midtpunkt (1.65V ved 3.3V)

  // Spill av WAV-data (forenklet, mono, 8-bit eller 16-bit)
  uint32_t sampleDelay = 1000000 / header.sampleRate; // Mikrosekunder per sample
  while (wavFile.available()) {
    int bytesRead = wavFile.read(buffer, BUFFER_SIZE);
    for (int i = 0; i < bytesRead; i++) {
      // For 8-bit WAV, skriv direkte til DAC
      if (header.bitsPerSample == 8) {
        dac_output_voltage(DAC_CHANNEL_1, buffer[i]);
      }
      // For 16-bit WAV, konverter til 8-bit for DAC
      else if (header.bitsPerSample == 16 && i < bytesRead - 1) {
        int16_t sample = (buffer[i] | (buffer[i + 1] << 8));
        uint8_t dacValue = (sample + 32768) >> 8; // Skaler til 8-bit
        dac_output_voltage(DAC_CHANNEL_1, dacValue);
        i++; // Hopp over neste byte
      }
      delayMicroseconds(sampleDelay);
    }
  }

  Serial.println("✅ Avspilling ferdig");
  wavFile.close();
  dac_output_disable(DAC_CHANNEL_1);
}

void loop() {}