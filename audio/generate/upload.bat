C:\mkspiffs\mkspiffs.exe -c tts_wavs -b 4096 -p 256 -s 0x160000 spiffs.bin
python -m esptool --chip esp32 --port COM6 write-flash 0x290000 spiffs.bin