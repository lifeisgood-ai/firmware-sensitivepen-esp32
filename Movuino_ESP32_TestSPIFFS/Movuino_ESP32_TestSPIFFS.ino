#include "SPIFFS.h"

// https://www.mischianti.org/2020/06/04/esp32-integrated-spiffs-filesystem-part-2/
void printDirectory(File dir, int numTabs = 3);

void setup() {
  Serial.begin(115200);

  delay(500);

  Serial.println(F("Inizializing FS..."));
  if (SPIFFS.begin()) {
    Serial.println(F("done."));
  } else {
    Serial.println(F("fail."));
  }

  // To format all space in SPIFFS
  // SPIFFS.format()

  // Get all information of your SPIFFS

  unsigned int totalBytes = SPIFFS.totalBytes();
  unsigned int usedBytes = SPIFFS.usedBytes();

  Serial.println("File sistem info.");

  Serial.print("Total space:      ");
  Serial.print(totalBytes);
  Serial.println("byte");

  Serial.print("Total space used: ");
  Serial.print(usedBytes);
  Serial.println("byte");

  Serial.println();

  // Open dir folder
  File dir = SPIFFS.open("/");
  // Cycle all the content
  printDirectory(dir);
}

void loop() {
  Serial.print("Writting into SPIFFS...");
  File file = SPIFFS.open("spiffTest.txt", "w");
  delay(10);
  if (file.print("Content written on the spiffs")) {
    Serial.println("SUCCESS");
  } else {
    Serial.println("FAIL");
  }
  delay(10);
  file.close();

  delay(1000);

  Serial.println("Reading SPIFFS content...");
  file = SPIFFS.open("spiffTest.txt", "r");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  delay(1000);
}


void printDirectory(File dir, int numTabs) {
  while (true) {
 
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
