/* =====================================================================
 *  AGROCONNECT - NODO DE CAMPO (EMISOR LoRa)
 * ---------------------------------------------------------------------
 *  Lee los sensores del nodo de campo:
 *    - Humedad de suelo 15 cm  (GPIO 34, analógico)
 *    - Humedad de suelo 40 cm  (GPIO 35, analógico)
 *    - Temperatura de raíz     (DS18B20 en GPIO 4)
 *    - Temperatura/humedad aire (SHT30 por I2C 21/22)
 *    - Presión atmosférica      (BMP280 por I2C 21/22)
 *
 *  Envía por LoRa (433 MHz) una trama CSV de 9 campos EXACTAMENTE en el
 *  orden que espera el Nodo Central:
 *
 *    nodoId,tempAire,humAire,tempSuelo,vpd,presion,hum15,hum40,bateria
 *
 *  Ejemplo:
 *    nodo_campo_01,24.31,58.20,21.75,1.02,1013.45,47,63,4.15
 *
 *  El Nodo Central (receptor) la parte con ',' y la sube a Firebase.
 *  IMPORTANTE: ambos nodos deben usar 433 MHz, SyncWord 0xF3 y CRC ON.
 * ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <LoRa.h>

// ================= IDENTIFICACIÓN =================
#define NODO_ID "nodo_campo_01"     // Debe ser distinto por cada nodo de campo

// ================= PINES SENSORES =================
#define SENSOR_HUM_15CM 34
#define SENSOR_HUM_40CM 35
#define PIN_DS18B20     4

#define I2C_SDA 21                  // Cable amarillo (SDA)
#define I2C_SCL 22                  // Cable azul   (SCL)

// ================= PINES LoRa (SPI) =================
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define BAND      433E6

// ================= CALIBRACIÓN SUELO =================
// Valor en aire (seco) y en agua (100%). Ajustar con tus mediciones.
#define SUELO_AIRE 3200
#define SUELO_AGUA 1300

// ================= BATERÍA (opcional) =================
// Divisor resistivo: BAT+ ---[R]---+---[R]--- GND, y el punto medio a GPIO 32.
// Si no usás batería, dejá BATTERY_PIN en -1 y se reporta BATTERY_FIJA.
#define BATTERY_PIN   (-1)
#define BATTERY_RATIO  2.0f          // Ej: 100k/100k -> 2.0
#define BATTERY_FIJA   4.20f

// ================= TIEMPOS =================
const unsigned long INTERVALO_ENVIO_MS = 2000;   // 1 envío cada 2 s
const int           REINTENTOS_LORA    = 2;

// ================= OBJETOS =================
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorRaiz(&oneWire);
Adafruit_SHT31 sht31 = Adafruit_SHT31(&Wire);
Adafruit_BMP280 bmp(&Wire);

bool shtOk      = false;
bool bmpOk      = false;
bool loraOk     = false;
unsigned long ultimoEnvio = 0;
uint32_t contadorPaquetes = 0;

// ================= UTILIDADES =================
float calcularVPD(float temp, float hum) {
  if (isnan(temp) || isnan(hum) || hum <= 0) return 0.0f;
  float es = 0.61078f * exp((17.27f * temp) / (temp + 237.3f));
  float ea = es * (hum / 100.0f);
  float vpd = es - ea;
  return (vpd < 0) ? 0.0f : vpd;
}

int leerHumedadSuelo(int pin) {
  int raw = analogRead(pin);
  int pct = map(raw, SUELO_AIRE, SUELO_AGUA, 0, 100);
  return constrain(pct, 0, 100);
}

float leerBateria() {
#if BATTERY_PIN >= 0
  uint32_t mV = analogReadMilliVolts(BATTERY_PIN);
  return (mV / 1000.0f) * BATTERY_RATIO;
#else
  return BATTERY_FIJA;
#endif
}

void escanearI2C() {
  Serial.println("\n--- ESCANEO I2C ---");
  int nDevices = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Dispositivo en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("  Error desconocido en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) Serial.println("  CRITICO: ningun dispositivo I2C (revisa 3.3V, GND, SDA 21, SCL 22)");
  else               Serial.println("  Encontrados: " + String(nDevices));
  Serial.println("-------------------\n");
}

// Devuelve true si la trama salió del módulo LoRa.
bool enviarLoRa(const char *trama) {
  if (!loraOk) {
    Serial.println("LoRa no inicializado: no se envía.");
    return false;
  }

  for (int intento = 1; intento <= REINTENTOS_LORA; intento++) {
    LoRa.beginPacket();
    LoRa.print(trama);
    if (LoRa.endPacket()) return true;

    Serial.printf("  Reintento %d/%d falló\n", intento, REINTENTOS_LORA);
    delay(400);
  }
  return false;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==========================================");
  Serial.println("   AGROCONNECT - NODO DE CAMPO (EMISOR)   ");
  Serial.println("==========================================");

  pinMode(SENSOR_HUM_15CM, INPUT);
  pinMode(SENSOR_HUM_40CM, INPUT);

  // 1. I2C y sensores
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(1000);
  escanearI2C();

  sensorRaiz.begin();
  sensorRaiz.setResolution(11);   // 11 bits = 0.125 °C, conversión más rápida

  if (sht31.begin(0x44) || sht31.begin(0x45)) {
    shtOk = true;
    Serial.println("SHT30 listo.");
  } else {
    Serial.println("Alerta: SHT30 no detectado.");
  }

  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmpOk = true;
    Serial.println("BMP280 listo.");
  } else {
    Serial.println("Alerta: BMP280 no detectado (revisa 3.3V, GND, SDA 21, SCL 22).");
  }

  // 2. LoRa después de I2C (comparten pines distintos, pero orden claro)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (LoRa.begin(BAND)) {
    LoRa.setSyncWord(0xF3);   // Mismo valor que el Nodo Central
    LoRa.setTxPower(17);
    LoRa.enableCrc();         // Mismo que el Nodo Central
    loraOk = true;
    Serial.println("LoRa listo (433 MHz | Sync 0xF3 | CRC ON | 17 dBm).");
  } else {
    Serial.println("ERROR: no se detectó el módulo LoRa.");
    Serial.println("       Revisa SCK=18, MISO=19, MOSI=23, CS=5, RST=14, DIO0=2.");
  }

  Serial.printf("Nodo: %s | Envío cada %lu s\n",
                NODO_ID, INTERVALO_ENVIO_MS / 1000UL);
  Serial.println("==========================================\n");
}

// ================= LOOP =================
void loop() {
  unsigned long ahora = millis();
  if (ahora - ultimoEnvio < INTERVALO_ENVIO_MS) {
    delay(20);
    return;
  }
  ultimoEnvio = ahora;

  // 1. Humedad de suelo
  int hum15 = leerHumedadSuelo(SENSOR_HUM_15CM);
  int hum40 = leerHumedadSuelo(SENSOR_HUM_40CM);

  // 2. Temperatura de raíz
  sensorRaiz.requestTemperatures();               // bloquea ~375 ms
  float tempSuelo = sensorRaiz.getTempCByIndex(0);
  bool sueloOk = (tempSuelo != DEVICE_DISCONNECTED_C &&
                  tempSuelo > -50.0f && tempSuelo < 85.0f);
  if (!sueloOk) tempSuelo = 0.0f;

  // 3. Ambiente (SHT30)
  float tempAire = 0.0f, humAire = 0.0f;
  if (shtOk) {
    tempAire = sht31.readTemperature();
    humAire  = sht31.readHumidity();
    if (isnan(tempAire)) tempAire = 0.0f;
    if (isnan(humAire))  humAire  = 0.0f;
  }

  // 4. Presión (BMP280)
  float presion = 0.0f;
  if (bmpOk) {
    presion = bmp.readPressure() / 100.0f;        // Pa -> hPa
    if (isnan(presion)) presion = 0.0f;
  }

  float vpd     = calcularVPD(tempAire, humAire);
  float bateria = leerBateria();

  // 5. Trama CSV (9 campos, sin espacios: el central no acepta vacíos)
  char trama[110];
  snprintf(trama, sizeof(trama),
           "%s,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f",
           NODO_ID, tempAire, humAire, tempSuelo, vpd, presion,
           hum15, hum40, bateria);

  // 6. Mostrar en monitor serie (igual que tu prueba local)
  contadorPaquetes++;
  Serial.println("====== NUEVA LECTURA ======");
  Serial.print("Humedad Suelo 15cm: ");            Serial.print(hum15);     Serial.println("%");
  Serial.print("Humedad Suelo 40cm: ");            Serial.print(hum40);     Serial.println("%");
  Serial.print("Temp. Suelo (DS18B20): ");         Serial.print(tempSuelo, 1); Serial.println(" °C");
  Serial.print("Temp. Ambiente (SHT30): ");        Serial.print(tempAire, 1);  Serial.println(" °C");
  Serial.print("Humedad Ambiente (SHT30): ");      Serial.print(humAire, 1);   Serial.println("%");
  Serial.print("Presion Atmosferica (BMP280): ");  Serial.print(presion, 1);   Serial.println(" hPa");
  Serial.print("VPD: ");                           Serial.print(vpd, 2);       Serial.println(" kPa");
  Serial.print("Bateria: ");                       Serial.print(bateria, 2);   Serial.println(" V");
  Serial.println("===========================");

  // 7. Enviar por LoRa
  Serial.printf("TX #%lu -> %s\n", contadorPaquetes, trama);
  bool ok = enviarLoRa(trama);
  Serial.println(ok ? "Enviado OK" : "ERROR de envio");
  Serial.println();
}
