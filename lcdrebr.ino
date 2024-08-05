#include <Wire.h>
#include <Adafruit_CCS811.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <LiquidCrystal_I2C.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

Adafruit_CCS811 ccs;
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define PM25PIN 34
#define RELAY_PIN 15

unsigned long durationPM25;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancyPM25 = 0;
unsigned long displayTimer = 0;
unsigned long lastMeasurementTime = 0;

#define DATABASE_URL "https://udara-85026-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define API_KEY "AIzaSyDeSJ-fy-BGGRHVphnJpGExddpocdnpiGs"

#define WIFI_SSID "zzz"
#define WIFI_PASSWORD "azzaazza"

FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000);

void setup() {
    Serial.begin(9600);
    Serial.println("Menginisialisasi CCS811...");

    lcd.init();
    lcd.backlight();
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi terhubung.");

    timeClient.begin();
    
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    config.timeout.wifiReconnect = 30 * 1000;
    config.timeout.socketConnection = 30 * 1000;
    config.timeout.serverResponse = 60 * 1000;
    firebaseData.setResponseSize(4096);

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Autentikasi berhasil.");
    } else {
        Serial.print("Gagal autentikasi: ");
        Serial.println(config.signer.signupError.message.c_str());
    }

    if (!ccs.begin()) {
        Serial.println("Inisialisasi CCS811 gagal!");
        while (1);
    }

    Serial.println("Inisialisasi CCS811 selesai");

    while (!ccs.available()) {
        Serial.println("Menunggu data dari CCS811...");
        delay(500);
    }

    pinMode(PM25PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    starttime = millis();
    lastMeasurementTime = millis();
}

float calculateConcentration(long lowpulseInMicroSeconds, long durationinSeconds) {
    float ratio = (lowpulseInMicroSeconds / 1000000.0) / durationinSeconds * 100.0;
    float concentration = 0.001915 * pow(ratio, 2) + 0.09522 * ratio - 0.04884;

    if (concentration < 0) {
        concentration = 0;
    }

    return concentration;
}

void reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.disconnect();
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("WiFi terhubung kembali.");
    }
}

bool kirimKeFirebase(const char* path, FirebaseJson* json) {
    int retries = 3;
    bool berhasil = false;

    while (retries > 0 && !berhasil) {
        retries--;

        if (Firebase.RTDB.pushJSON(&firebaseData, path, json)) {
            Serial.println("Data JSON berhasil dikirim ke Firebase.");
            berhasil = true;
        } else {
            Serial.print("Gagal mengirim data JSON ke Firebase: ");
            Serial.println(firebaseData.errorReason());
            delay(1000);
        }
    }

    return berhasil;
}

void tampilkanData(const char* namaPolutan, int nilai, const char* satuan) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(namaPolutan);
    lcd.setCursor(0, 1);
    lcd.print(nilai);
    lcd.print(" ");
    lcd.print(satuan);
}

String getFormattedTime(unsigned long epochTime) {
    struct tm *ptm = gmtime((time_t *)&epochTime);
    String formattedTime = String(ptm->tm_year + 1900) + "-" + 
                           String(ptm->tm_mon + 1) + "-" + 
                           String(ptm->tm_mday) + " " + 
                           String(ptm->tm_hour) + ":" + 
                           String(ptm->tm_min) + ":" + 
                           String(ptm->tm_sec);
    return formattedTime;
}

void loop() {
    reconnectWiFi();

    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    String formattedTime = getFormattedTime(epochTime);

    static int co2 = 0;
    static int tvoc = 0;
    static float conPM25_ugm3 = 0;

    durationPM25 = pulseIn(PM25PIN, LOW);
    lowpulseoccupancyPM25 += durationPM25;

    if (millis() - lastMeasurementTime >= sampletime_ms) {
        if (ccs.available()) {
            if (!ccs.readData()) {
                co2 = ccs.geteCO2();
                tvoc = ccs.getTVOC();

                float conPM25 = calculateConcentration(lowpulseoccupancyPM25, sampletime_ms / 1000);
                conPM25_ugm3 = conPM25 * 1;

                FirebaseJson json;
                json.set("timestamp", formattedTime);
                json.set("co2", co2);
                json.set("voc", tvoc);
                json.set("pm2_5", conPM25_ugm3);

                if (co2 > 1000 || tvoc > 277 || conPM25 > 13) {
                    digitalWrite(RELAY_PIN, HIGH);
                    json.set("kondisi_relay", "menyala");
                } else {
                    digitalWrite(RELAY_PIN, LOW);
                    json.set("kondisi_relay", "mati");
                }

                String kodeUnik = "sensor/kode-unik"; // Ganti dengan kode unik sebenarnya
                if (!kirimKeFirebase(kodeUnik.c_str(), &json)) {
                    Serial.println("Gagal mengirim data JSON ke Firebase.");
                }

                Serial.print("CO2=");
                Serial.print(co2);
                Serial.println(" ppm");
                Serial.print("TVOC=");
                Serial.print(tvoc);
                Serial.println(" ppb");
                Serial.print("PM2.5 ");
                Serial.print(conPM25_ugm3);
                Serial.println(" µg/m³");
            } else {
                Serial.println("Kesalahan pembacaan CCS811!");
                Serial.print("Error: ");
                Serial.println(ccs.checkError());
            }
        } else {
            Serial.println("CCS811 tidak tersedia");
        }

        lowpulseoccupancyPM25 = 0;
        lastMeasurementTime = millis();
    }

    if (millis() - displayTimer >= 15000) {
        static int polutanIndex = 0;
        displayTimer = millis();

        switch (polutanIndex) {
            case 0:
                tampilkanData("CO2", co2, "ppm");
                break;
            case 1:
                tampilkanData("TVOC", tvoc, "ppb");
                break;
            case 2:
                tampilkanData("PM2.5", conPM25_ugm3, "ug/m3");
                break;
        }
        polutanIndex = (polutanIndex + 1) % 3;
    }
}
