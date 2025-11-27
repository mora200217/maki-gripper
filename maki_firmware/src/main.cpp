#include <WiFi.h>
#include "maki_controller.hpp"

#define DirectionPin 10
#define BaudRate 1000000

const char* ssid     = "FAMILIA MARTINEZ1";
const char* password = "JUDAS1967";

WiFiServer server(80);

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.begin(ssid, password);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }

    Serial.println("\nConnected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    maki_controller.begin(BaudRate, DirectionPin, &Serial);

    server.begin();
    Serial.println("HTTP server ready on port 80");
}

// ------------------ FUNCIONES UTILIDAD ------------------
String getValue(String data, String key) {
    int keyIndex = data.indexOf(key + "=");
    if (keyIndex < 0) return "";

    int start = keyIndex + key.length() + 1;
    int end = data.indexOf("&", start);
    if (end < 0) end = data.length();

    return data.substring(start, end);
}

// -------------------------- LOOP -------------------------
void loop() {
    WiFiClient client = server.available();
    if (!client) return;

    Serial.println("Client connected");

    unsigned long timeout = millis();
    String req = "";

    // --- Espera cabecera con timeout ---
    while (client.connected() && millis() - timeout < 1000) {
        if (client.available()) {
            req = client.readStringUntil('\n');
            Serial.print("REQ: ");
            Serial.println(req);
            break;
        }
    }

    if (req.length() == 0) {
        Serial.println("Timeout o petición vacía");
        client.stop();
        return;
    }

    // ------------- PROCESAR GET /move -----------------
    if (req.startsWith("GET /move")) {

        String idStr  = getValue(req, "id");
        String posStr = getValue(req, "pos");

        if (idStr == "" || posStr == "") {
            Serial.println("Missing parameters");
            client.println("HTTP/1.1 400 Bad Request");
            client.println("Content-Type: text/plain");
            client.println("Connection: close");
            client.println();
            client.println("Missing parameters");
            client.stop();
            return;
        }

        int id  = idStr.toInt();
        int pos = posStr.toInt();

        Serial.printf("Moving ID=%d to pos=%d\n", id, pos);
        maki_controller.move(id, pos);

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println("Connection: close");
        client.println();
        client.printf("Moved ID=%d to pos=%d\n", id, pos);

        client.stop();
        return;
    }

    // ------- SI NO ES /move → 404 -------
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Not Found");
    client.stop();
}
