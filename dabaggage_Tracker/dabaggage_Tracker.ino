#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// Wi-Fi and server credentials
const char* ssid = ""; // Replace with your Wi-Fi SSID
const char* password = ""; // Replace with your Wi-Fi password
const char* serverURL = "https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/update_gps";
const char* getGeofenceURL = "https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/get_geofence";

// GPS and buzzer setup
HardwareSerial mySerial(1);
TinyGPSPlus gps;
const int manualBuzzerPin = 12;
const int geofenceBuzzerPin = 13;
const int resetButtonPin = 14; // Pin for the reset button

// State variables
bool manualBuzzerState = false;
bool geofenceBuzzerState = false;
bool manualOverride = false;

// Timer for geofence timeout
unsigned long lastGeofenceUpdateTime = 0;
const unsigned long geofenceTimeout = 30000; // 30 seconds timeout for geofence updates

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(manualBuzzerPin, OUTPUT);
  pinMode(geofenceBuzzerPin, OUTPUT);
  pinMode(resetButtonPin, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  digitalWrite(manualBuzzerPin, LOW);
  digitalWrite(geofenceBuzzerPin, LOW);

  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi!");
}

void loop() {
  while (mySerial.available()) {
    gps.encode(mySerial.read());
  }

  if (gps.location.isValid()) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    Serial.print("Latitude: ");
    Serial.print(latitude, 6);
    Serial.print(" | Longitude: ");
    Serial.println(longitude, 6);

    checkGeofence(latitude, longitude);

    if (WiFi.status() == WL_CONNECTED) {
      sendGPSData(latitude, longitude);
    }

    delay(10000);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }

  checkManualBuzzerStatus();

  // Check if the reset button is pressed
  if (digitalRead(resetButtonPin) == LOW) {
    resetGeofence();
    delay(1000); // Debounce delay
  }
}

// Functions for controlling manual buzzer
void turnOnManualBuzzer() {
  sendBuzzerCommand("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/turn_on_manual_buzzer");
  digitalWrite(manualBuzzerPin, HIGH);
  manualBuzzerState = true; // Set state to on
  Serial.println("Manual buzzer turned ON.");
}

void turnOffManualBuzzer() {
  sendBuzzerCommand("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/turn_off_manual_buzzer");
  digitalWrite(manualBuzzerPin, LOW);
  manualBuzzerState = false; // Set state to off
  Serial.println("Manual buzzer turned OFF.");
}

// Function to send GPS data
void sendGPSData(float latitude, float longitude) {
  HTTPClient http;
  if (http.begin(serverURL)) {
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{\"latitude\": " + String(latitude, 6) + ", \"longitude\": " + String(longitude, 6) + "}";
    Serial.print("Sending GPS Data: ");
    Serial.println(jsonPayload);
    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode > 0) {
      Serial.println("GPS data sent successfully. Response code: " + String(httpResponseCode));
    } else {
      Serial.print("Error in sending POST request: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Failed to connect to server.");
  }
}

// Function to check the manual buzzer status from the server
void checkManualBuzzerStatus() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    if (http.begin("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/get_last_gps")) {
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.print("Received manual buzzer status response: ");
        Serial.println(response);

        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        if (!error) {
          manualBuzzerState = doc["data"]["manual_buzzer_status"];
          if (manualBuzzerState) {
            turnOnManualBuzzer();
          } else {
            turnOffManualBuzzer();
          }
        } else {
          Serial.print("Failed to parse JSON: ");
          Serial.println(error.c_str());
        }
      } else {
        Serial.print("Error in checking manual buzzer status: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
}

// Functions for controlling geofence buzzer
void turnOnGeofenceBuzzer() {
  sendBuzzerCommand("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/turn_on_geofence_buzzer");
  digitalWrite(geofenceBuzzerPin, HIGH);
  geofenceBuzzerState = true;
  Serial.println("Geofence buzzer turned ON.");
}

void turnOffGeofenceManually() {
  sendBuzzerCommand("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/turn_off_geofence_buzzer");
  digitalWrite(geofenceBuzzerPin, LOW);
  geofenceBuzzerState = false;
  Serial.println("Geofence buzzer turned OFF.");
}

// Function to check if GPS coordinates are outside the geofence
void checkGeofence(float latitude, float longitude) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    if (http.begin(getGeofenceURL)) {
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.print("Received geofence data: ");
        Serial.println(response);

        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        if (!error) {
          float geofenceLat = doc["data"]["latitude"];
          float geofenceLon = doc["data"]["longitude"];
          float geofenceRadius = doc["data"]["radius"];

          Serial.print("Geofence Latitude: ");
          Serial.println(geofenceLat, 6);
          Serial.print("Geofence Longitude: ");
          Serial.println(geofenceLon, 6);
          Serial.print("Geofence Radius: ");
          Serial.println(geofenceRadius, 2);

          // Check if the geofence is not set (radius = 0)
          
          float distance = isOutsideGeofence(latitude, longitude, geofenceLat, geofenceLon, geofenceRadius);
          Serial.print("Current Location Latitude: ");
          Serial.println(latitude, 6);
          Serial.print("Current Location Longitude: ");
          Serial.println(longitude, 6);
          Serial.print("Distance from geofence: ");
          Serial.println(distance);

          // Only turn on the geofence buzzer if outside the geofence and no manual override is active
          if (distance > geofenceRadius && !geofenceBuzzerState && !(geofenceRadius == 0.0)) {
            turnOnGeofenceBuzzer();
          } else if (distance <= geofenceRadius && geofenceBuzzerState || geofenceRadius == 0.0) {
            turnOffGeofenceManually(); // Turn off buzzer if it was previously on
          }
        } else {
          Serial.print("Failed to parse JSON: ");
          Serial.println(error.c_str());
        }
      } else {
        Serial.print("Error in checking geofence: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }
  }
}


// Helper function to send buzzer commands to the server
void sendBuzzerCommand(const char* url) {
  HTTPClient http;
  if (http.begin(url)) {
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      Serial.println("Buzzer command sent successfully.");
    } else {
      Serial.print("Error in sending buzzer command: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
}

// Function to calculate the distance from the geofence center
float isOutsideGeofence(float lat1, float lon1, float lat2, float lon2, float radius) {
  // Haversine formula to calculate the distance between two coordinates
  const float R = 6371e3; // Earth's radius in meters
  float phi1 = lat1 * PI / 180;
  float phi2 = lat2 * PI / 180;
  float deltaPhi = (lat2 - lat1) * PI / 180;
  float deltaLambda = (lon2 - lon1) * PI / 180;

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c; // Distance in meters
}

// Function to reset geofence settings
// Function to reset geofence settings
void resetGeofence() {
  HTTPClient http;
  if (http.begin("https://baggagetrackerkn-70c29e2e94f3.herokuapp.com/reset_geofence")) {
    http.addHeader("Content-Type", "application/json");
    
    // If your server expects a JSON payload, you can include it here
    String jsonPayload = "{\"action\":\"reset\"}"; // Example payload
    int httpResponseCode = http.POST(jsonPayload);
    
    if (httpResponseCode > 0) {
      Serial.println("Geofence reset command sent successfully.");
    } else {
      Serial.print("Error in sending geofence reset command: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("Failed to connect to reset geofence endpoint.");
  }
}