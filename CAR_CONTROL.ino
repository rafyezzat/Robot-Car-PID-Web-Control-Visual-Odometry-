#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi credentials for AP mode
const char* ssid = "Car_Controller";
const char* password = "12345678";

// UART setup to talk to Arduino Mega
#define ARDUINO_TX 21  // TX → Mega RX1 (Pin 19)
#define ARDUINO_RX 20  // RX ← Mega TX1 (Pin 18)
HardwareSerial ArduinoMega(1);  // Use UART1 on ESP32-C3

// Stepper motor control pins
#define STEP 4
#define DIR  3

WebServer server(80);
String latestMessage = "Waiting for message...";
bool stepperRunning = false;

// Message from ESP32 LiDAR
String lidarData = "No data yet";

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Robot Car Control</title>
  <style>
    body { background-color: black; color: white; text-align: center; font-family: Arial; }
    h1 { color: cyan; }
    .button {
      background-color: blue;
      color: white;
      padding: 20px 40px;
      margin: 10px;
      font-size: 24px;
      border: none;
      border-radius: 12px;
      cursor: pointer;
    }
    #statusBox {
      margin-top: 20px;
      font-size: 20px;
      color: yellow;
    }
    #lidarBox { margin-top: 10px; font-size: 18px; color: lightgreen; }
  </style>
</head>
<body>
  <h1>Control Car</h1>
  <div>
    <button class="button" onclick="sendCmd('F')">Forward</button><br>
    <button class="button" onclick="sendCmd('L')">Left</button>
    <button class="button" onclick="sendCmd('S')">Stop</button>
    <button class="button" onclick="sendCmd('R')">Right</button><br>
    <button class="button" onclick="sendCmd('B')">Backward</button><br><br>

    <button class="button" id="lidarToggle" onclick="toggleLidar()">Start LiDAR Stream</button>

    <button class="button" onclick="sendCmd('Q')">Turn Left 90°</button>
    <button class="button" onclick="sendCmd('E')">Turn Right 90°</button>
    <button class="button" onclick="sendCmd('H')">Set Heading (H)</button>
  </div>
  
  <h2>Control GPS</h2>
  <div>
    <button class="button" onclick="sendCmd('I')">GPS Info (I)</button>
    <button class="button" onclick="sendCmd('Y')">Set Waypoint (Y)</button>
    <button class="button" onclick="sendCmd('W')">Go to Waypoint (W)</button><br>
    <button class="button" onclick="sendCmd('D')">Done (D)</button>
    <button class="button" onclick="sendCmd('C')">Clear Waypoints (C)</button>
  </div>

  <h2>Stepper Motor</h2>
  <div>
    <button class="button" onclick="sendCmd('T')">Start Stepper (T)</button>
    <button class="button" onclick="sendCmd('X')">Stop Stepper (X)</button>
  </div>

  <h3 id="statusBox">Status: Waiting for message...</h3>
  <h3 id="lidarBox">LiDAR: No data yet</h3>

  <script>
    let lidarActive = false;
    let lidarInterval = null;

    function sendCmd(cmd) {
      if (lidarActive) return; // Block other commands when LiDAR stream is active
      fetch("/move?dir=" + cmd);
    }

    function toggleLidar() {
      lidarActive = !lidarActive;
      const btn = document.getElementById("lidarToggle");

      if (lidarActive) {
        btn.innerText = "Stop LiDAR Stream";
        lidarInterval = setInterval(() => {
          fetch("/lidar")
            .then(res => res.text())
            .then(text => {
              document.getElementById("lidarBox").innerText = "LiDAR: " + text;
            });
        }, 200); // Stream every 200ms
      } else {
        clearInterval(lidarInterval);
        btn.innerText = "Start LiDAR Stream";
      }
    }

    setInterval(() => {
      fetch("/status")
        .then(res => res.text())
        .then(text => {
          document.getElementById("statusBox").innerText = "Status: " + text;
        });
    }, 100);
  </script>
</body>
</html>
)rawliteral";

// Handle web server
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleMove() {
  if (server.hasArg("dir")) {
    String dir = server.arg("dir");
    if (String("FBLRSIYWDCHQE").indexOf(dir) >= 0) {
      ArduinoMega.print(dir);
      Serial.print("Sent command: ");
      Serial.println(dir);
    } else if (dir == "T") {
      stepperRunning = true;
      latestMessage = "Stepper Started";
    } else if (dir == "X") {
      stepperRunning = false;
      latestMessage = "Stepper Stopped";
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  server.send(200, "text/plain", latestMessage);
}

void handleReceive() {
  if (server.hasArg("angle") && server.hasArg("distance")) {
    lidarData = "Angle: " + server.arg("angle") + "°, Distance: " + server.arg("distance") + " mm";
    Serial.println("[LiDAR] " + lidarData);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing angle or distance");
  }
}

void handleLidar() {
  server.send(200, "text/plain", lidarData);
}

void setup() {
  Serial.begin(115200);
  ArduinoMega.begin(9600, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH);

  WiFi.softAP(ssid, password);
  Serial.println("ESP32-C3 started in AP mode");

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/status", handleStatus);
  server.on("/receive", handleReceive);
  server.on("/lidar", handleLidar);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();

  // UART Receive
  static String buffer = "";
  while (ArduinoMega.available()) {
    char c = ArduinoMega.read();
    if (c == '\n') {
      latestMessage = buffer;
      Serial.println("From Mega: " + buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }

  // Stepper pulse generation
  static unsigned long lastStepTime = 0;
  if (stepperRunning) {
    unsigned long now = micros();
    if (now - lastStepTime >= 1000) {  // ~500 µs HIGH + 500 µs LOW = 1ms per step
      digitalWrite(STEP, !digitalRead(STEP));
      lastStepTime = now;
    }
  } else {
    digitalWrite(STEP, LOW);  // Stop pulses
  }
}
