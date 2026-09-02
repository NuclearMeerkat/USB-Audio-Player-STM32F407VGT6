#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// WiFi credentials for the ESP32 access point
const char* ssid = "USB_AUDIO_PLAYER";
const char* password = "12345678";

// Create a web server on port 80
AsyncWebServer server(80);

// UART baud rate 38400
#define UART_BAUD 38400

// Sample rate and Q factor used for coefficient calculation
#define FS 44100.0f
#define Q_FACTOR 1.4f // 0.703f

// Command identifiers for packets
#define CMD_VOLUME 0x01
#define CMD_COEF   0x02

// Packet headers
#define HEADER_BYTE1 0xAA
#define HEADER_BYTE2 0x55

// Packet sizes
#define VOLUME_PACKET_SIZE 5    // Header (2) + CMD (1) + Volume (1) + Checksum (1)
#define COEF_PACKET_SIZE   25   // Header (2) + CMD (1) + FilterID (1) + 5 floats (20) + Checksum (1)

// Frequency bands (in Hz) corresponding to each EQ slider (filter IDs 0-9)
const float eqFrequencies[10] = {32.0f, 64.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f, 16000.0f};


// Global volume value (0 to 255)
volatile int volumeValue = 200;

// ---------- Functions for Packet Sending ----------

// Function to send a volume packet over UART
void sendVolumePacket(int volume) {
  uint8_t packet[VOLUME_PACKET_SIZE];
  packet[0] = HEADER_BYTE1;
  packet[1] = HEADER_BYTE2;
  packet[2] = CMD_VOLUME;
  packet[3] = (uint8_t)volume;
  
  // Calculate checksum: sum of header, CMD and volume modulo 256
  uint8_t checksum = 0;
  for (int i = 0; i < VOLUME_PACKET_SIZE - 1; i++) {
    checksum += packet[i];
  }
  packet[4] = checksum;
  
  Serial.write(packet, VOLUME_PACKET_SIZE);
}

// Function to send a coefficient packet over UART
void sendCoefPacket(uint8_t filterID, float coef[5]) {
  uint8_t packet[COEF_PACKET_SIZE];
  packet[0] = HEADER_BYTE1;
  packet[1] = HEADER_BYTE2;
  packet[2] = CMD_COEF;
  packet[3] = filterID;
  
  // Pack the 5 float coefficients (each 4 bytes) into packet (little-endian)
  for (int i = 0; i < 5; i++) {
    union {
      float f;
      uint8_t b[4];
    } data;
    data.f = coef[i];
    memcpy(packet + 4 + i*4, data.b, 4);
  }
  
  // Calculate checksum: sum of first 24 bytes modulo 256
  uint8_t checksum = 0;
  for (int i = 0; i < COEF_PACKET_SIZE - 1; i++) {
    checksum += packet[i];
  }
  packet[COEF_PACKET_SIZE - 1] = checksum;
  
  Serial.write(packet, COEF_PACKET_SIZE);
}

// ---------- Function to Calculate Peaking EQ Coefficients ----------
// This example uses a standard peaking EQ formula (RBJ Cookbook).
// It calculates 5 coefficients: [a0', a1', a2', b1', b2'] to be used in a biquad filter.
// The filter difference equation is assumed to be:
// y[n] = a0'*x[n] + a1'*x[n-1] + a2'*x[n-2] - b1'*y[n-1] - b2'*y[n-2]
void calcPeakingCoeffs(float fc, float Q, float gain_dB, float fs, float coef[5]) {
  float A = pow(10.0f, gain_dB / 40.0f);
  float w0 = 2.0f * PI * fc / fs;
  float cosw0 = cosf(w0);
  float sinw0 = sinf(w0);
  float alpha = sinw0 / (2.0f * Q);
  
  // Standard peaking EQ formulas (RBJ Cookbook):
  float b0 = 1.0f + alpha * A;
  float b1 = -2.0f * cosw0;
  float b2 = 1.0f - alpha * A;
  float a0 = 1.0f + alpha / A;
  float a1 = -2.0f * cosw0;
  float a2 = 1.0f - alpha / A;
  
  // Normalize the coefficients (dividing all by a0)
  coef[0] = b0 / a0;  // a0'
  coef[1] = b1 / a0;  // a1'
  coef[2] = b2 / a0;  // a2'
  coef[3] = a1 / a0;  // b1' (feedback, note the minus sign is applied in filter processing)
  coef[4] = a2 / a0;  // b2'
}

// Calculate Low-Shelf filter coefficients
// fc: cutoff frequency (Hz)
// S: shelf slope (usually 1.0)
// gain_dB: boost or cut in decibels (positive = boost, negative = cut)
// fs: sampling rate (Hz)
// coef: output array of 5 coefficients [a0', a1', a2', b1', b2']
void calcLowShelfCoeffs(float fc, float S, float gain_dB, float fs, float coef[5]) {
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    // Calculate alpha using the shelf slope S
    float alpha = sinw0 / 2.0f * sqrtf((A + 1.0f/A) * (1.0f/S - 1.0f) + 2.0f);
    
    float b0 =    A * ((A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtf(A) * alpha);
    float b1 =  2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosw0);
    float b2 =    A * ((A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtf(A) * alpha);
    float a0 =        (A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtf(A) * alpha;
    float a1 =   -2.0f * ((A - 1.0f) + (A + 1.0f) * cosw0);
    float a2 =        (A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtf(A) * alpha;
    
    // Normalize coefficients by a0
    coef[0] = b0 / a0;  // a0'
    coef[1] = b1 / a0;  // a1'
    coef[2] = b2 / a0;  // a2'
    coef[3] = a1 / a0;  // b1' (note: used with a minus in the difference eq.)
    coef[4] = a2 / a0;  // b2'
}

// Calculate High-Shelf filter coefficients
// fc: cutoff frequency (Hz)
// S: shelf slope (usually 1.0)
// gain_dB: boost or cut in decibels (positive = boost, negative = cut)
// fs: sampling rate (Hz)
// coef: output array of 5 coefficients [a0', a1', a2', b1', b2']
void calcHighShelfCoeffs(float fc, float S, float gain_dB, float fs, float coef[5]) {
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * M_PI * fc / fs;
    float cosw0 = cosf(w0);
    float sinw0 = sinf(w0);
    float alpha = sinw0 / 2.0f * sqrtf((A + 1.0f/A) * (1.0f/S - 1.0f) + 2.0f);
    
    float b0 =    A * ((A + 1.0f) + (A - 1.0f) * cosw0 + 2.0f * sqrtf(A) * alpha);
    float b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosw0);
    float b2 =    A * ((A + 1.0f) + (A - 1.0f) * cosw0 - 2.0f * sqrtf(A) * alpha);
    float a0 =        (A + 1.0f) - (A - 1.0f) * cosw0 + 2.0f * sqrtf(A) * alpha;
    float a1 =    2.0f * ((A - 1.0f) - (A + 1.0f) * cosw0);
    float a2 =        (A + 1.0f) - (A - 1.0f) * cosw0 - 2.0f * sqrtf(A) * alpha;
    
    // Normalize coefficients by a0
    coef[0] = b0 / a0;  // a0'
    coef[1] = b1 / a0;  // a1'
    coef[2] = b2 / a0;  // a2'
    coef[3] = a1 / a0;  // b1'
    coef[4] = a2 / a0;  // b2'
}

// ---------- Web Server HTML Page ----------

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 Audio Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      background-color: #000;
      color: #ffffff;
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 0px;
    }
    h2 {
      color: #FFD700; /* Gold/Yellow */
    }
    /* Horizontal slider for volume */
    .volume-container {
      margin-bottom: 30px;
    }
    .volume-slider {
      width: 80%;
    }
    /* Vertical EQ sliders */
    .eq-container {
      display: flex;
      justify-content: center;
      align-items: flex-end;
      height: 300px;
      border: 2px solid #FFD700;
      border-radius: 10px;
      background-color: #222;
    }
    .eq-slider {
      -webkit-appearance: none;  /* Remove default styling */
      width: 200px; /* This is the length of the slider track */
      height: 15px; /* Thickness of the slider track */
      transform: rotate(-90deg);
      margin: 100px -25px;
      background: #FFD700; /* Yellow track */
      border-radius: 5px;
      outline: none;
    }
    .eq-slider::-webkit-slider-thumb {
      appearance: none;
      width: 25px;
      height: 25px;
      background: #ffffff; /* White thumb */
      cursor: pointer;
      border-radius: 50%;
    }
    .eq-label {
      margin-top: 5px;
      font-size: 14px;
    }
    /* Mobile-friendly fallback: on smaller screens, display horizontal sliders */
    @media (max-width: 600px) {
      .eq-container {
        flex-direction: column;
        height: auto;
        align-items: center;
      }
      .eq-band {
        width: 100%;
        margin: 5px 0;
      }
      .slider-wrapper {
        width: 100%;
        height: auto;
        position: static;
      }
      .eq-slider {
        position: static;
        transform: none;
        width: 200px;
        margin: 10px auto;
      }
    }
  </style>
</head>
<body>
  <h2>USB Audio Player Control</h2>
  <div class="volume-container">
    <div>
      <span>Volume</span><br>
      <input type="range" min="0" max="100" value="50" class="volume-slider" id="volumeSlider">
      <output id="volOut">50</output>
    </div>
  </div>
  <div class="eq-container">
    <!-- Create 10 vertical sliders for EQ bands -->
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq0">
      <div class="eq-label">32 Hz<br><span id="eqOut0">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq1">
      <div class="eq-label">64 Hz<br><span id="eqOut1">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq2">
      <div class="eq-label">125 Hz<br><span id="eqOut2">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq3">
      <div class="eq-label">250 Hz<br><span id="eqOut3">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq4">
      <div class="eq-label">500 Hz<br><span id="eqOut4">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq5">
      <div class="eq-label">1000 Hz<br><span id="eqOut5">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq6">
      <div class="eq-label">2000 Hz<br><span id="eqOut6">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq7">
      <div class="eq-label">4000 Hz<br><span id="eqOut7">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq8">
      <div class="eq-label">8000 Hz<br><span id="eqOut8">0 dB</span></div>
    </div>
    <div>
      <input type="range" min="-20" max="20" value="0" class="eq-slider" id="eq9">
      <div class="eq-label">16000 Hz<br><span id="eqOut9">0 dB</span></div>
    </div>
  </div>
  <script>
    // Volume slider event
    var volumeSlider = document.getElementById("volumeSlider");
    var volOut = document.getElementById("volOut");
    volumeSlider.oninput = function() {
      volOut.innerHTML = this.value;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/setVolume?value=" + this.value, true);
      xhr.send();
    };

    // EQ slider event handler
    function updateEQ(filterIndex, sliderId, outputId) {
      var slider = document.getElementById(sliderId);
      var output = document.getElementById(outputId);
      slider.oninput = function() {
        output.innerHTML = this.value + " dB";
        // Send command to set EQ parameters for this filter
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/setEQ?filter=" + filterIndex + "&gain=" + this.value, true);
        xhr.send();
      };
    }
    updateEQ(0, "eq0", "eqOut0");
    updateEQ(1, "eq1", "eqOut1");
    updateEQ(2, "eq2", "eqOut2");
    updateEQ(3, "eq3", "eqOut3");
    updateEQ(4, "eq4", "eqOut4");
    updateEQ(5, "eq5", "eqOut5");
    updateEQ(6, "eq6", "eqOut6");
    updateEQ(7, "eq7", "eqOut7");
    updateEQ(8, "eq8", "eqOut8");
    updateEQ(9, "eq9", "eqOut9");
  </script>
</body>
</html>
)rawliteral";

// ---------- ESP32 Code for Handling Requests ----------

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  // Start Serial for UART communication with STM32
  Serial.begin(UART_BAUD);

  // Set ESP32 as an access point
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println();

  // Serve the HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  // Endpoint to set volume (sends a volume packet)
  server.on("/setVolume", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
      int vol = request->getParam("value")->value().toInt();
      sendVolumePacket(vol);
      request->send(200, "text/plain", "Volume OK");
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });

  // Endpoint to set EQ for a specific filter band (sends a coefficient packet)
  server.on("/setEQ", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("filter") && request->hasParam("gain")) {
      int filterID = request->getParam("filter")->value().toInt();
      float gain_dB = request->getParam("gain")->value().toFloat();
      
      // For our example, use a fixed Q factor.
      float Q = Q_FACTOR;
      // Get the center frequency from our predefined table.
      float fc = eqFrequencies[filterID];
      
      // Calculate filter coefficients based on peaking EQ formula.
      float coef[5];

      //calcLowShelfCoeffs(fc, 1, gain_dB, FS, coef);
      //calcHighShelfCoeffs(fc, 1, gain_dB, FS, coef);
      calcPeakingCoeffs(fc, Q, gain_dB, FS, coef);
      
      // Send the coefficient packet for this filter band.
      sendCoefPacket((uint8_t)filterID, coef);
      sendCoefPacket((uint8_t)filterID, coef);
      sendCoefPacket((uint8_t)filterID, coef);
      
      request->send(200, "text/plain", "EQ OK");
    } else {
      request->send(400, "text/plain", "Bad Request");
    }
  });

  server.onNotFound(notFound);
  server.begin();
}

void loop() {
  // The async server handles requests in the background.
  // No blocking code here.
}
