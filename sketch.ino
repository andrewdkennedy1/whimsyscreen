/*
  Whimsy Screen (Improved - Full Sketch)
  ESP8266 + Adafruit ILI9341 + SD

  Improvements vs your original:
  - Proper scannable QR code using ricmoo/QRCode (qrcode.h)
  - mDNS hostname: http://whimsy-screen.local/ (fallback to IP if mDNS fails)
  - Optional AP fallback if STA WiFi fails (shows QR for 192.168.4.1)
  - Enhanced mobile-friendly web UI (no CDN dependencies)
  - Better API endpoints: /api/status, /api/image.bmp, /api/draw, /api/boot
  - Upload hardening: size limit + error responses
  - Last draw status + heap + uptime in status

  Library required:
    - QRCode (ricmoo) https://github.com/ricmoo/qrcode/
      Include: #include <qrcode.h>
*/

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <SPI.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <stdarg.h>
#include <stdlib.h>    // malloc
#include <qrcode.h>    // ricmoo QRCode

// ===================== Config =====================

// WiFi STA credentials
static const char* WIFI_SSID = "ThunderDoges";
static const char* WIFI_PASS = "itsinthediscord";

// Device label
static const char* DEVICE_NAME = "Whimsy Screen";

// mDNS host (http://<host>.local/)
static const char* MDNS_HOST = "whimsy-screen";

// AP fallback (if STA fails)
#define ENABLE_AP_FALLBACK   1
static const char* AP_SSID   = "WhimsyScreen";
static const char* AP_PASS   = "";         // empty = open AP; set if you want WPA2

// Upload limits
static const uint32_t MAX_UPLOAD_BYTES = 450000; // ample for BMP exports

// FeatherWing pins (ESP8266)
#define TFT_CS     0
#define TFT_DC     15
#define TFT_RST   -1
#define SD_CS      2

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
ESP8266WebServer server(80);

// Image path
static const char* BMP_PATH = "/latest.bmp";

// QR config
static const uint8_t QR_VERSION = 4;       // v4 => 33x33 modules
static const uint8_t QR_ECC     = ECC_MEDIUM;

// ===================== State =====================

static bool sdOK = false;
static bool mdnsOK = false;
static bool apMode = false;

static bool firstImageReceived = false;
static volatile bool pendingDraw = false;

static uint32_t lastUploadBytes = 0;
static uint32_t uptimeStartMs = 0;

static bool uploadError = false;
static char uploadErrorMsg[96] = {0};

static int lastBmpResult = 0;

// ===================== BMP draw result codes =====================

enum BmpResult {
  BMP_OK = 0,
  BMP_OPEN_FAIL,
  BMP_NOT_BM,
  BMP_DIB_TOO_SMALL,
  BMP_BAD_PLANES,
  BMP_UNSUPPORTED_BPP,
  BMP_UNSUPPORTED_COMP,
  BMP_BAD_DIMS,
  BMP_SEEK_FAIL,
  BMP_SHORT_READ
};

static const char* bmpResultToStr(BmpResult r) {
  switch (r) {
    case BMP_OK: return "OK";
    case BMP_OPEN_FAIL: return "open failed";
    case BMP_NOT_BM: return "not BM header";
    case BMP_DIB_TOO_SMALL: return "DIB < 40";
    case BMP_BAD_PLANES: return "planes != 1";
    case BMP_UNSUPPORTED_BPP: return "bpp not 24/32";
    case BMP_UNSUPPORTED_COMP: return "unsupported compression";
    case BMP_BAD_DIMS: return "bad width/height";
    case BMP_SEEK_FAIL: return "seek failed";
    case BMP_SHORT_READ: return "short pixel read";
    default: return "unknown";
  }
}

// ===================== Logging =====================

static void logf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}

// ===================== Helpers =====================

static inline void deselectAll() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
}

static uint16_t read16(File &f) {
  uint16_t r;
  r  = (uint16_t)f.read();
  r |= (uint16_t)f.read() << 8;
  return r;
}

static uint32_t read32(File &f) {
  uint32_t r;
  r  = (uint32_t)f.read();
  r |= (uint32_t)f.read() << 8;
  r |= (uint32_t)f.read() << 16;
  r |= (uint32_t)f.read() << 24;
  return r;
}

static void tftBannerError(const char* msg) {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  tft.setCursor(10, 18);
  tft.print("ERROR");

  tft.setCursor(10, 52);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.print(msg);

  tft.setCursor(10, 74);
  tft.print("Check Serial @115200");
}

// ===================== Safe BMP draw =====================
// Supports 24-bit BI_RGB and 32-bit BI_RGB/BI_BITFIELDS
static BmpResult drawBMPFromSD_Safe(const char* path) {
  if (!sdOK) {
    logf("[BMP] SD not OK");
    return BMP_OPEN_FAIL;
  }

  File bmp = SD.open(path, FILE_READ);
  if (!bmp) {
    logf("[BMP] open failed: %s", path);
    return BMP_OPEN_FAIL;
  }

  uint16_t sig = read16(bmp);
  if (sig != 0x4D42) { // 'BM'
    logf("[BMP] signature not BM: 0x%04X", sig);
    bmp.close();
    return BMP_NOT_BM;
  }

  uint32_t fileSize = read32(bmp);
  (void)read32(bmp); // reserved
  uint32_t pixelOffset = read32(bmp);

  uint32_t dib = read32(bmp);
  if (dib < 40) {
    logf("[BMP] DIB too small: %lu", (unsigned long)dib);
    bmp.close();
    return BMP_DIB_TOO_SMALL;
  }

  int32_t bmpW = (int32_t)read32(bmp);
  int32_t bmpH = (int32_t)read32(bmp);
  uint16_t planes = read16(bmp);
  uint16_t depth  = read16(bmp);      // 24 or 32
  uint32_t comp   = read32(bmp);      // 0=BI_RGB, 3=BI_BITFIELDS

  // rest of BITMAPINFOHEADER
  uint32_t imgSize = read32(bmp);
  (void)read32(bmp); // xppm
  (void)read32(bmp); // yppm
  (void)read32(bmp); // clrUsed
  (void)read32(bmp); // clrImp

  logf("[BMP] fileSize=%lu pixelOffset=%lu dib=%lu", (unsigned long)fileSize, (unsigned long)pixelOffset, (unsigned long)dib);
  logf("[BMP] w=%ld h=%ld planes=%u bpp=%u comp=%lu imgSize=%lu", (long)bmpW, (long)bmpH, planes, depth, (unsigned long)comp, (unsigned long)imgSize);

  if (planes != 1) { bmp.close(); return BMP_BAD_PLANES; }
  if (!(depth == 24 || depth == 32)) { bmp.close(); return BMP_UNSUPPORTED_BPP; }

  bool compOK = (comp == 0) || (comp == 3 && depth == 32);
  if (!compOK) {
    logf("[BMP] unsupported comp=%lu depth=%u", (unsigned long)comp, depth);
    bmp.close();
    return BMP_UNSUPPORTED_COMP;
  }

  if (bmpW <= 0 || bmpH == 0) { bmp.close(); return BMP_BAD_DIMS; }

  bool flip = true;
  if (bmpH < 0) { bmpH = -bmpH; flip = false; }

  const uint8_t bpp = depth / 8;
  uint32_t rowSize = ((uint32_t)bmpW * bpp + 3) & ~3;

  int16_t drawW = (bmpW > (int32_t)tft.width())  ? tft.width()  : (int16_t)bmpW;
  int16_t drawH = (bmpH > (int32_t)tft.height()) ? tft.height() : (int16_t)bmpH;

  logf("[BMP] rowSize=%lu drawW=%d drawH=%d flip=%s",
       (unsigned long)rowSize, drawW, drawH, flip ? "yes" : "no");

  static uint16_t line565[320];
  static uint8_t  lineRaw[320 * 4];

  tft.fillScreen(ILI9341_BLACK);

  for (int16_t row = 0; row < drawH; row++) {
    yield();

    uint32_t srcRow = flip ? (uint32_t)(bmpH - 1 - row) : (uint32_t)row;
    uint32_t pos = pixelOffset + srcRow * rowSize;

    if (!bmp.seek(pos)) {
      logf("[BMP] seek failed row=%d pos=%lu", row, (unsigned long)pos);
      bmp.close();
      return BMP_SEEK_FAIL;
    }

    size_t need = (size_t)drawW * bpp;
    size_t got  = bmp.read(lineRaw, need);
    if (got != need) {
      logf("[BMP] short read row=%d need=%u got=%u pos=%lu",
           row, (unsigned)need, (unsigned)got, (unsigned long)pos);
      bmp.close();
      return BMP_SHORT_READ;
    }

    // BMP order: B,G,R,(A)
    for (int16_t x = 0; x < drawW; x++) {
      uint8_t b = lineRaw[x*bpp + 0];
      uint8_t g = lineRaw[x*bpp + 1];
      uint8_t r = lineRaw[x*bpp + 2];
      line565[x] = tft.color565(r, g, b);
    }

    tft.drawRGBBitmap(0, row, line565, drawW, 1);
  }

  bmp.close();
  logf("[BMP] draw OK");
  return BMP_OK;
}

// ===================== Networking =====================

static String getBestBaseURL() {
  IPAddress ip = apMode ? WiFi.softAPIP() : WiFi.localIP();

  if (mdnsOK) {
    String u = "http://";
    u += MDNS_HOST;
    u += ".local/";
    return u;
  }
  return String("http://") + ip.toString() + "/";
}

static void connectWiFi() {
  apMode = false;
  mdnsOK = false;

  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  logf("[WiFi] connecting STA to %s ...", WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    yield();
    if (millis() - start > 15000) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    logf("[WiFi] STA connected IP=%s", WiFi.localIP().toString().c_str());
    mdnsOK = MDNS.begin(MDNS_HOST);
    if (mdnsOK) {
      MDNS.addService("http", "tcp", 80);
      logf("[mDNS] started: %s.local", MDNS_HOST);
    } else {
      logf("[mDNS] failed");
    }
    return;
  }

  logf("[WiFi] STA connect FAILED");

#if ENABLE_AP_FALLBACK
  logf("[WiFi] starting AP fallback...");
  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) == 0) WiFi.softAP(AP_SSID);
  else WiFi.softAP(AP_SSID, AP_PASS);

  apMode = true;
  logf("[WiFi] AP IP=%s SSID=%s", WiFi.softAPIP().toString().c_str(), AP_SSID);

  mdnsOK = MDNS.begin(MDNS_HOST);
  if (mdnsOK) {
    MDNS.addService("http", "tcp", 80);
    logf("[mDNS] started (AP): %s.local", MDNS_HOST);
  }
#endif
}

// ===================== Proper QR (fixes your compile error) =====================

static bool drawQRCodeText(const char* text, int16_t x, int16_t y, int16_t sizePx) {
  // Allocate once, reuse forever (no VLA, no non-const static array)
  static uint8_t* qrBuf = nullptr;
  static int qrBufSize = 0;

  if (!qrBuf) {
    qrBufSize = qrcode_getBufferSize(QR_VERSION);
    qrBuf = (uint8_t*)malloc(qrBufSize);
    if (!qrBuf) return false;
  }

  QRCode qr;
  qrcode_initText(&qr, qrBuf, QR_VERSION, QR_ECC, text);

  const uint8_t quiet = 4;                 // quiet zone in modules
  const int totalMods = qr.size + quiet * 2;

  int scale = sizePx / totalMods;
  if (scale < 1) scale = 1;

  const int qrPx = totalMods * scale;
  const int16_t ox = x + (sizePx - qrPx) / 2;
  const int16_t oy = y + (sizePx - qrPx) / 2;

  // white background
  tft.fillRect(x, y, sizePx, sizePx, ILI9341_WHITE);

  // draw black modules
  for (uint8_t yy = 0; yy < qr.size; yy++) {
    for (uint8_t xx = 0; xx < qr.size; xx++) {
      if (qrcode_getModule(&qr, xx, yy)) {
        int16_t px = ox + (quiet + xx) * scale;
        int16_t py = oy + (quiet + yy) * scale;
        tft.fillRect(px, py, scale, scale, ILI9341_BLACK);
      }
    }
  }

  return true;
}

// ===================== Boot Screen =====================

static void showBootScreen() {
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  // Rainbow border - top
  tft.fillRect(0, 0, 320, 4, ILI9341_RED);
  tft.fillRect(0, 4, 320, 4, ILI9341_ORANGE);
  tft.fillRect(0, 8, 320, 4, ILI9341_YELLOW);
  tft.fillRect(0, 12, 320, 4, ILI9341_GREEN);
  tft.fillRect(0, 16, 320, 4, ILI9341_CYAN);
  tft.fillRect(0, 20, 320, 4, ILI9341_BLUE);
  tft.fillRect(0, 24, 320, 4, ILI9341_MAGENTA);

  // Rainbow border - bottom
  tft.fillRect(0, 236, 320, 4, ILI9341_MAGENTA);
  tft.fillRect(0, 232, 320, 4, ILI9341_BLUE);
  tft.fillRect(0, 228, 320, 4, ILI9341_CYAN);
  tft.fillRect(0, 224, 320, 4, ILI9341_GREEN);
  tft.fillRect(0, 220, 320, 4, ILI9341_YELLOW);
  tft.fillRect(0, 216, 320, 4, ILI9341_ORANGE);
  tft.fillRect(0, 212, 320, 4, ILI9341_RED);

  // Rainbow border - left
  tft.fillRect(0, 0, 4, 240, ILI9341_RED);
  tft.fillRect(4, 0, 4, 240, ILI9341_ORANGE);
  tft.fillRect(8, 0, 4, 240, ILI9341_YELLOW);
  tft.fillRect(12, 0, 4, 240, ILI9341_GREEN);
  tft.fillRect(16, 0, 4, 240, ILI9341_CYAN);
  tft.fillRect(20, 0, 4, 240, ILI9341_BLUE);
  tft.fillRect(24, 0, 4, 240, ILI9341_MAGENTA);

  // Rainbow border - right
  tft.fillRect(316, 0, 4, 240, ILI9341_RED);
  tft.fillRect(312, 0, 4, 240, ILI9341_ORANGE);
  tft.fillRect(308, 0, 4, 240, ILI9341_YELLOW);
  tft.fillRect(304, 0, 4, 240, ILI9341_GREEN);
  tft.fillRect(300, 0, 4, 240, ILI9341_CYAN);
  tft.fillRect(296, 0, 4, 240, ILI9341_BLUE);
  tft.fillRect(292, 0, 4, 240, ILI9341_MAGENTA);

  // URL + QR (centered, slightly smaller to fit in border)
  String url = getBestBaseURL();
  char urlBuf[96];
  snprintf(urlBuf, sizeof(urlBuf), "%s", url.c_str());

  const int16_t qrSize = 160;
  const int16_t qrX = (320 - qrSize) / 2;
  const int16_t qrY = 40;

  if (!drawQRCodeText(urlBuf, qrX, qrY, qrSize)) {
    tftBannerError("QR generator failed");
    return;
  }

  // Simple "Scan to Start" text centered below QR
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t textW = 13 * 12; // "Scan to Start" approx width
  tft.setCursor((320 - textW) / 2, qrY + qrSize + 10);
  tft.print("Scan to Start");
}

// ===================== Web UI (no CDN) =====================

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Whimsy Screen</title>
<style>
  :root{
    color-scheme: dark;
    --bg:#0a0a0f; --card:#12121a; --card2:#0d0d14;
    --text:#f0f0f5; --muted:#6b7280;
    --border:#1f1f2e; --accent:#6366f1; --accent2:#8b5cf6; --danger:#ef4444; --ok:#22c55e;
    --radius:16px;
  }
  *{box-sizing:border-box}
  html,body{height:100%;touch-action:manipulation}
  body{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;background:var(--bg);color:var(--text);display:flex;flex-direction:column}
  .wrap{flex:1;max-width:600px;margin:0 auto;padding:16px;padding-bottom:120px;width:100%;overflow-y:auto}
  
  /* Modern Header */
  .header{background:linear-gradient(135deg,var(--accent),var(--accent2));border-radius:var(--radius);padding:20px;margin-bottom:16px;position:relative;overflow:hidden}
  .header::before{content:"";position:absolute;top:-50%;right:-20%;width:200px;height:200px;background:rgba(255,255,255,0.1);border-radius:50%}
  .header-title{font-size:24px;font-weight:800;margin:0;letter-spacing:-0.5px}
  .header-sub{font-size:13px;opacity:0.9;margin-top:4px}
  .status-dot{position:absolute;top:20px;right:20px;width:12px;height:12px;background:#22c55e;border-radius:50%;box-shadow:0 0 10px #22c55e}
  .status-dot.off{background:var(--danger);box-shadow:0 0 10px var(--danger)}
  
  /* Photo Upload Area */
  .upload-area{background:var(--card);border:2px dashed var(--border);border-radius:var(--radius);padding:24px;text-align:center;transition:all 0.2s}
  .upload-area.active{border-color:var(--accent);background:var(--card2)}
  .upload-btn{display:inline-flex;align-items:center;gap:8px;background:var(--accent);color:white;border:none;border-radius:12px;padding:14px 24px;font-size:16px;font-weight:600;cursor:pointer;margin-bottom:12px}
  .upload-btn:active{transform:scale(0.98)}
  .camera-btn{display:inline-flex;align-items:center;justify-content:center;width:52px;height:52px;background:var(--card2);border:2px solid var(--border);border-radius:12px;font-size:24px;cursor:pointer;margin-left:8px;vertical-align:middle}
  .camera-btn:active{transform:scale(0.95)}
  
  /* Cards */
  .card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);padding:16px;margin-bottom:12px}
  .card-title{font-size:14px;font-weight:600;color:var(--muted);text-transform:uppercase;letter-spacing:0.5px;margin:0 0 12px 0}
  
  /* Preview */
  .preview-wrap{position:relative;border-radius:12px;overflow:hidden;background:#000}
  canvas{width:100%;height:auto;display:block;touch-action:none}
  .preview-hint{position:absolute;bottom:8px;left:8px;right:8px;text-align:center;font-size:12px;color:rgba(255,255,255,0.6);background:rgba(0,0,0,0.5);padding:6px;border-radius:6px;pointer-events:none}
  
  /* Controls Grid */
  .controls-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:12px}
  .control{min-width:0}
  .control label{display:block;font-size:11px;color:var(--muted);margin-bottom:4px;font-weight:500}
  .control select,.control input[type=range]{width:100%;height:44px;border-radius:10px;border:1px solid var(--border);background:var(--card2);color:var(--text);font-size:14px;padding:0 8px}
  .control input[type=range]{padding:0;height:44px;-webkit-appearance:none;background:transparent}
  .control input[type=range]::-webkit-slider-runnable-track{height:8px;background:var(--border);border-radius:4px}
  .control input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;height:24px;background:var(--accent);border-radius:50%;margin-top:-8px;box-shadow:0 2px 6px rgba(99,102,241,0.4)}
  
  /* Checkbox row */
  .checkbox-row{display:flex;gap:16px;flex-wrap:wrap;margin-top:12px}
  .checkbox-row label{display:flex;align-items:center;gap:8px;font-size:14px;cursor:pointer}
  .checkbox-row input[type=checkbox]{width:24px;height:24px;accent-color:var(--accent)}
  
  /* Send Button with Progress */
  .bar{position:fixed;left:0;right:0;bottom:0;background:var(--bg);border-top:1px solid var(--border);padding:16px env(safe-area-inset-right) calc(16px + env(safe-area-inset-bottom)) env(safe-area-inset-left);z-index:100}
  .send-btn{position:relative;width:100%;height:56px;background:linear-gradient(135deg,var(--accent),var(--accent2));border:none;border-radius:14px;color:white;font-size:18px;font-weight:700;cursor:pointer;overflow:hidden;transition:transform 0.1s}
  .send-btn:active:not(:disabled){transform:scale(0.98)}
  .send-btn:disabled{opacity:0.5;cursor:not-allowed;background:var(--border)}
  .send-btn-text{position:relative;z-index:2}
  .send-progress{position:absolute;left:0;top:0;height:100%;background:rgba(255,255,255,0.3);width:0%;transition:width 0.1s linear}
  .send-btn.sending .send-btn-text::after{content:"";display:inline-block;width:20px;height:20px;margin-left:8px;border:3px solid rgba(255,255,255,0.3);border-top-color:white;border-radius:50%;animation:spin 1s linear infinite;vertical-align:middle}
  @keyframes spin{to{transform:rotate(360deg)}}
  
  .bad{color:var(--danger)}
  .ok{color:var(--ok)}
  
  /* GIF Search */
  .gif-section{margin-top:12px;padding-top:12px;border-top:1px solid var(--border)}
  .gif-toggle{background:var(--card2);border:1px solid var(--border);border-radius:10px;padding:12px 16px;width:100%;color:var(--text);font-size:14px;cursor:pointer;display:flex;align-items:center;justify-content:space-between}
  .gif-toggle::after{content:"▼";font-size:10px;transition:transform 0.2s}
  .gif-toggle.open::after{transform:rotate(180deg)}
  .gif-panel{display:none;background:var(--card2);border:1px solid var(--border);border-top:none;border-radius:0 0 10px 10px;padding:12px}
  .gif-panel.open{display:block}
  .gif-search{display:flex;gap:8px;margin-bottom:12px}
  .gif-search input{flex:1;height:44px;border-radius:10px;border:1px solid var(--border);background:var(--bg);color:var(--text);padding:0 12px;font-size:14px}
  .gif-search button{height:44px;padding:0 20px;background:var(--accent);color:white;border:none;border-radius:10px;font-weight:600;cursor:pointer}
  .gif-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-height:300px;overflow-y:auto}
  .gif-item{aspect-ratio:1;background:var(--bg);border-radius:8px;overflow:hidden;cursor:pointer;position:relative}
  .gif-item img{width:100%;height:100%;object-fit:cover}
  .gif-item:hover::after{content:"";position:absolute;inset:0;background:var(--accent);opacity:0.3}
  .gif-loading{text-align:center;padding:20px;color:var(--muted)}
</style>
</head>
<body>
<div class="wrap">
  <!-- Modern Header -->
  <div class="header">
    <div class="header-title">Whimsy Screen</div>
    <div class="header-sub">Send photos to your display</div>
    <div class="status-dot" id="connDot"></div>
  </div>

  <!-- Photo Upload -->
  <div class="upload-area" id="uploadArea">
    <input id="file" type="file" accept="image/*" style="display:none">
    <button class="upload-btn" id="pickBtn">📁 Choose Photo</button>
    <button class="camera-btn" id="cameraBtn">📷</button>
    <div style="font-size:13px;color:var(--muted);margin-top:8px">or paste an image (Ctrl+V)</div>
    
    <!-- GIF Search Section -->
    <div class="gif-section">
      <button class="gif-toggle" id="gifToggle">🎬 Search GIFs</button>
      <div class="gif-panel" id="gifPanel">
        <div class="gif-search">
          <input type="text" id="gifQuery" placeholder="Search GIPHY..." value="funny">
          <button id="gifSearchBtn">Search</button>
        </div>
        <div class="gif-grid" id="gifGrid"></div>
      </div>
    </div>
  </div>

  <!-- Preview -->
  <div class="card" id="previewCard" style="display:none">
    <div class="card-title">Preview</div>
    <div class="preview-wrap">
      <canvas id="preview" width="320" height="240"></canvas>
      <div class="preview-hint" id="previewHint">Drag to pan • Pinch to zoom • Double-tap to reset</div>
    </div>
  </div>

  <!-- Adjustments -->
  <div class="card" id="adjustCard" style="display:none">
    <div class="card-title">Adjustments</div>
    <div class="controls-grid">
      <div class="control">
        <label>Filter</label>
        <select id="filter"><option value="none">None</option><option value="grayscale">B&W</option><option value="sepia">Sepia</option><option value="invert">Invert</option><option value="vintage">Vintage</option><option value="cool">Cool</option><option value="warm">Warm</option></select>
      </div>
      <div class="control">
        <label>Fit</label>
        <select id="fit"><option value="contain">Fit</option><option value="cover" selected>Fill</option></select>
      </div>
      <div class="control">
        <label>Rotate</label>
        <select id="rot"><option value="0">0°</option><option value="90">90°</option><option value="180">180°</option><option value="270">270°</option></select>
      </div>
    </div>
    <div class="controls-grid">
      <div class="control">
        <label>Brightness</label>
        <input id="bright" type="range" min="-60" max="60" value="0">
      </div>
      <div class="control">
        <label>Contrast</label>
        <input id="contrast" type="range" min="-50" max="50" value="0">
      </div>
      <div class="control">
        <label>Saturation</label>
        <input id="sat" type="range" min="0" max="200" value="100">
      </div>
    </div>
    <div class="controls-grid">
      <div class="control">
        <label>Zoom</label>
        <input id="zoom" type="range" min="0.7" max="2.2" step="0.01" value="1">
      </div>
    </div>
    <div class="checkbox-row">
      <label><input type="checkbox" id="mirror"> Mirror</label>
      <label><input type="checkbox" id="dither"> Dither</label>
      <label><input type="checkbox" id="q565" checked> TFT Preview</label>
    </div>
  </div>

  <div style="display:none"><progress id="prog" value="0" max="100"></progress><div id="hint"></div></div>
</div>

<div class="bar">
  <button class="send-btn" id="send" disabled>
    <div class="send-progress" id="sendProgress"></div>
    <span class="send-btn-text">Send to Screen</span>
  </button>
</div>

<script>
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);

  const connDot = $('connDot');
  const fileEl = $('file');
  const pickBtn = $('pickBtn');
  const cameraBtn = $('cameraBtn');
  const uploadArea = $('uploadArea');
  const previewCard = $('previewCard');
  const adjustCard = $('adjustCard');
  const sendBtn = $('send');
  const sendProgress = $('sendProgress');
  const previewHintEl = $('previewHint');

  // GIF elements
  const gifToggle = $('gifToggle');
  const gifPanel = $('gifPanel');
  const gifQuery = $('gifQuery');
  const gifSearchBtn = $('gifSearchBtn');
  const gifGrid = $('gifGrid');
  const GIPHY_API_KEY = 'SwhpwjpfvaK2RgOMsWlNzwa4VGAebBLB';

  const fitEl = $('fit');
  const rotEl = $('rot');
  const mirrorEl = $('mirror');
  const zoomEl = $('zoom');
  const brightEl = $('bright');
  const q565El = $('q565');
  const ditherEl = $('dither');
  const filterEl = $('filter');
  const contrastEl = $('contrast');
  const satEl = $('sat');

  const preview = $('preview');
  const pctx = preview.getContext('2d', { willReadFrequently: true });

  const img = new Image();
  let hasImage = false;

  const state = {
    fit: 'cover',
    rot: 0,
    mirror: 0,
    zoom: 1,
    dx: 0,
    dy: 0,
    bright: 0,
    q565: 1,
    dither: 0,
    filter: 'none',
    contrast: 0,
    sat: 100,
  };

  async function fetchStatus(){
    try {
      const r = await fetch('/api/status', { cache:'no-store' });
      const j = await r.json();
      const allGood = j.sdOK && j.heapFree > 5000;
      connDot.className = 'status-dot' + (allGood ? '' : ' off');
      connDot.title = j.sdOK ? 'Connected' : 'SD Error';
      sendBtn.disabled = !hasImage || !j.sdOK;
    } catch (e) {
      connDot.className = 'status-dot off';
      connDot.title = 'Connection lost';
    }
  }


  function drawImageTo(ctx, W, H) {
    ctx.save();
    ctx.clearRect(0,0,W,H);

    const rot = state.rot;
    const mirror = state.mirror;
    const zoom = state.zoom;

    const iw = img.naturalWidth || img.width;
    const ih = img.naturalHeight || img.height;

    ctx.translate(W/2, H/2);
    ctx.rotate(rot * Math.PI / 180);
    if (mirror) ctx.scale(-1, 1);

    const tw = (rot % 180 === 0) ? W : H;
    const th = (rot % 180 === 0) ? H : W;

    // Calculate base scale without zoom first
    const sContain = Math.min(tw/iw, th/ih);
    const sCover = Math.max(tw/iw, th/ih);
    const sBase = (state.fit === 'cover') ? sCover : sContain;

    // Apply zoom as a multiplier on top of base scale
    // This way zoom 1.0 = fit mode, zoom > 1 = zoom in, zoom < 1 = zoom out
    const s = sBase * zoom;
    const dw = iw * s;
    const dh = ih * s;

    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(img, -dw/2 + state.dx, -dh/2 + state.dy, dw, dh);

    ctx.restore();
  }

  function clamp8(x){ return x < 0 ? 0 : (x > 255 ? 255 : x|0); }

  function rgbToHsl(r, g, b) {
    r /= 255; g /= 255; b /= 255;
    const max = Math.max(r, g, b), min = Math.min(r, g, b);
    let h, s, l = (max + min) / 2;
    if (max === min) { h = s = 0; }
    else {
      const d = max - min;
      s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
      switch (max) {
        case r: h = (g - b) / d + (g < b ? 6 : 0); break;
        case g: h = (b - r) / d + 2; break;
        case b: h = (r - g) / d + 4; break;
      }
      h /= 6;
    }
    return [h, s, l];
  }

  function hslToRgb(h, s, l) {
    let r, g, b;
    if (s === 0) { r = g = b = l; }
    else {
      const hue2rgb = (p, q, t) => {
        if (t < 0) t += 1;
        if (t > 1) t -= 1;
        if (t < 1/6) return p + (q - p) * 6 * t;
        if (t < 1/2) return q;
        if (t < 2/3) return p + (q - p) * (2/3 - t) * 6;
        return p;
      };
      const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
      const p = 2 * l - q;
      r = hue2rgb(p, q, h + 1/3);
      g = hue2rgb(p, q, h);
      b = hue2rgb(p, q, h - 1/3);
    }
    return [r * 255, g * 255, b * 255];
  }

  function applyPost(ctx, W, H) {
    const imgData = ctx.getImageData(0,0,W,H);
    const d = imgData.data;

    const bright = state.bright|0;
    const q565 = !!state.q565;
    const dither = !!state.dither;
    const filter = state.filter;
    const contrast = state.contrast;
    const sat = state.sat / 100;

    // Contrast factor
    const contrastFactor = (259 * (contrast + 255)) / (255 * (259 - contrast));

    const bayer4 = [
      0,  8,  2, 10,
      12, 4, 14,  6,
      3, 11,  1,  9,
      15, 7, 13,  5
    ];

    for (let y=0; y<H; y++){
      for (let x=0; x<W; x++){
        const i = (y*W + x)*4;

        let r = d[i+0];
        let g = d[i+1];
        let b = d[i+2];

        // Apply brightness
        r += bright; g += bright; b += bright;

        // Apply contrast
        r = contrastFactor * (r - 128) + 128;
        g = contrastFactor * (g - 128) + 128;
        b = contrastFactor * (b - 128) + 128;

        // Apply saturation
        if (sat !== 1) {
          const gray = 0.299 * r + 0.587 * g + 0.114 * b;
          r = gray + (r - gray) * sat;
          g = gray + (g - gray) * sat;
          b = gray + (b - gray) * sat;
        }

        // Apply filters
        switch (filter) {
          case 'grayscale':
            const gray = 0.299 * r + 0.587 * g + 0.114 * b;
            r = g = b = gray;
            break;
          case 'sepia':
            const tr = 0.393 * r + 0.769 * g + 0.189 * b;
            const tg = 0.349 * r + 0.686 * g + 0.168 * b;
            const tb = 0.272 * r + 0.534 * g + 0.131 * b;
            r = tr; g = tg; b = tb;
            break;
          case 'invert':
            r = 255 - r; g = 255 - g; b = 255 - b;
            break;
          case 'vintage':
            r = r * 0.9 + 25; g = g * 0.85 + 15; b = b * 0.75 + 10;
            break;
          case 'cool':
            r *= 0.9; g *= 0.95; b *= 1.1;
            break;
          case 'warm':
            r *= 1.1; g *= 0.95; b *= 0.9;
            break;
        }

        if (q565) {
          if (dither) {
            const t = (bayer4[(y&3)*4 + (x&3)] - 7.5) / 16;
            r += t * 24; g += t * 24; b += t * 24;
          }

          r = clamp8(r); g = clamp8(g); b = clamp8(b);

          const r5 = (r * 31 + 127) / 255;
          const g6 = (g * 63 + 127) / 255;
          const b5 = (b * 31 + 127) / 255;

          r = (r5 * 255) / 31;
          g = (g6 * 255) / 63;
          b = (b5 * 255) / 31;
        } else {
          r = clamp8(r); g = clamp8(g); b = clamp8(b);
        }

        d[i+0]=r; d[i+1]=g; d[i+2]=b; d[i+3]=255;
      }
    }
    ctx.putImageData(imgData, 0, 0);
  }

  function updatePreview(){
    if (!hasImage) return;
    drawImageTo(pctx, preview.width, preview.height);
    applyPost(pctx, preview.width, preview.height);
  }

  function makeBMP24(canvas){
    const w = canvas.width, h = canvas.height;
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    const imgData = ctx.getImageData(0,0,w,h);
    const data = imgData.data;

    const rowSize = (w*3 + 3) & ~3;
    const pixelDataSize = rowSize * h;
    const fileSize = 54 + pixelDataSize;

    const buf = new ArrayBuffer(fileSize);
    const dv = new DataView(buf);
    let p = 0;

    const w8  = (v)=>{ dv.setUint8(p++, v & 255); };
    const w16 = (v)=>{ dv.setUint16(p, v & 65535, true); p+=2; };
    const w32 = (v)=>{ dv.setUint32(p, v >>> 0, true); p+=4; };
    const w32s= (v)=>{ dv.setInt32(p, v | 0, true); p+=4; };

    w8(0x42); w8(0x4D);
    w32(fileSize);
    w16(0); w16(0);
    w32(54);

    w32(40);
    w32s(w);
    w32s(-h); // top-down
    w16(1);
    w16(24);
    w32(0);
    w32(pixelDataSize);
    w32(2835); w32(2835);
    w32(0); w32(0);

    const pad = rowSize - (w*3);
    let idx = 0;

    for (let y=0; y<h; y++){
      for (let x=0; x<w; x++){
        const r = data[idx++], g = data[idx++], b = data[idx++]; idx++;
        w8(b); w8(g); w8(r);
      }
      for (let k=0; k<pad; k++) w8(0);
    }
    return buf;
  }

  function uploadBMP(arrayBuffer){
    const blob = new Blob([arrayBuffer], {type:'image/bmp'});
    const fd = new FormData();
    fd.append('image', blob, 'latest.bmp');

    return new Promise((resolve,reject)=>{
      const xhr = new XMLHttpRequest();
      xhr.open('POST','/upload',true);

      xhr.upload.onprogress = (e)=>{
        if (e.lengthComputable) {
          const pct = Math.round((e.loaded/e.total)*100);
          sendProgress.style.width = pct + '%';
        }
      };
      xhr.onload = ()=>{
        sendProgress.style.width = '0%';
        if (xhr.status === 200) resolve(xhr.responseText);
        else reject(new Error(xhr.responseText || ('HTTP ' + xhr.status)));
      };
      xhr.onerror = ()=>{
        sendProgress.style.width = '0%';
        reject(new Error('network error'));
      };
      xhr.send(fd);
    });
  }

  async function send(){
    if (!hasImage) return;

    sendBtn.classList.add('sending');
    sendBtn.disabled = true;
    const btnText = sendBtn.querySelector('.send-btn-text');
    const originalText = btnText.textContent;
    btnText.textContent = 'Sending…';

    try {
      const work = document.createElement('canvas');
      work.width = 320; work.height = 240;
      const wctx = work.getContext('2d', { willReadFrequently: true });

      drawImageTo(wctx, work.width, work.height);
      applyPost(wctx, work.width, work.height);

      const bmp = makeBMP24(work);
      await uploadBMP(bmp);

      try { await fetch('/api/draw', { method:'POST' }); } catch(_){}

      btnText.textContent = 'Sent!';
      setTimeout(() => { btnText.textContent = originalText; }, 1500);
    } catch(e) {
      btnText.textContent = 'Failed';
      setTimeout(() => { btnText.textContent = originalText; }, 2000);
    } finally {
      sendBtn.classList.remove('sending');
      sendBtn.disabled = !hasImage;
      await fetchStatus();
    }
  }

  function loadFile(f){
    hasImage = false;
    sendBtn.disabled = true;
    previewHintEl.textContent = 'Loading…';

    const url = URL.createObjectURL(f);
    img.onload = ()=>{
      URL.revokeObjectURL(url);
      hasImage = true;
      state.dx = 0; state.dy = 0; state.zoom = 1;
      zoomEl.value = 1;

      // Show preview and adjust cards
      previewCard.style.display = 'block';
      adjustCard.style.display = 'block';

      updatePreview();
      sendBtn.disabled = false;
      previewHintEl.textContent = 'Drag to pan • Pinch to zoom • Double-tap to reset';
    };
    img.onerror = ()=>{
      previewHintEl.textContent = 'Failed to load image';
      hasImage = false;
      previewCard.style.display = 'none';
      adjustCard.style.display = 'none';
    };
    img.src = url;
  }

  // Enhanced pinch-to-zoom with momentum and better tracking
  let pinchRafId = null;
  let targetZoom = 1;
  let currentZoom = 1;
  let initialPinchDist = 0;
  let initialZoom = 1;
  let lastPinchTime = 0;
  let pinchCenterX = 0;
  let pinchCenterY = 0;
  let initialDx = 0;
  let initialDy = 0;

  function lerp(start, end, factor) {
    return start + (end - start) * factor;
  }

  function animateZoom() {
    if (Math.abs(currentZoom - targetZoom) > 0.001) {
      currentZoom = lerp(currentZoom, targetZoom, 0.2);
      state.zoom = currentZoom;
      zoomEl.value = currentZoom;
      updatePreview();
      pinchRafId = requestAnimationFrame(animateZoom);
    } else {
      state.zoom = targetZoom;
      currentZoom = targetZoom;
      zoomEl.value = targetZoom;
      updatePreview();
      pinchRafId = null;
    }
  }

  preview.addEventListener('touchstart', (e)=>{
    if (!hasImage) return;

    if (e.touches.length === 2) {
      // Pinch start - capture initial state
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      initialPinchDist = Math.sqrt(dx*dx + dy*dy);
      initialZoom = state.zoom;
      currentZoom = state.zoom;
      targetZoom = state.zoom;
      lastPinchTime = Date.now();

      // Calculate pinch center for potential future use
      const rect = preview.getBoundingClientRect();
      pinchCenterX = ((e.touches[0].clientX + e.touches[1].clientX) / 2) - rect.left;
      pinchCenterY = ((e.touches[0].clientY + e.touches[1].clientY) / 2) - rect.top;

      // Store initial pan position
      initialDx = state.dx;
      initialDy = state.dy;
    }
  }, {passive:true});

  preview.addEventListener('touchmove', (e)=>{
    if (!hasImage || e.touches.length !== 2) return;
    e.preventDefault();

    const dx = e.touches[0].clientX - e.touches[1].clientX;
    const dy = e.touches[0].clientY - e.touches[1].clientY;
    const dist = Math.sqrt(dx*dx + dy*dy);

    if (initialPinchDist > 0) {
      const scale = dist / initialPinchDist;
      // Smoother zoom range: 0.5x to 4x
      targetZoom = Math.max(0.5, Math.min(4.0, initialZoom * scale));
      lastPinchTime = Date.now();

      if (!pinchRafId) {
        pinchRafId = requestAnimationFrame(animateZoom);
      }
    }
  }, {passive:false});

  preview.addEventListener('touchend', (e)=>{
    if (e.touches.length < 2) {
      initialPinchDist = 0;
      // Don't cancel animation immediately - let it settle
      if (pinchRafId && Date.now() - lastPinchTime > 100) {
        cancelAnimationFrame(pinchRafId);
        pinchRafId = null;
      }
    }
  });

  // drag-to-pan
  let dragging = false;
  let lastX = 0, lastY = 0;

  preview.addEventListener('pointerdown', (e)=>{
    if (!hasImage) return;
    dragging = true;
    preview.setPointerCapture(e.pointerId);
    lastX = e.clientX; lastY = e.clientY;
  });
  preview.addEventListener('pointermove', (e)=>{
    if (!dragging) return;
    const dx = (e.clientX - lastX);
    const dy = (e.clientY - lastY);
    lastX = e.clientX; lastY = e.clientY;

    state.dx += dx * (preview.width / preview.getBoundingClientRect().width);
    state.dy += dy * (preview.height / preview.getBoundingClientRect().height);

    updatePreview();
  });
  preview.addEventListener('pointerup', ()=>{ dragging = false; });

  // controls
  pickBtn.addEventListener('click', ()=> fileEl.click());
  fileEl.addEventListener('change', ()=>{
    const f = fileEl.files && fileEl.files[0];
    if (f) loadFile(f);
  });

  // Paste support
  document.addEventListener('paste', (e)=>{
    const items = e.clipboardData && e.clipboardData.items;
    if (!items) return;
    for (let i=0; i<items.length; i++) {
      if (items[i].type.indexOf('image') !== -1) {
        const blob = items[i].getAsFile();
        if (blob) loadFile(blob);
        break;
      }
    }
  });

  // Camera button - creates a temporary input with capture attribute
  cameraBtn.addEventListener('click', ()=>{
    const camInput = document.createElement('input');
    camInput.type = 'file';
    camInput.accept = 'image/*';
    camInput.capture = 'environment';
    camInput.style.display = 'none';
    camInput.addEventListener('change', ()=>{
      const f = camInput.files && camInput.files[0];
      if (f) {
        // Copy file to main input for consistency
        const dt = new DataTransfer();
        dt.items.add(f);
        fileEl.files = dt.files;
        loadFile(f);
      }
      document.body.removeChild(camInput);
    });
    document.body.appendChild(camInput);
    camInput.click();
  });

  // Handle select and range inputs
  ['fit','rot','zoom','bright','filter','contrast','sat'].forEach((k)=>{
    $(k).addEventListener('change', ()=>{
      state.fit = fitEl.value;
      state.rot = parseInt(rotEl.value,10)||0;
      state.zoom = parseFloat(zoomEl.value)||1;
      state.bright = parseInt(brightEl.value,10)||0;
      state.filter = filterEl.value;
      state.contrast = parseInt(contrastEl.value,10)||0;
      state.sat = parseInt(satEl.value,10)||100;
      if (hasImage) updatePreview();
    });
  });

  // Handle checkboxes
  mirrorEl.addEventListener('change', ()=>{
    state.mirror = mirrorEl.checked ? 1 : 0;
    if (hasImage) updatePreview();
  });
  ditherEl.addEventListener('change', ()=>{
    state.dither = ditherEl.checked ? 1 : 0;
    if (hasImage) updatePreview();
  });
  q565El.addEventListener('change', ()=>{
    state.q565 = q565El.checked ? 1 : 0;
    if (hasImage) updatePreview();
  });

  sendBtn.addEventListener('click', async ()=>{
    try {
      sendBtn.disabled = true;
      await send();
    } catch(e) {
      setHint('Send failed: ' + e.message, 'bad');
    } finally {
      sendBtn.disabled = !hasImage;
    }
  });


  // Double-tap to reset zoom/pan
  let lastTap = 0;
  preview.addEventListener('touchend', (e)=>{
    const now = Date.now();
    if (now - lastTap < 300) {
      // Double tap - reset with animation
      targetZoom = 1;
      currentZoom = state.zoom;
      state.dx = 0; state.dy = 0;
      state.rot = 0; rotEl.value = 0;
      if (!pinchRafId) {
        pinchRafId = requestAnimationFrame(animateZoom);
      }
    }
    lastTap = now;
  }, {passive:true});

  // Drag/drop on upload area
  uploadArea.addEventListener('dragover', (e)=>{ e.preventDefault(); uploadArea.classList.add('active'); });
  uploadArea.addEventListener('dragleave', ()=> uploadArea.classList.remove('active'));
  uploadArea.addEventListener('drop', (e)=>{
    e.preventDefault();
    uploadArea.classList.remove('active');
    const f = e.dataTransfer.files && e.dataTransfer.files[0];
    if (f) loadFile(f);
  });

  // GIF Search functionality
  gifToggle.addEventListener('click', ()=>{
    gifToggle.classList.toggle('open');
    gifPanel.classList.toggle('open');
    if (gifPanel.classList.contains('open') && gifGrid.children.length === 0) {
      searchGIFs('funny');
    }
  });

  async function searchGIFs(query) {
    gifGrid.innerHTML = '<div class="gif-loading">Loading...</div>';
    try {
      const url = `https://api.giphy.com/v1/gifs/search?api_key=${GIPHY_API_KEY}&q=${encodeURIComponent(query)}&limit=12&rating=g`;
      const r = await fetch(url);
      const data = await r.json();
      gifGrid.innerHTML = '';
      data.data.forEach(gif => {
        const div = document.createElement('div');
        div.className = 'gif-item';
        const img = document.createElement('img');
        img.src = gif.images.fixed_width_small.url;
        img.alt = gif.title;
        div.appendChild(img);
        div.addEventListener('click', ()=> loadGIFFromURL(gif.images.original.url));
        gifGrid.appendChild(div);
      });
    } catch(e) {
      gifGrid.innerHTML = '<div class="gif-loading">Failed to load GIFs</div>';
    }
  }

  async function loadGIFFromURL(url) {
    previewHintEl.textContent = 'Loading GIF...';
    try {
      const r = await fetch(url);
      const blob = await r.blob();
      // For now, just load first frame as static image
      // Full GIF animation would require frame extraction
      const imgUrl = URL.createObjectURL(blob);
      img.onload = ()=>{
        URL.revokeObjectURL(imgUrl);
        hasImage = true;
        state.dx = 0; state.dy = 0; state.zoom = 1;
        zoomEl.value = 1;
        previewCard.style.display = 'block';
        adjustCard.style.display = 'block';
        updatePreview();
        sendBtn.disabled = false;
        previewHintEl.textContent = 'GIF loaded (static frame)';
      };
      img.src = imgUrl;
    } catch(e) {
      previewHintEl.textContent = 'Failed to load GIF';
    }
  }

  gifSearchBtn.addEventListener('click', ()=>{
    const q = gifQuery.value.trim() || 'funny';
    searchGIFs(q);
  });

  gifQuery.addEventListener('keypress', (e)=>{
    if (e.key === 'Enter') gifSearchBtn.click();
  });

  fetchStatus();
  setInterval(fetchStatus, 5000);
})();
</script>
</body>
</html>
)HTML";

// ===================== HTTP handlers =====================

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleStatus() {
  const uint32_t upS = (millis() - uptimeStartMs) / 1000;

  String body = "{";
  body += "\"mode\":\""; body += (apMode ? "AP" : "STA"); body += "\",";
  body += "\"sdOK\":"; body += (sdOK ? "true" : "false"); body += ",";
  body += "\"hasImage\":"; body += (firstImageReceived ? "true" : "false"); body += ",";
  body += "\"lastBytes\":"; body += String(lastUploadBytes); body += ",";
  body += "\"uptimeS\":"; body += String(upS); body += ",";
  body += "\"heapFree\":"; body += String(ESP.getFreeHeap()); body += ",";
  body += "\"mdnsOK\":"; body += (mdnsOK ? "true" : "false"); body += ",";
  body += "\"baseUrl\":\""; body += getBestBaseURL(); body += "\",";
  body += "\"lastDraw\":\""; body += bmpResultToStr((BmpResult)lastBmpResult); body += "\"";
  body += "}";

  server.send(200, "application/json", body);
}

static void handleImageGet() {
  if (!sdOK || !SD.exists(BMP_PATH)) {
    server.send(404, "text/plain", "No image");
    return;
  }
  File f = SD.open(BMP_PATH, FILE_READ);
  if (!f) {
    server.send(500, "text/plain", "Open failed");
    return;
  }
  server.streamFile(f, "image/bmp");
  f.close();
}

static void handleDrawNow() {
  pendingDraw = true;
  server.send(200, "text/plain", "OK");
}

static void handleBootNow() {
  showBootScreen();
  server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"not found\"}");
}

// ===================== Upload (multipart stream) =====================

static File uploadFile;

static void handleUploadStream() {
  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    uploadError = false;
    uploadErrorMsg[0] = 0;
    lastUploadBytes = 0;

    logf("[UPLOAD] start filename=%s type=%s", up.filename.c_str(), up.type.c_str());

    if (!sdOK) sdOK = SD.begin(SD_CS);
    if (!sdOK) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD.begin failed");
      logf("[UPLOAD] SD.begin failed");
      return;
    }

    if (SD.exists(BMP_PATH)) SD.remove(BMP_PATH);
    uploadFile = SD.open(BMP_PATH, FILE_WRITE);
    if (!uploadFile) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD open for write failed");
      logf("[UPLOAD] open for write failed: %s", BMP_PATH);
    }
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadError) return;

    lastUploadBytes += up.currentSize;
    if (lastUploadBytes > MAX_UPLOAD_BYTES) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "Upload too large (>%lu)", (unsigned long)MAX_UPLOAD_BYTES);
      logf("[UPLOAD] too large: %lu", (unsigned long)lastUploadBytes);
      if (uploadFile) uploadFile.close();
      if (SD.exists(BMP_PATH)) SD.remove(BMP_PATH);
      return;
    }

    if (uploadFile) {
      size_t w = uploadFile.write(up.buf, up.currentSize);
      if (w != up.currentSize) {
        uploadError = true;
        snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD write failed");
        logf("[UPLOAD] write failed");
      }
    } else {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "No file handle");
      logf("[UPLOAD] no file handle");
    }
  }
  else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();

    if (uploadError) {
      logf("[UPLOAD] end ERROR: %s (bytes=%lu)", uploadErrorMsg, (unsigned long)lastUploadBytes);
      return;
    }

    logf("[UPLOAD] end OK bytes=%lu saved=%s", (unsigned long)lastUploadBytes, BMP_PATH);
    firstImageReceived = true;
    pendingDraw = true;
  }
  else if (up.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    uploadError = true;
    snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "Upload aborted");
    logf("[UPLOAD] aborted");
  }
}

static void handleUploadDone() {
  if (uploadError) {
    server.send(500, "text/plain", uploadErrorMsg[0] ? uploadErrorMsg : "upload failed");
  } else {
    server.send(200, "text/plain", "OK");
  }
}

// ===================== Setup / Loop =====================

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  logf("=== %s boot ===", DEVICE_NAME);

  uptimeStartMs = millis();

  pinMode(TFT_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  deselectAll();

  SPI.begin();
  SPI.setFrequency(8000000);
  logf("[SPI] 8MHz");

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  sdOK = SD.begin(SD_CS);
  logf("[SD] begin=%s", sdOK ? "OK" : "FAIL");

  connectWiFi();
  showBootScreen();

  // Routes
  server.on("/", HTTP_GET, handleRoot);

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/image.bmp", HTTP_GET, handleImageGet);
  server.on("/api/draw", HTTP_POST, handleDrawNow);
  server.on("/api/boot", HTTP_POST, handleBootNow);

  // Back-compat endpoints
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/upload", HTTP_POST, handleUploadDone, handleUploadStream);

  server.onNotFound(handleNotFound);

  server.begin();
  logf("[HTTP] server started");
}

void loop() {
  server.handleClient();
  if (mdnsOK) MDNS.update();

  if (pendingDraw) {
    pendingDraw = false;

    if (!sdOK) {
      tftBannerError("SD not initialized");
      logf("[DRAW] SD not OK at draw time");
      lastBmpResult = BMP_OPEN_FAIL;
      return;
    }

    if (!SD.exists(BMP_PATH)) {
      tftBannerError("BMP missing on SD");
      logf("[DRAW] file missing: %s", BMP_PATH);
      lastBmpResult = BMP_OPEN_FAIL;
      return;
    }

    logf("[DRAW] attempting draw: %s", BMP_PATH);

    BmpResult r = drawBMPFromSD_Safe(BMP_PATH);
    lastBmpResult = (int)r;

    if (r != BMP_OK) {
      logf("[DRAW] FAILED: %s", bmpResultToStr(r));
      char msg[96];
      snprintf(msg, sizeof(msg), "BMP draw failed: %s", bmpResultToStr(r));
      tftBannerError(msg);
    } else {
      logf("[DRAW] success");
    }
  }
}
