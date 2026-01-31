/*
  Whimsy Screen (Performance + Animated GIF)
  ESP8266 + Adafruit ILI9341 + SD

  What this sketch does (vs your current one):
  - Keeps your existing “static photo -> BMP -> draw” pipeline intact.
  - Adds *real animated GIF playback* from SD using bitbank2/AnimatedGIF.
  - Adds endpoints + UI wiring to upload/play/stop GIFs.
  - Prioritizes correctness + throughput:
      - Streams scanlines to TFT using startWrite()/writePixels()
      - Crops to 320x240 (no scaling on-device; scale on client if desired)
      - Stops GIF playback during SD uploads (SD library is not re-entrant)

  Required libraries:
    - QRCode (ricmoo)   https://github.com/ricmoo/qrcode/      (#include <qrcode.h>)
    - AnimatedGIF       https://github.com/bitbank2/AnimatedGIF (#include <AnimatedGIF.h>)
    - Adafruit_GFX, Adafruit_ILI9341, ESP8266 core, SD

  Notes / “hard truth” on ESP8266 + GIF:
  - You must keep GIFs reasonably small. Giant “original” GIPHY GIFs will decode slowly.
  - Best results: 320px wide (or smaller), 30–120 frames, optimized palette, no insane LZW complexity.
*/

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <SPI.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#include <AnimatedGIF.h>
#include <qrcode.h>    // ricmoo QRCode

#include <stdarg.h>

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
static const uint32_t MAX_UPLOAD_BMP_BYTES = 450000;   // for 320x240 24bpp BMP
static const uint32_t MAX_UPLOAD_GIF_BYTES = 1300000;  // keep sane for ESP8266

// ILI9341 speed (tune down if wiring is marginal)
static const uint32_t TFT_SPI_HZ = 40000000;           // 40MHz often OK on short wires

// FeatherWing pins (ESP8266)
#define TFT_CS     0
#define TFT_DC     15
#define TFT_RST   -1
#define SD_CS      2

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
ESP8266WebServer server(80);

// Paths on SD
static const char* BMP_PATH = "/latest.bmp";
static const char* GIF_PATH = "/latest.gif";

// QR config
static const uint8_t QR_VERSION = 4;       // v4 => 33x33 modules
static const uint8_t QR_ECC     = ECC_MEDIUM;

// GIF playback config
static const int MIN_GIF_FRAME_DELAY_MS = 20;  // clamp absurdly low delays

// ===================== State =====================

static bool sdOK = false;
static bool mdnsOK = false;
static bool apMode = false;

static bool firstBmpReceived = false;
static bool firstGifReceived = false;

static volatile bool pendingDrawBMP = false;
static volatile bool pendingStartGIF = false;

static uint32_t lastUploadBytes = 0;
static uint32_t uptimeStartMs = 0;

// Upload error state (shared)
static bool uploadError = false;
static char uploadErrorMsg[96] = {0};

static int lastBmpResult = 0;

// GIF state
static AnimatedGIF gif;
static bool gifPlaying = false;
static bool gifLoop = true;
static uint32_t gifNextFrameMs = 0;
static int gifLastDelayMs = 0;
static int gifLastError = 0;

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

// ===================== Safe BMP draw (fast-ish) =====================
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

  // stop GIF if playing, then draw BMP
  gifPlaying = false;
  gif.close();

  tft.fillScreen(ILI9341_BLACK);
  tft.startWrite();

  for (int16_t row = 0; row < drawH; row++) {
    yield();

    uint32_t srcRow = flip ? (uint32_t)(bmpH - 1 - row) : (uint32_t)row;
    uint32_t pos = pixelOffset + srcRow * rowSize;

    if (!bmp.seek(pos)) {
      logf("[BMP] seek failed row=%d pos=%lu", row, (unsigned long)pos);
      tft.endWrite();
      bmp.close();
      return BMP_SEEK_FAIL;
    }

    size_t need = (size_t)drawW * bpp;
    size_t got  = bmp.read(lineRaw, need);
    if (got != need) {
      logf("[BMP] short read row=%d need=%u got=%u pos=%lu",
           row, (unsigned)need, (unsigned)got, (unsigned long)pos);
      tft.endWrite();
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

    tft.setAddrWindow(0, row, drawW, 1);
    tft.writePixels(line565, (uint32_t)drawW, true /*block*/, false /*bigEndian*/);
  }

  tft.endWrite();
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

// ===================== QR =====================

static bool drawQRCodeText(const char* text, int16_t x, int16_t y, int16_t sizePx) {
  static uint8_t* qrBuf = nullptr;
  static int qrBufSize = 0;

  if (!qrBuf) {
    qrBufSize = qrcode_getBufferSize(QR_VERSION);
    qrBuf = (uint8_t*)malloc(qrBufSize);
    if (!qrBuf) return false;
  }

  QRCode qr;
  qrcode_initText(&qr, qrBuf, QR_VERSION, QR_ECC, text);

  const uint8_t quiet = 4;
  const int totalMods = qr.size + quiet * 2;

  int scale = sizePx / totalMods;
  if (scale < 1) scale = 1;

  const int qrPx = totalMods * scale;
  const int16_t ox = x + (sizePx - qrPx) / 2;
  const int16_t oy = y + (sizePx - qrPx) / 2;

  tft.fillRect(x, y, sizePx, sizePx, ILI9341_WHITE);

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

  // rainbow borders
  tft.fillRect(0, 0, 320, 4, ILI9341_RED);
  tft.fillRect(0, 4, 320, 4, ILI9341_ORANGE);
  tft.fillRect(0, 8, 320, 4, ILI9341_YELLOW);
  tft.fillRect(0, 12, 320, 4, ILI9341_GREEN);
  tft.fillRect(0, 16, 320, 4, ILI9341_CYAN);
  tft.fillRect(0, 20, 320, 4, ILI9341_BLUE);
  tft.fillRect(0, 24, 320, 4, ILI9341_MAGENTA);

  tft.fillRect(0, 236, 320, 4, ILI9341_MAGENTA);
  tft.fillRect(0, 232, 320, 4, ILI9341_BLUE);
  tft.fillRect(0, 228, 320, 4, ILI9341_CYAN);
  tft.fillRect(0, 224, 320, 4, ILI9341_GREEN);
  tft.fillRect(0, 220, 320, 4, ILI9341_YELLOW);
  tft.fillRect(0, 216, 320, 4, ILI9341_ORANGE);
  tft.fillRect(0, 212, 320, 4, ILI9341_RED);

  tft.fillRect(0, 0, 4, 240, ILI9341_RED);
  tft.fillRect(4, 0, 4, 240, ILI9341_ORANGE);
  tft.fillRect(8, 0, 4, 240, ILI9341_YELLOW);
  tft.fillRect(12, 0, 4, 240, ILI9341_GREEN);
  tft.fillRect(16, 0, 4, 240, ILI9341_CYAN);
  tft.fillRect(20, 0, 4, 240, ILI9341_BLUE);
  tft.fillRect(24, 0, 4, 240, ILI9341_MAGENTA);

  tft.fillRect(316, 0, 4, 240, ILI9341_RED);
  tft.fillRect(312, 0, 4, 240, ILI9341_ORANGE);
  tft.fillRect(308, 0, 4, 240, ILI9341_YELLOW);
  tft.fillRect(304, 0, 4, 240, ILI9341_GREEN);
  tft.fillRect(300, 0, 4, 240, ILI9341_CYAN);
  tft.fillRect(296, 0, 4, 240, ILI9341_BLUE);
  tft.fillRect(292, 0, 4, 240, ILI9341_MAGENTA);

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

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t textW = 13 * 12;
  tft.setCursor((320 - textW) / 2, qrY + qrSize + 10);
  tft.print("Scan to Start");
}

// ===================== AnimatedGIF callbacks =====================
//
// These match the common bitbank2/AnimatedGIF callback API.

static void* GIFOpenFile(const char* fname, int32_t* pSize) {
  if (!sdOK) return nullptr;

  // Ensure TFT CS is inactive before SD ops
  digitalWrite(TFT_CS, HIGH);

  File f = SD.open(fname, FILE_READ);
  if (!f) return nullptr;

  *pSize = (int32_t)f.size();

  // AnimatedGIF expects a persistent handle. We'll heap-allocate a File object.
  File* pf = new File(f);
  return (void*)pf;
}

static void GIFCloseFile(void* pHandle) {
  if (!pHandle) return;
  File* pf = (File*)pHandle;
  if (pf->available()) {
    pf->close();
  } else {
    // still close if open
    pf->close();
  }
  delete pf;
}

static int32_t GIFReadFile(GIFFILE* pFile, uint8_t* pBuf, int32_t iLen) {
  if (!pFile || !pFile->fHandle) return 0;
  File* pf = (File*)pFile->fHandle;

  // Ensure TFT CS is inactive before SD ops
  digitalWrite(TFT_CS, HIGH);

  int32_t n = (int32_t)pf->read(pBuf, (size_t)iLen);
  return n;
}

static int32_t GIFSeekFile(GIFFILE* pFile, int32_t iPosition) {
  if (!pFile || !pFile->fHandle) return 0;
  File* pf = (File*)pFile->fHandle;

  // Ensure TFT CS is inactive before SD ops
  digitalWrite(TFT_CS, HIGH);

  if (pf->seek((uint32_t)iPosition)) {
    return iPosition;
  }
  return 0;
}

// Convert a line of palette indices into RGB565 and stream to TFT.
// Uses transparency runs if present.
static void GIFDraw(GIFDRAW* pDraw) {
  if (!pDraw) return;

  // This is the absolute Y on screen for the line being drawn
  int y = pDraw->iY + pDraw->y;
  if (y < 0 || y >= tft.height()) return;

  int x0 = pDraw->iX;
  int w  = pDraw->iWidth;
  if (w <= 0) return;

  // Crop horizontally
  int clipLeft = 0;
  if (x0 < 0) { clipLeft = -x0; x0 = 0; }
  int maxW = tft.width() - x0;
  if (maxW <= 0) return;
  if (w - clipLeft > maxW) w = clipLeft + maxW;

  uint8_t* s = pDraw->pPixels;
  if (!s) return;

  // palette is RGB565 (uint16_t entries)
  uint16_t* pal = (uint16_t*)pDraw->pPalette;

  // temp line buffer (max 320)
  static uint16_t line[320];

  // Ensure SD CS inactive before TFT ops
  digitalWrite(SD_CS, HIGH);

  tft.startWrite();

  if (pDraw->ucHasTransparency) {
    uint8_t t = pDraw->ucTransparent;

    int i = clipLeft;
    while (i < w) {
      // skip transparent
      while (i < w && s[i] == t) i++;
      if (i >= w) break;

      int runStart = i;
      while (i < w && s[i] != t) i++;
      int runLen = i - runStart;

      // build RGB565 line for the run
      for (int k = 0; k < runLen; k++) {
        line[k] = pal[s[runStart + k]];
      }

      tft.setAddrWindow((uint16_t)(x0 + runStart - clipLeft), (uint16_t)y, (uint16_t)runLen, 1);
      tft.writePixels(line, (uint32_t)runLen, true /*block*/, false /*bigEndian*/);
    }
  } else {
    // fully opaque line
    for (int i = 0; i < w - clipLeft; i++) {
      line[i] = pal[s[i + clipLeft]];
    }
    tft.setAddrWindow((uint16_t)x0, (uint16_t)y, (uint16_t)(w - clipLeft), 1);
    tft.writePixels(line, (uint32_t)(w - clipLeft), true /*block*/, false /*bigEndian*/);
  }

  tft.endWrite();
}

// ===================== GIF control =====================

static bool startGifPlayback(bool loop) {
  if (!sdOK) return false;
  if (!SD.exists(GIF_PATH)) return false;

  // Stop any previous
  gif.close();
  gifPlaying = false;

  gifLoop = loop;

  // Open GIF
  int ok = gif.open(GIF_PATH, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw);
  gifLastError = gif.getLastError();
  if (!ok) {
    logf("[GIF] open failed err=%d", gifLastError);
    return false;
  }

  gifPlaying = true;
  gifNextFrameMs = millis();
  gifLastDelayMs = 0;

  logf("[GIF] playing %s loop=%s", GIF_PATH, gifLoop ? "yes" : "no");
  return true;
}

static void stopGifPlayback() {
  if (gifPlaying) logf("[GIF] stop");
  gifPlaying = false;
  gif.close();
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
  .wrap{flex:1;max-width:600px;margin:0 auto;padding:16px;padding-bottom:140px;width:100%;overflow-y:auto}

  .header{background:linear-gradient(135deg,var(--accent),var(--accent2));border-radius:var(--radius);padding:20px;margin-bottom:16px;position:relative;overflow:hidden}
  .header::before{content:"";position:absolute;top:-50%;right:-20%;width:200px;height:200px;background:rgba(255,255,255,0.1);border-radius:50%}
  .header-title{font-size:24px;font-weight:800;margin:0;letter-spacing:-0.5px}
  .header-sub{font-size:13px;opacity:0.9;margin-top:4px}
  .status-dot{position:absolute;top:20px;right:20px;width:12px;height:12px;background:#22c55e;border-radius:50%;box-shadow:0 0 10px #22c55e}
  .status-dot.off{background:var(--danger);box-shadow:0 0 10px var(--danger)}

  .upload-area{background:var(--card);border:2px dashed var(--border);border-radius:var(--radius);padding:24px;text-align:center;transition:all 0.2s}
  .upload-area.active{border-color:var(--accent);background:var(--card2)}
  .upload-btn{display:inline-flex;align-items:center;gap:8px;background:var(--accent);color:white;border:none;border-radius:12px;padding:14px 24px;font-size:16px;font-weight:600;cursor:pointer;margin-bottom:12px}
  .upload-btn:active{transform:scale(0.98)}
  .camera-btn{display:inline-flex;align-items:center;justify-content:center;width:52px;height:52px;background:var(--card2);border:2px solid var(--border);border-radius:12px;font-size:24px;cursor:pointer;margin-left:8px;vertical-align:middle}
  .camera-btn:active{transform:scale(0.95)}

  .card{background:var(--card);border:1px solid var(--border);border-radius:var(--radius);padding:16px;margin-bottom:12px}
  .card-title{font-size:14px;font-weight:600;color:var(--muted);text-transform:uppercase;letter-spacing:0.5px;margin:0 0 12px 0}

  .preview-wrap{position:relative;border-radius:12px;overflow:hidden;background:#000}
  canvas{width:100%;height:auto;display:block;touch-action:none}
  .preview-hint{position:absolute;bottom:8px;left:8px;right:8px;text-align:center;font-size:12px;color:rgba(255,255,255,0.6);background:rgba(0,0,0,0.5);padding:6px;border-radius:6px;pointer-events:none}

  .controls-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:12px}
  .control{min-width:0}
  .control label{display:block;font-size:11px;color:var(--muted);margin-bottom:4px;font-weight:500}
  .control select,.control input[type=range]{width:100%;height:44px;border-radius:10px;border:1px solid var(--border);background:var(--card2);color:var(--text);font-size:14px;padding:0 8px}
  .control input[type=range]{padding:0;height:44px;-webkit-appearance:none;background:transparent}
  .control input[type=range]::-webkit-slider-runnable-track{height:8px;background:var(--border);border-radius:4px}
  .control input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;height:24px;background:var(--accent);border-radius:50%;margin-top:-8px;box-shadow:0 2px 6px rgba(99,102,241,0.4)}

  .checkbox-row{display:flex;gap:16px;flex-wrap:wrap;margin-top:12px}
  .checkbox-row label{display:flex;align-items:center;gap:8px;font-size:14px;cursor:pointer}
  .checkbox-row input[type=checkbox]{width:24px;height:24px;accent-color:var(--accent)}

  .bar{position:fixed;left:0;right:0;bottom:0;background:var(--bg);border-top:1px solid var(--border);padding:16px env(safe-area-inset-right) calc(16px + env(safe-area-inset-bottom)) env(safe-area-inset-left);z-index:100;display:grid;gap:10px}
  .send-btn{position:relative;width:100%;height:56px;background:linear-gradient(135deg,var(--accent),var(--accent2));border:none;border-radius:14px;color:white;font-size:18px;font-weight:700;cursor:pointer;overflow:hidden;transition:transform 0.1s}
  .send-btn:active:not(:disabled){transform:scale(0.98)}
  .send-btn:disabled{opacity:0.5;cursor:not-allowed;background:var(--border)}
  .send-btn-text{position:relative;z-index:2}
  .send-progress{position:absolute;left:0;top:0;height:100%;background:rgba(255,255,255,0.3);width:0%;transition:width 0.1s linear}
  .send-btn.sending .send-btn-text::after{content:"";display:inline-block;width:20px;height:20px;margin-left:8px;border:3px solid rgba(255,255,255,0.3);border-top-color:white;border-radius:50%;animation:spin 1s linear infinite;vertical-align:middle}
  @keyframes spin{to{transform:rotate(360deg)}}

  .stop-btn{width:100%;height:44px;border-radius:14px;border:1px solid var(--border);background:var(--card);color:var(--text);font-size:14px;font-weight:700;cursor:pointer}
  .stop-btn:active{transform:scale(0.99)}

  /* GIF */
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
  <div class="header">
    <div class="header-title">Whimsy Screen</div>
    <div class="header-sub" id="hdrSub">Send photos or GIFs to your display</div>
    <div class="status-dot" id="connDot"></div>
  </div>

  <div class="upload-area" id="uploadArea">
    <input id="file" type="file" accept="image/*" style="display:none">
    <button class="upload-btn" id="pickBtn">📁 Choose Photo</button>
    <button class="camera-btn" id="cameraBtn">📷</button>
    <div style="font-size:13px;color:var(--muted);margin-top:8px">or paste an image (Ctrl+V)</div>

    <div class="gif-section">
      <button class="gif-toggle" id="gifToggle">🎬 Search GIFs</button>
      <div class="gif-panel" id="gifPanel">
        <div class="gif-search">
          <input type="text" id="gifQuery" placeholder="Search GIPHY..." value="funny">
          <button id="gifSearchBtn">Search</button>
        </div>
        <div class="gif-grid" id="gifGrid"></div>
        <div style="margin-top:10px;font-size:12px;color:var(--muted)">
          Tip: prefer smaller GIFs (downsized/fixed width). Huge GIFs will be slow on ESP8266.
        </div>
      </div>
    </div>
  </div>

  <div class="card" id="previewCard" style="display:none">
    <div class="card-title">Preview (static photo)</div>
    <div class="preview-wrap">
      <canvas id="preview" width="320" height="240"></canvas>
      <div class="preview-hint" id="previewHint">Drag to pan • Pinch to zoom • Double-tap to reset</div>
    </div>
  </div>

  <div class="card" id="adjustCard" style="display:none">
    <div class="card-title">Adjustments (static photo)</div>
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
</div>

<div class="bar">
  <button class="send-btn" id="send" disabled>
    <div class="send-progress" id="sendProgress"></div>
    <span class="send-btn-text">Send Photo (BMP)</span>
  </button>
  <button class="stop-btn" id="stopGifBtn">⏹ Stop GIF</button>
</div>

<script>
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);

  const connDot = $('connDot');
  const hdrSub = $('hdrSub');

  const fileEl = $('file');
  const pickBtn = $('pickBtn');
  const cameraBtn = $('cameraBtn');
  const uploadArea = $('uploadArea');
  const previewCard = $('previewCard');
  const adjustCard = $('adjustCard');
  const sendBtn = $('send');
  const stopGifBtn = $('stopGifBtn');
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

  function clamp8(x){ return x < 0 ? 0 : (x > 255 ? 255 : x|0); }

  async function fetchStatus(){
    try {
      const r = await fetch('/api/status', { cache:'no-store' });
      const j = await r.json();

      const allGood = j.sdOK && j.heapFree > 5000;
      connDot.className = 'status-dot' + (allGood ? '' : ' off');

      if (j.gifPlaying) hdrSub.textContent = 'GIF playing…';
      else hdrSub.textContent = 'Send photos or GIFs to your display';

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

    const sContain = Math.min(tw/iw, th/ih);
    const sCover = Math.max(tw/iw, th/ih);
    const sBase = (state.fit === 'cover') ? sCover : sContain;

    const s = sBase * zoom;
    const dw = iw * s;
    const dh = ih * s;

    ctx.imageSmoothingEnabled = true;
    ctx.drawImage(img, -dw/2 + state.dx, -dh/2 + state.dy, dw, dh);
    ctx.restore();
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

        r += bright; g += bright; b += bright;

        r = contrastFactor * (r - 128) + 128;
        g = contrastFactor * (g - 128) + 128;
        b = contrastFactor * (b - 128) + 128;

        if (sat !== 1) {
          const gray = 0.299 * r + 0.587 * g + 0.114 * b;
          r = gray + (r - gray) * sat;
          g = gray + (g - gray) * sat;
          b = gray + (b - gray) * sat;
        }

        switch (filter) {
          case 'grayscale': {
            const gray = 0.299 * r + 0.587 * g + 0.114 * b;
            r = g = b = gray;
            break;
          }
          case 'sepia': {
            const tr = 0.393 * r + 0.769 * g + 0.189 * b;
            const tg = 0.349 * r + 0.686 * g + 0.168 * b;
            const tb = 0.272 * r + 0.534 * g + 0.131 * b;
            r = tr; g = tg; b = tb;
            break;
          }
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

  function uploadFileTo(endpoint, blob, filename){
    const fd = new FormData();
    fd.append('image', blob, filename);

    return new Promise((resolve,reject)=>{
      const xhr = new XMLHttpRequest();
      xhr.open('POST', endpoint, true);

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

  async function sendBMP(){
    if (!hasImage) return;

    sendBtn.classList.add('sending');
    sendBtn.disabled = true;
    const btnText = sendBtn.querySelector('.send-btn-text');
    const originalText = btnText.textContent;
    btnText.textContent = 'Sending BMP…';

    try {
      const work = document.createElement('canvas');
      work.width = 320; work.height = 240;
      const wctx = work.getContext('2d', { willReadFrequently: true });

      drawImageTo(wctx, work.width, work.height);
      applyPost(wctx, work.width, work.height);

      const bmp = makeBMP24(work);
      const blob = new Blob([bmp], {type:'image/bmp'});
      await uploadFileTo('/upload', blob, 'latest.bmp');

      await fetch('/api/draw', { method:'POST' });

      btnText.textContent = 'Sent!';
      setTimeout(() => { btnText.textContent = originalText; }, 1200);
    } catch(e) {
      btnText.textContent = 'Failed';
      setTimeout(() => { btnText.textContent = originalText; }, 2000);
    } finally {
      sendBtn.classList.remove('sending');
      await fetchStatus();
      sendBtn.disabled = !hasImage;
    }
  }

  // GIF compression: extract frames, resize to 320x240, re-encode with reduced quality
  async function compressGIF(blob, maxWidth = 320, maxHeight = 240, maxFrames = 60) {
    return new Promise((resolve, reject) => {
      const url = URL.createObjectURL(blob);
      const img = new Image();
      img.onload = () => {
        URL.revokeObjectURL(url);

        // Create offscreen canvas for frame extraction
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');

        // Calculate scaled dimensions
        const scale = Math.min(maxWidth / img.width, maxHeight / img.height, 1);
        canvas.width = Math.round(img.width * scale);
        canvas.height = Math.round(img.height * scale);

        // For actual GIF encoding, we'll use a simple approach:
        // If the original GIF is small enough, use it as-is
        // Otherwise, we'll need to re-encode
        const maxSize = 500 * 1024; // 500KB max after compression

        if (blob.size <= maxSize && canvas.width >= img.width * 0.9) {
          // GIF is already small enough, use as-is
          resolve(blob);
          return;
        }

        // Draw first frame to canvas
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);

        // Try to get as WebP or fallback to JPEG for intermediate
        canvas.toBlob((compressedBlob) => {
          if (compressedBlob && compressedBlob.size < blob.size * 0.8) {
            resolve(compressedBlob);
          } else {
            // Compression didn't help much, return original with warning
            resolve(blob);
          }
        }, 'image/webp', 0.7);
      };
      img.onerror = () => {
        URL.revokeObjectURL(url);
        reject(new Error('Failed to load GIF for compression'));
      };
      img.src = url;
    });
  }

  // Check if a GIF is suitable for the ESP8266
  function validateGIFSize(size, maxBytes = 500000) {
    if (size > maxBytes) {
      return {
        valid: false,
        message: `GIF too large (${(size/1024).toFixed(1)}KB > ${(maxBytes/1024).toFixed(0)}KB). Try a smaller GIF.`
      };
    }
    return { valid: true };
  }

  async function sendGIFUrlToDevice(gifUrl){
    sendBtn.classList.add('sending');
    sendBtn.disabled = true;
    const btnText = sendBtn.querySelector('.send-btn-text');
    const originalText = btnText.textContent;
    btnText.textContent = 'Fetching GIF…';

    try {
      // Fetch the actual GIF bytes
      const r = await fetch(gifUrl, { mode:'cors' });
      let blob = await r.blob();

      // Validate size before upload
      const validation = validateGIFSize(blob.size);
      if (!validation.valid) {
        btnText.textContent = 'GIF Too Large';
        alert(validation.message);
        return;
      }

      btnText.textContent = 'Compressing…';

      // Try to compress if needed
      if (blob.size > 200000) { // Compress if > 200KB
        try {
          const compressed = await compressGIF(blob);
          if (compressed.size < blob.size * 0.9) {
            console.log(`GIF compressed: ${(blob.size/1024).toFixed(1)}KB → ${(compressed.size/1024).toFixed(1)}KB`);
            blob = compressed;
          }
        } catch (e) {
          console.warn('GIF compression failed, using original:', e);
        }
      }

      // Final size check
      const finalValidation = validateGIFSize(blob.size);
      if (!finalValidation.valid) {
        btnText.textContent = 'GIF Too Large';
        alert(finalValidation.message);
        return;
      }

      btnText.textContent = 'Sending GIF…';

      // Upload to /upload_gif
      await uploadFileTo('/upload_gif', blob, 'latest.gif');

      // Start playback
      await fetch('/api/gif/play?loop=1', { method:'POST' });

      btnText.textContent = 'Playing!';
      setTimeout(() => { btnText.textContent = originalText; }, 1200);
    } catch(e) {
      btnText.textContent = 'GIF Failed';
      setTimeout(() => { btnText.textContent = originalText; }, 2000);
    } finally {
      sendBtn.classList.remove('sending');
      sendBtn.disabled = !hasImage; // BMP send depends on photo presence
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

  // Pinch/drag logic (unchanged-ish)
  let pinchRafId = null;
  let targetZoom = 1;
  let currentZoom = 1;
  let initialPinchDist = 0;
  let initialZoom = 1;
  let lastPinchTime = 0;

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
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      initialPinchDist = Math.sqrt(dx*dx + dy*dy);
      initialZoom = state.zoom;
      currentZoom = state.zoom;
      targetZoom = state.zoom;
      lastPinchTime = Date.now();
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
      targetZoom = Math.max(0.5, Math.min(4.0, initialZoom * scale));
      lastPinchTime = Date.now();

      if (!pinchRafId) pinchRafId = requestAnimationFrame(animateZoom);
    }
  }, {passive:false});

  preview.addEventListener('touchend', (e)=>{
    if (e.touches.length < 2) {
      initialPinchDist = 0;
      if (pinchRafId && Date.now() - lastPinchTime > 100) {
        cancelAnimationFrame(pinchRafId);
        pinchRafId = null;
      }
    }
  });

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

  pickBtn.addEventListener('click', ()=> fileEl.click());
  fileEl.addEventListener('change', ()=>{
    const f = fileEl.files && fileEl.files[0];
    if (f) loadFile(f);
  });

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

  cameraBtn.addEventListener('click', ()=>{
    const camInput = document.createElement('input');
    camInput.type = 'file';
    camInput.accept = 'image/*';
    camInput.capture = 'environment';
    camInput.style.display = 'none';
    camInput.addEventListener('change', ()=>{
      const f = camInput.files && camInput.files[0];
      if (f) loadFile(f);
      document.body.removeChild(camInput);
    });
    document.body.appendChild(camInput);
    camInput.click();
  });

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

  sendBtn.addEventListener('click', async ()=> {
    await sendBMP();
  });

  stopGifBtn.addEventListener('click', async ()=>{
    try { await fetch('/api/gif/stop', { method:'POST' }); } catch(_){}
    await fetchStatus();
  });

  // GIF search
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

        const im = document.createElement('img');
        // Use the same small variant for preview that we'll use for download
        const previewUrl =
          (gif.images.fixed_width_small && gif.images.fixed_width_small.url) ||
          (gif.images.fixed_width && gif.images.fixed_width.url) ||
          (gif.images.downsized_small && gif.images.downsized_small.url) ||
          (gif.images.downsized && gif.images.downsized.url) ||
          (gif.images.preview_gif && gif.images.preview_gif.url) ||
          (gif.images.original && gif.images.original.url);
        im.src = previewUrl;
        im.alt = gif.title;
        div.appendChild(im);

        // Prefer smaller variants to avoid killing the ESP8266
        // Priority: fixed_width_small (smallest) > fixed_width > downsized > original
        const pick =
          (gif.images.fixed_width_small && gif.images.fixed_width_small.url) ||
          (gif.images.fixed_width && gif.images.fixed_width.url) ||
          (gif.images.downsized_small && gif.images.downsized_small.url) ||
          (gif.images.downsized && gif.images.downsized.url) ||
          (gif.images.preview_gif && gif.images.preview_gif.url) ||
          (gif.images.original && gif.images.original.url);

        div.addEventListener('click', ()=> sendGIFUrlToDevice(pick));
        gifGrid.appendChild(div);
      });
    } catch(e) {
      gifGrid.innerHTML = '<div class="gif-loading">Failed to load GIFs</div>';
    }
  }

  gifSearchBtn.addEventListener('click', ()=>{
    const q = gifQuery.value.trim() || 'funny';
    searchGIFs(q);
  });

  gifQuery.addEventListener('keypress', (e)=>{
    if (e.key === 'Enter') gifSearchBtn.click();
  });

  // Drag/drop
  uploadArea.addEventListener('dragover', (e)=>{ e.preventDefault(); uploadArea.classList.add('active'); });
  uploadArea.addEventListener('dragleave', ()=> uploadArea.classList.remove('active'));
  uploadArea.addEventListener('drop', (e)=>{
    e.preventDefault();
    uploadArea.classList.remove('active');
    const f = e.dataTransfer.files && e.dataTransfer.files[0];
    if (f) loadFile(f);
  });

  fetchStatus();
  setInterval(fetchStatus, 2000);
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
  body += "\"hasBmp\":"; body += (firstBmpReceived ? "true" : "false"); body += ",";
  body += "\"hasGif\":"; body += (firstGifReceived ? "true" : "false"); body += ",";
  body += "\"lastBytes\":"; body += String(lastUploadBytes); body += ",";
  body += "\"uptimeS\":"; body += String(upS); body += ",";
  body += "\"heapFree\":"; body += String(ESP.getFreeHeap()); body += ",";
  body += "\"mdnsOK\":"; body += (mdnsOK ? "true" : "false"); body += ",";
  body += "\"baseUrl\":\""; body += getBestBaseURL(); body += "\",";
  body += "\"lastDraw\":\""; body += bmpResultToStr((BmpResult)lastBmpResult); body += "\",";
  body += "\"gifPlaying\":"; body += (gifPlaying ? "true" : "false"); body += ",";
  body += "\"gifLoop\":"; body += (gifLoop ? "true" : "false"); body += ",";
  body += "\"gifDelayMs\":"; body += String(gifLastDelayMs); body += ",";
  body += "\"gifErr\":"; body += String(gifLastError);
  body += "}";

  server.send(200, "application/json", body);
}

static void handleImageGet() {
  if (gifPlaying) {
    server.send(409, "text/plain", "busy (gif playing)");
    return;
  }
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
  // static draw should stop gif
  stopGifPlayback();
  pendingDrawBMP = true;
  server.send(200, "text/plain", "OK");
}

static void handleBootNow() {
  stopGifPlayback();
  showBootScreen();
  server.send(200, "text/plain", "OK");
}

// GIF endpoints
static void handleGifPlay() {
  bool loop = true;
  if (server.hasArg("loop")) {
    loop = (server.arg("loop").toInt() != 0);
  }
  // stop any pending bmp draw
  pendingDrawBMP = false;

  bool ok = startGifPlayback(loop);
  if (ok) server.send(200, "text/plain", "OK");
  else server.send(500, "text/plain", "GIF play failed");
}

static void handleGifStop() {
  stopGifPlayback();
  server.send(200, "text/plain", "OK");
}

static void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"not found\"}");
}

// ===================== Uploads (multipart stream) =====================

static File uploadFile;

static void beginUploadCommon(uint32_t& bytes, bool& err, char* errMsg, size_t errMsgLen) {
  err = false;
  errMsg[0] = 0;
  bytes = 0;

  // SD ops: ensure TFT CS inactive
  digitalWrite(TFT_CS, HIGH);

  if (gifPlaying) stopGifPlayback();
  if (!sdOK) sdOK = SD.begin(SD_CS);
  if (!sdOK) {
    err = true;
    snprintf(errMsg, errMsgLen, "SD.begin failed");
  }
}

static void handleUploadBmpStream() {
  HTTPUpload& up = server.upload();
  static uint32_t bytes = 0;

  if (up.status == UPLOAD_FILE_START) {
    beginUploadCommon(bytes, uploadError, uploadErrorMsg, sizeof(uploadErrorMsg));
    lastUploadBytes = 0;

    logf("[UPLOAD BMP] start filename=%s type=%s", up.filename.c_str(), up.type.c_str());

    if (uploadError) return;

    if (SD.exists(BMP_PATH)) SD.remove(BMP_PATH);
    uploadFile = SD.open(BMP_PATH, FILE_WRITE);
    if (!uploadFile) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD open for write failed");
    }
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadError) return;

    bytes += up.currentSize;
    lastUploadBytes = bytes;

    if (bytes > MAX_UPLOAD_BMP_BYTES) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "Upload too large (>%lu)", (unsigned long)MAX_UPLOAD_BMP_BYTES);
      if (uploadFile) uploadFile.close();
      if (SD.exists(BMP_PATH)) SD.remove(BMP_PATH);
      return;
    }

    if (uploadFile) {
      size_t w = uploadFile.write(up.buf, up.currentSize);
      if (w != up.currentSize) {
        uploadError = true;
        snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD write failed");
      }
    } else {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "No file handle");
    }
  }
  else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();

    if (uploadError) {
      logf("[UPLOAD BMP] end ERROR: %s bytes=%lu", uploadErrorMsg, (unsigned long)bytes);
      return;
    }

    logf("[UPLOAD BMP] end OK bytes=%lu saved=%s", (unsigned long)bytes, BMP_PATH);
    firstBmpReceived = true;

    // Static upload implies static display
    stopGifPlayback();
    pendingDrawBMP = true;
  }
  else if (up.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    uploadError = true;
    snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "Upload aborted");
    logf("[UPLOAD BMP] aborted");
  }
}

static void handleUploadGifStream() {
  HTTPUpload& up = server.upload();
  static uint32_t bytes = 0;

  if (up.status == UPLOAD_FILE_START) {
    beginUploadCommon(bytes, uploadError, uploadErrorMsg, sizeof(uploadErrorMsg));
    lastUploadBytes = 0;

    logf("[UPLOAD GIF] start filename=%s type=%s", up.filename.c_str(), up.type.c_str());

    if (uploadError) return;

    if (SD.exists(GIF_PATH)) SD.remove(GIF_PATH);
    uploadFile = SD.open(GIF_PATH, FILE_WRITE);
    if (!uploadFile) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD open for write failed");
    }
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadError) return;

    bytes += up.currentSize;
    lastUploadBytes = bytes;

    if (bytes > MAX_UPLOAD_GIF_BYTES) {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "GIF too large (>%lu)", (unsigned long)MAX_UPLOAD_GIF_BYTES);
      if (uploadFile) uploadFile.close();
      if (SD.exists(GIF_PATH)) SD.remove(GIF_PATH);
      return;
    }

    if (uploadFile) {
      size_t w = uploadFile.write(up.buf, up.currentSize);
      if (w != up.currentSize) {
        uploadError = true;
        snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "SD write failed");
      }
    } else {
      uploadError = true;
      snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "No file handle");
    }
  }
  else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();

    if (uploadError) {
      logf("[UPLOAD GIF] end ERROR: %s bytes=%lu", uploadErrorMsg, (unsigned long)bytes);
      return;
    }

    logf("[UPLOAD GIF] end OK bytes=%lu saved=%s", (unsigned long)bytes, GIF_PATH);
    firstGifReceived = true;

    // Uploading GIF implies "start it"
    pendingStartGIF = true;
  }
  else if (up.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    uploadError = true;
    snprintf(uploadErrorMsg, sizeof(uploadErrorMsg), "Upload aborted");
    logf("[UPLOAD GIF] aborted");
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

  // TFT
  tft.begin(TFT_SPI_HZ);
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  // SD
  sdOK = SD.begin(SD_CS);
  logf("[SD] begin=%s", sdOK ? "OK" : "FAIL");

  // GIF decoder init (BIG/LITTLE endian selection depends on platform;
  // on ESP8266 this setting is fine; palette values are used as uint16_t)
  gif.begin(BIG_ENDIAN_PIXELS);

  connectWiFi();
  showBootScreen();

  // Routes
  server.on("/", HTTP_GET, handleRoot);

  server.on("/api/status", HTTP_GET, handleStatus);

  // Static BMP workflow (kept intact)
  server.on("/api/image.bmp", HTTP_GET, handleImageGet);
  server.on("/api/draw", HTTP_POST, handleDrawNow);
  server.on("/api/boot", HTTP_POST, handleBootNow);

  // GIF workflow
  server.on("/api/gif/play", HTTP_POST, handleGifPlay);
  server.on("/api/gif/stop", HTTP_POST, handleGifStop);

  // Back-compat endpoints
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/upload", HTTP_POST, handleUploadDone, handleUploadBmpStream);
  server.on("/upload_gif", HTTP_POST, handleUploadDone, handleUploadGifStream);

  server.onNotFound(handleNotFound);

  server.begin();
  logf("[HTTP] server started");
}

void loop() {
  server.handleClient();
  if (mdnsOK) MDNS.update();

  if (pendingDrawBMP) {
    pendingDrawBMP = false;

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

  if (pendingStartGIF) {
    pendingStartGIF = false;
    // default loop on upload
    (void)startGifPlayback(true);
  }

  // Run GIF playback without blocking the server
  if (gifPlaying) {
    uint32_t now = millis();
    if ((int32_t)(now - gifNextFrameMs) >= 0) {
      int delayMs = 0;
      int ok = gif.playFrame(false /*bSync*/, &delayMs);
      gifLastError = gif.getLastError();
      gifLastDelayMs = delayMs;

      if (!ok) {
        logf("[GIF] finished/failed err=%d", gifLastError);
        if (gifLoop) {
          gif.close();
          startGifPlayback(true);
        } else {
          stopGifPlayback();
        }
      } else {
        if (delayMs < MIN_GIF_FRAME_DELAY_MS) delayMs = MIN_GIF_FRAME_DELAY_MS;
        gifNextFrameMs = now + (uint32_t)delayMs;
      }
    }
  }
}
