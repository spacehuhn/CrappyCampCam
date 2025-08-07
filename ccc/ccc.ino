#include <SD_MMC.h> // SD Card SD_MMC
#include <USB.h>    // USB
#include <USBMSC.h> // USB mass storage
#include <Adafruit_NeoPixel.h> // LED Adafruit_NeoPixel

#include "esp_camera.h"     // Camera camera_config_t
#include "esp_heap_caps.h"  // Check PSRAM usage
#include "esp_log.h"        // Logging

// Sleep & power-saving
extern "C" {
  #include "esp_sleep.h"
  #include "driver/gpio.h"
}

// Button and buzzer pins
#define BUTTON_PIN 2
#define BUZZ_PIN 14
#define LED_PIN 21

// SD Pins (SDMMC 1-bit)
#define SD_CLK 39
#define SD_CMD 38
#define SD_D0 40
#define SD_ONEBIT true

// Camera pin definitions (ESP32-S3-EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#if !SOC_USB_OTG_SUPPORTED || ARDUINO_USB_MODE
#error Device does not support USB_OTG or native USB CDC/JTAG is selected
#endif

USBMSC msc;
Adafruit_NeoPixel led { 1, LED_PIN, NEO_GRBW + NEO_KHZ800 };
RTC_DATA_ATTR int photo_num { 0 };

static unsigned long last_ms { 0 }; // Last time a photo was taken using millis()
//const unsigned long IDLE_TIMEOUT_MS { 300'000 }; // 5min timeout (for testing, didn't work)
const unsigned long IDLE_WARN_BEEP_MS { 300'000 }; // Beep every 5 min to remind user to turn off

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  uint32_t secSize = SD_MMC.sectorSize();
  if (!secSize) {
    return false;  // disk error
  }
  log_v("Write lba: %ld\toffset: %ld\tbufsize: %ld", lba, offset, bufsize);
  for (int x = 0; x < bufsize / secSize; x++) {
    uint8_t blkbuffer[secSize];
    memcpy(blkbuffer, (uint8_t *)buffer + secSize * x, secSize);
    if (!SD_MMC.writeRAW(blkbuffer, lba + x)) {
      return false;
    }
  }
  return bufsize;
}

static int32_t onRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  uint32_t secSize = SD_MMC.sectorSize();
  if (!secSize) {
    return false;  // disk error
  }
  log_v("Read lba: %ld\toffset: %ld\tbufsize: %ld\tsector: %lu", lba, offset, bufsize, secSize);
  for (int x = 0; x < bufsize / secSize; x++) {
    if (!SD_MMC.readRAW((uint8_t *)buffer + (x * secSize), lba + x)) {
      return false;  // outside of volume boundary
    }
  }
  return bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  log_i("Start/Stop power: %u\tstart: %d\teject: %d", power_condition, start, load_eject);
  return true;
}

static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT: Serial.println("USB PLUGGED"); break;
      case ARDUINO_USB_STOPPED_EVENT: Serial.println("USB UNPLUGGED"); break;
      case ARDUINO_USB_SUSPEND_EVENT: Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en); break;
      case ARDUINO_USB_RESUME_EVENT:  Serial.println("USB RESUMED"); break;

      default: break;
    }
  }
}

int count_photos() {
  File root { SD_MMC.open("/") };
  int num { 0 };

  if (!root || !root.isDirectory()) {
    Serial.println("[SD] Failed to open root directory");
    return -1;
  }

  while (true) {
    File file = root.openNextFile();
    if (!file) break;
    if (file.isDirectory()) continue;

    ++num;

    file.close();
  }

  Serial.printf("Photo number: %d\n", num);
  return num;
}

void go_deep_sleep() {
  Serial.println("Entering deep-sleep…");

  // De-init camera & quiet pins to save uA
  esp_camera_deinit();
  gpio_set_direction((gpio_num_t)XCLK_GPIO_NUM, GPIO_MODE_INPUT);

  // Turn off LED and buzzer
  led.clear(); 
  led.show();
  noTone(BUZZ_PIN);

  // Wake on button press (0 = wake on LOW)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);

  esp_deep_sleep_start();
}

// Write JPEG with an added COM segment: "Crappy Camp Cam ..."
static bool write_jpeg_with_comment(File &f, const uint8_t *buf, size_t len, const char *comment) {
    if (len < 2 || buf[0] != 0xFF || buf[1] != 0xD8) return false; // not a JPEG

    // Build COM header: 0xFF 0xFE, 16-bit big-endian length = payload + 2
    size_t c_len = strlen(comment);
    if (c_len > 65533) c_len = 65533;  // max payload so length fits in 16 bits

    uint8_t hdr[4] = { 0xFF, 0xFE, uint8_t(((c_len + 2) >> 8) & 0xFF), uint8_t((c_len + 2) & 0xFF) };

    // 1) SOI
    if (f.write(buf, 2) != 2) return false;

    // 2) COM segment
    if (f.write(hdr, 4) != 4) return false;
    if (f.write((const uint8_t*)comment, c_len) != c_len) return false;

    // 3) Rest of original JPEG
    if (f.write(buf + 2, len - 2) != (len - 2)) return false;

    return true;
}

void cam_fail() {
  led.setPixelColor(0, 255,0,0);
  led.show();

  while (true) {
    Serial.println("Camera init failed");

    tone(BUZZ_PIN, 400);
    delay(200);
    noTone(BUZZ_PIN);
    delay(200);
  }
}

void sd_fail() {
  bool toggle { false };

  while (true) {
    Serial.println("SD_MMC Mount Failed!");

    led.setPixelColor(0, 0,toggle?255:0,0);
    led.show();

    tone(BUZZ_PIN, 400);
    delay(200);
    noTone(BUZZ_PIN);

    toggle = !toggle;
    delay(1000);
  }
}

void takePhoto() {
  // Beep
  tone(BUZZ_PIN, 400);
  delay(77);
  noTone(BUZZ_PIN);

  // LED flash
  led.setBrightness(255);
  led.setPixelColor(0, 255,255,255);
  led.show();
  delay(17);
  led.setBrightness(20);
  led.setPixelColor(0, 0,0,255);
  led.show();

  // Take Photo
  Serial.println("Taking Photo");
  camera_fb_t *fb { nullptr };
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed!");
    led.setPixelColor(0, 255,0,0);
    led.show();
    return;
  }

  // Filename
  char filename[32];
  do {
    sprintf(filename, "/ccc_%d.jpg", ++photo_num);
  } while(SD_MMC.exists(filename));

  // Open file and write photo
  File file { SD_MMC.open(filename, FILE_WRITE) };

  if(file){
    Serial.printf("Saving as %s\n", filename);
  } else {
    Serial.printf("Failed to open %s for writing.\n", filename);
    sd_fail();
  }

  // Add Tag (JPEG Comment) for crappy camp cam
  if (!write_jpeg_with_comment(file, fb->buf, fb->len, "Crappy Camp Cam | FW 1.0")) {
    // Fallback: write as usual (without tag)
    file.write(fb->buf, fb->len);
  }
  file.close();
  Serial.printf("Saved %s (%d bytes)\n", filename, fb->len);

  // Free camera buffer
  esp_camera_fb_return(fb);

  // LED green
  led.setPixelColor(0, 0,255,0);
  led.show();

  // Boop
  tone(BUZZ_PIN, 800);
  delay(77);
  noTone(BUZZ_PIN);

  Serial.println("Photo saved.");
}

void setup() {
  // Start Serial
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Starting Crappy Camp Cam");
  Serial.printf("PSRAM free: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

  // LED
  led.begin();
  led.setBrightness(20);
  led.setPixelColor(0, 0,0,255);
  led.show();

  // ----- Init Camera ----- //
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_1;
  config.ledc_timer   = LEDC_TIMER_1;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QSXGA; // Set biggest possible frame_size for max. buffer size
  config.jpeg_quality = 8; // Smaller values have proven instable
  config.fb_count     = 1;

  if (esp_camera_init(&config) != ESP_OK) cam_fail();

  sensor_t *s = esp_camera_sensor_get();
  if(!s) cam_fail();
  s->set_vflip(s, 1);     // Flip vertically
  //s->set_hmirror(s, 1);   // Flip horizontally
  if(s->id.PID == OV2640_PID) {
    s->set_framesize(s, FRAMESIZE_UXGA); // 1600 × 1200 (1.9MP)
    Serial.println("Detected OV2640, decreasing resolution to 1600 × 1200 (1.9MP)");
  } else {
    s->set_framesize(s, FRAMESIZE_QXGA); // 2048 × 1536 (3.1 MP) 
    Serial.println("Set resolution to 2048 × 1536 (3.1 MP)");
  }

  // ---- Init SD card ----
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, -1, -1, -1);

  if (!SD_MMC.begin("/sdcard", true)) sd_fail();

  photo_num = count_photos();

  Serial.printf("SD card OK. Size: %lluMB\n", SD_MMC.totalBytes() / 1024 / 1024);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ----- USB MSC ----- //
  if(digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("Checking button press to enable USB-MSC");
    delay(59);
    if(digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("USB-MSC enabled!");
        // Initialize USB metadata and callbacks for MSC (Mass Storage Class)
        msc.vendorID("ESP32");
        msc.productID("USB_MSC");
        msc.productRevision("1.0");
        msc.onRead(onRead);
        msc.onWrite(onWrite);
        msc.onStartStop(onStartStop);
        msc.mediaPresent(true);
        msc.begin(SD_MMC.numSectors(), SD_MMC.sectorSize());

        Serial.println("Initializing USB");

        USB.begin();
        USB.onEvent(usbEventCallback);

        Serial.printf("Card Size: %lluMB\n", SD_MMC.totalBytes() / 1024 / 1024);
        Serial.printf("Sector: %d\tCount: %d\n", SD_MMC.sectorSize(), SD_MMC.numSectors());

        led.setPixelColor(0, 255,255,255);
        led.show();

        tone(BUZZ_PIN, 400);
        delay(33);
        noTone(BUZZ_PIN);

        while(true) delay(-1);
    } else {
      Serial.println("USB-MSC not enabled");
    }
  }

  // Use button for wake-up
  gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // If woke up from deep-sleep, take photo
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up from deep sleep");
    takePhoto();
    return;
  }

  // ----- Finished setup ----- //
  led.setPixelColor(0, 0,255,0);
  led.show();

  tone(BUZZ_PIN, 400);
  delay(33);
  tone(BUZZ_PIN, 500);
  delay(33);
  tone(BUZZ_PIN, 600);
  delay(55);
  noTone(BUZZ_PIN);

  last_ms = millis();
  Serial.println("Ready");
}

void loop() {
  // Deep-sleep
  //if (millis() - last_ms > IDLE_TIMEOUT_MS) go_deep_sleep();

  // Idle Warn Beep
  if(millis() - last_ms > IDLE_WARN_BEEP_MS) {
    last_ms = millis();
    led.setBrightness(255);

    for(int i=0; i<3; ++i) {
      tone(BUZZ_PIN, 400);
      delay(50);
      noTone(BUZZ_PIN);

      led.setPixelColor(0, 255,255,255);
      led.show();

      tone(BUZZ_PIN, 500);
      delay(50);
      noTone(BUZZ_PIN);

      led.setPixelColor(0, 0,0,0);
      led.show();

      delay(200);
    }

    led.setBrightness(20);
  }
  
  // Debounced shutter button
  if (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW) {
          last_ms = millis();
          takePhoto();
      }
  }

  led.setBrightness(2);
  led.setPixelColor(0, 0,255,0);
  led.show();
  esp_sleep_enable_timer_wakeup(30'000'000ULL); // 30 s
  esp_light_sleep_start(); // Enter light sleep between photos
}