#include <Arduino.h>
#include <WS2812.h>

/*-------------------------彩條燈定義-------------------------*/
#define NUM_LEDS 8
#define COLOR_PER_LEDS 3
#define NUM_BYTES (NUM_LEDS * COLOR_PER_LEDS)

#if NUM_BYTES > 255
#error "NUM_BYTES can not be larger than 255."
#endif

#define lightStrip 31 // WS2812 資料腳位

__xdata uint8_t ledData[NUM_BYTES]; // 燈條資料緩衝區

const int MODE_LENGTH = 6;

/*  command
    0: 白色  
    1: 紅色
    2: 綠色
    3: 藍色
    4: 彩色
    5: 隨機
*/

int command = 0;
bool power = false;
bool rainbow_init = false;


/*-------------------------語音定義-------------------------*/

#define SDA_PIN 32 // 定義 SDA 為 P3.2
#define SCL_PIN 14  // 定義 SCL 為 P1.4

#define I2C_DELAY_US 5  // I²C 時鐘延遲 (調整延遲來控制速率)

// 語音設備 I²C 位址
#define VOICE_I2C_ADDR 0x0B

// 學習指令及恢復出場指令
const uint8_t  STUDY_COMMAND = 0x50;
const uint8_t  INIT_COMMAND  = 0x60;


/*-------------------------彩條燈函式區-------------------------*/
// 初始化彩條燈腳位
void pinInit() {
    pinMode(lightStrip, OUTPUT); // 設置燈條腳位為輸出
}

void update_GRB_LED(int R, int G, int B){
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      set_pixel_for_GRB_LED(ledData, i, R, G, B);
    }
    neopixel_show_P3_1(ledData, NUM_BYTES); // 更新 WS2812 燈條
}


void initializeRainbow(__xdata uint8_t *ledData) {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        uint8_t r = (i * 85) % 256;  // 紅色逐漸增強
        uint8_t g = ((i * 85) + 85) % 256; // 綠色逐漸增強
        uint8_t b = ((i * 85) + 170) % 256; // 藍色逐漸增強
        set_pixel_for_GRB_LED(ledData, i, r, g, b);
    }
}

void shiftRainbow(__xdata uint8_t *ledData) {
    uint8_t lastR = ledData[(NUM_LEDS - 1) * COLOR_PER_LEDS];
    uint8_t lastG = ledData[(NUM_LEDS - 1) * COLOR_PER_LEDS + 1];
    uint8_t lastB = ledData[(NUM_LEDS - 1) * COLOR_PER_LEDS + 2];

    for (int8_t i = NUM_LEDS - 1; i > 0; i--) {
        ledData[i * COLOR_PER_LEDS] = ledData[(i - 1) * COLOR_PER_LEDS];
        ledData[i * COLOR_PER_LEDS + 1] = ledData[(i - 1) * COLOR_PER_LEDS + 1];
        ledData[i * COLOR_PER_LEDS + 2] = ledData[(i - 1) * COLOR_PER_LEDS + 2];
    }

    ledData[0] = lastR;
    ledData[1] = lastG;
    ledData[2] = lastB;
}
/*-------------------------I²C函式區-------------------------*/
// 初始化 I²C 線路
void i2c_init() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}

// 產生 I²C 起始信號
void i2c_start() {
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
}

// 產生 I²C 停止信號
void i2c_stop() {
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
}

// 傳送 1 個位元
void i2c_send_bit(bool bit) {
  digitalWrite(SDA_PIN, bit);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
}

// 接收 1 個位元
bool i2c_read_bit() {
  pinMode(SDA_PIN, INPUT);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  bool bit = digitalRead(SDA_PIN);
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);
  return bit;
}

// 傳送 1 個位元組 (8 位元) 並讀取 ACK/NACK
bool i2c_write_byte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    i2c_send_bit(data & 0x80);
    data <<= 1;
  }
  // 接收 ACK/NACK
  return !i2c_read_bit();  // 0: ACK, 1: NACK
}

// 讀取 1 個位元組 (8 位元)
uint8_t i2c_read_byte(bool ack) {
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    if (i2c_read_bit()) {
      data |= 0x01;
    }
  }
  i2c_send_bit(!ack);  // 傳送 ACK (0) 或 NACK (1)
  return data;
}

// 寫入資料到 I²C 裝置
bool i2c_write(uint8_t addr, uint8_t *data, uint8_t len) {
  i2c_start();
  if (!i2c_write_byte(addr << 1)) {  // 傳送寫入位址
    USBSerial_print("位置寫入失敗\n");
    i2c_stop();
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    if (!i2c_write_byte(data[i])) {  // 傳送資料
      USBSerial_print("資料傳入失敗\n");
      i2c_stop();
      return false;
    }
  }
  i2c_stop();
  return true;
}

// 從 I²C 裝置讀取資料
bool i2c_read(uint8_t addr, uint8_t *buffer, uint8_t len) {
  i2c_start();
  if (!i2c_write_byte((addr << 1) | 1)) {  // 傳送讀取位址
    i2c_stop();
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = i2c_read_byte(i < (len - 1));  // 除了最後一個位元組外，都傳送 ACK
  }
  i2c_stop();
  return true;
}

// 讀取語音 I²C 設備的資料
bool read_voice_data(uint8_t *buffer, uint8_t len) {
  return i2c_read(VOICE_I2C_ADDR, buffer, len);  // 從語音設備讀取資料
}

/*-------------------------語音訊息函式區-------------------------*/
void data_process(const uint8_t data) {
  // 開啟燈條
  if (data == 0x50) {
    power = true;
  }

  // 關閉燈條
  else if (data == 0x51) {
    rainbow_init = false;
    power = false;
  }

  // 切換模式
  else if (data == 0x52) {
    command = (command + 1) % MODE_LENGTH;
  }
}

void setup() {
  // 初始化I²C
  i2c_init();
  pinInit();
  //延遲1秒
  delay(5000);
  //i2c_write(VOICE_I2C_ADDR, &STUDY_COMMAND, 1);
}

void loop() {
  uint8_t voice_data[2];  // 用來接收語音設備的資料

  // 從語音設備讀取資料
  if (read_voice_data(voice_data, 1)) {
    USBSerial_print("Voice Data: ");
    USBSerial_println(voice_data[0], HEX);  // 顯示語音設備的第1個位元組
  } else {
    USBSerial_print("Failed to read voice data\n");
  }
  if(voice_data[0] != 0x00){
    data_process(voice_data[0]);
  }

  if(power == true){
    // 白燈
    if(command == 0){
      update_GRB_LED(255, 255, 255);
    }
    // 紅燈
    else if(command == 1){
      update_GRB_LED(255, 0, 0);
    }
    // 綠燈
    else if(command == 2){
      update_GRB_LED(0, 255, 0);
    }
    // 藍燈
    else if(command == 3){
      update_GRB_LED(0, 0, 255);
    }
    // 彩色
    else if(command == 4){
      if(!rainbow_init){
        rainbow_init = true;
        initializeRainbow(ledData);
      }else{
      shiftRainbow(ledData);
      neopixel_show_P3_1(ledData, NUM_BYTES);
      }
    }
    else if(command == 5){
      rainbow_init = false;
      int randomR = random(255); // 生成 0 到 255 之間的隨機紅色值
      int randomG = random(255); // 生成 0 到 255 之間的隨機綠色值
      int randomB = random(255); // 生成 0 到 255 之間的隨機藍色值
      // 更新燈條顏色
      update_GRB_LED(randomR, randomG, randomB);
    }
  }
  else{
    update_GRB_LED(0, 0, 0);
  }

  delay(500);  // 等待 500 msec
}