// #include <WS2812.h>

// #define NUM_LEDS 8
// #define COLOR_PER_LEDS 3
// #define NUM_BYTES (NUM_LEDS * COLOR_PER_LEDS)

// #if NUM_BYTES > 255
// #error "NUM_BYTES can not be larger than 255."
// #endif

// #define lightStrip 31 // WS2812 資料腳位

// __xdata uint8_t ledData[NUM_BYTES]; // 燈條資料緩衝區

// unsigned int avgRed = 0;
// unsigned int avgGreen = 0;
// unsigned int avgBlue = 0;

// // UART 初始化
// void UART_Init(void) {
//     SM0 = 0;  // UART 模式 1（8 位數據）
//     SM1 = 1;
//     TMOD |= 0x20; // 設置定時器 1 為 8 位自動重載模式
//     TH1 = 0xFD;   // 設定波特率為 9600 (12 MHz 晶振)
//     TR1 = 1;      // 啟動定時器 1
//     REN = 1;      // 啟用接收功能
//     TI = 1;       // 設置發送準備完成標誌
// }

// // UART 接收一個字節
// char UART_ReceiveChar(void) {
//     while (!RI);  // 等待接收完成
//     RI = 0;       // 清除接收標誌
//     return SBUF;  // 返回接收到的字節
// }

// // UART 接收 RGB 資料
// void UART_ReceiveRGB(void) {
//     avgRed = UART_ReceiveChar();    // 接收 Red 值
//     avgGreen = UART_ReceiveChar();  // 接收 Green 值
//     avgBlue = UART_ReceiveChar();   // 接收 Blue 值
// }

// // 初始化腳位
// void pinInit() {
//     pinMode(lightStrip, OUTPUT); // 設置燈條腳位為輸出
// }

// // 主程式初始化
// void setup() {
//     pinInit();      // 初始化腳位
//     UART_Init();    // 初始化 UART
// }

// // 主程式循環
// void loop() {
//     UART_ReceiveRGB(); // 接收 UART 傳輸的 RGB 資料

//     // 更新燈條顏色
//     for (uint8_t i = 0; i < NUM_LEDS; i++) {
//         set_pixel_for_GRB_LED(ledData, i, avgRed, avgGreen, avgBlue);
//     }
//     neopixel_show_P3_1(ledData, NUM_BYTES); // 更新 WS2812 燈條

//     delay(100); // 短暫延遲，防止過快更新
// }

#include <Arduino.h>

#define SDA_PIN 32 // 定義 SDA 為 P3.2
#define SCL_PIN 14  // 定義 SCL 為 P1.4

#define I2C_DELAY_US 5  // I²C 時鐘延遲 (調整延遲來控制速率)

// 語音設備 I²C 位址
#define VOICE_I2C_ADDR 0x0B

// 學習指令及恢復出場指令
const unit_t STUDY_COMMAND = 0x50;
const unit_t INIT_COMMAND = 0x60;

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

void setup() {

  i2c_init();
  //延遲1秒
  delay(1000);
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

  // 若喚醒設備，進入學習模式
  if (voice_data[0] == 0x01) {
      sleep = false;
      // 進入學習模式
      delay(2000);
      i2c_write(VOICE_I2C_ADDR, &data, 1);

      USBSerial_print("Entering study mode");
  } 
  else {
      USBSerial_print("Unexpected response: ");
      USBSerial_print(voice_data[0], HEX);
  }

  delay(500);  // 等待 500 msec
}