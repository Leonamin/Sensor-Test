#include <Wire.h>

#define MAX30101_I2C_ADDRESS        0xA7

#define MAX30101_INTR_STATUS_1      0x00
#define MAX30101_INTR_STATUS_2      0x01
#define MAX30101_INTR_ENABLE_1      0x02
#define MAX30101_INTR_ENABLE_2      0x03
#define MAX30101_FIFO_WR_PTR        0x04
#define MAX30101_OVF_COUNTER        0x05
#define MAX30101_FIFO_RD_PTR        0x06
#define MAX30101_FIFO_DATA          0x07
#define MAX30101_FIFO_CONFIG        0x08
#define MAX30101_MODE_CONFIG        0x09
#define MAX30101_SPO2_CONFIG        0x0A
#define MAX30101_LED1_PA            0x0C
#define MAX30101_LED2_PA            0x0D
#define MAX30101_LED3_PA            0x0E
#define MAX30101_LED3_PA            0x0F
#define MAX30101_PILOT_PA           0x10
#define MAX30101_MULTI_LED_CTRL1    0x11
#define MAX30101_MULTI_LED_CTRL2    0x12
#define MAX30101_TEMP_INTR          0x1F
#define MAX30101_TEMP_FRAC          0x20
#define MAX30101_TEMP_CONFIG        0x21
#define MAX30101_PROX_INT_THRESH    0x30
#define MAX30101_REV_ID             0xFE
#define MAX30101_PART_ID            0xFF

int MAX30101_Write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MAX30101_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

int MAX30101_Read(uint8_t reg, uint8_t *buf, int size) {
    Wire.beginTransmission(MAX30101_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MAX30101_I2C_ADDRESS, size);
    
    if (Wire.available() != size)
        return 0;
    for(int i = 0; i < size; i++) {
        buf[i] = Wire.read();
    }

    return 1;
}

int MAX30101_Read_FIFO(uint32_t *red_led, uint32_t *ir_led) {
    uint8_t buf[6];
    *red_led = 0;
    *ir_led = 0;
    if ( !MAX30101_Read(MAX30101_I2C_ADDRESS, buf, 6))
        return 0;
    *red_led += buf[0] << 16;
    *red_led += buf[1] << 8;
    *red_led += buf[2];

    *ir_led += buf[3] << 16;
    *ir_led += buf[4] << 8;
    *ir_led += buf[5];

    return 1;
}

void MAX30101_Reset() {
    MAX30101_Write(MAX30101_MODE_CONFIG, 0x40);
}

void MAX30101_Init() {
    MAX30101_Write(MAX30101_INTR_ENABLE_1, 0xC0);
    MAX30101_Write(MAX30101_INTR_ENABLE_2, 0x00);
    MAX30101_Write(MAX30101_FIFO_WR_PTR, 0x00);
    MAX30101_Write(MAX30101_OVF_COUNTER, 0x00);
    MAX30101_Write(MAX30101_FIFO_RD_PTR, 0x00);
    MAX30101_Write(MAX30101_FIFO_CONFIG, 0x0F);
    MAX30101_Write(MAX30101_MODE_CONFIG, 0x03);         //010 심작방동 모드 011 산소포화도 모드 111 멀티LED 모드
    MAX30101_Write(MAX30101_SPO2_CONFIG, 0x27);
    MAX30101_Write(MAX30101_LED1_PA, 0x24);
    MAX30101_Write(MAX30101_LED2_PA, 0x24);
    MAX30101_Write(MAX30101_LED3_PA, 0x7F);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Serial Connected");
    Wire.begin();
    Serial.println("I2C Connected");

    MAX30101_Init();
}

void loop() {
    uint32_t red_led, ir_led;
    MAX30101_Read_FIFO(&red_led, &ir_led);

    Serial.print("RED LED: ");
    Serial.println(red_led);
    Serial.print("IR LED: ");
    Serial.println(ir_led);

    delay(100);
}