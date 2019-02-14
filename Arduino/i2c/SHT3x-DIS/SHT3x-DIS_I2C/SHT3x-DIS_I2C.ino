#include <Wire.h>

//SHT3x의 모든 명령과 데이터는 16비트
#define SHT31_DEFAULT_ADDR         0x44
#define SHT31_MEAS_HIGHREP_STRETCH 0x2C06
#define SHT31_MEAS_MEDREP_STRETCH  0x2C0D
#define SHT31_MEAS_LOWREP_STRETCH  0x2C10
#define SHT31_MEAS_HIGHREP         0x2400
#define SHT31_MEAS_MEDREP          0x240B
#define SHT31_MEAS_LOWREP          0x2416
#define SHT31_READSTATUS           0xF32D
#define SHT31_CLEARSTATUS          0x3041
#define SHT31_SOFTRESET            0x30A2
#define SHT31_HEATEREN             0x306D
#define SHT31_HEATERDIS            0x3066

double hum, temp;

void WriteCommand(uint16_t cmd) {
    int n;
    Wire.beginTransmission(SHT31_DEFAULT_ADDR);
    n = Wire.write(cmd >> 8);           //앞쪽 8비트 예 0x2C06을 8bit 시프트 하면 0x2C
    n = Wire.write(cmd & 0xFF);         //뒤쪽 8비트 예 0x2C06을 0xFF로 논리연산 하면 0x06
    Wire.endTransmission();
}

int ReadTempHum() {
    uint8_t buf[6];

    WriteCommand(SHT31_MEAS_HIGHREP);

    delay(20);          //대기
    Wire.requestFrom((uint8_t)SHT31_DEFAULT_ADDR, (uint8_t)6);
    if (Wire.available() != 6)
        return 0;
    for (int i = 0; i < 6; i++) {
        buf[i] = Wire.read();
    }
    uint16_t ST, SRH;
    //raw 데이터 받기 형식은 8비트씩 |Temp MSB|TEMPLSB|CRC|HUM MSB|HUM LSB|CRC|
    ST = buf[0];        //앞쪽 8비트 온도
    ST <<= 8;
    ST |= buf[1];

    SRH = buf[3];
    SRH <<= 8;
    SRH |= buf[4];


    //온도 및 습도 구하기
    double stemp = ST;
    stemp *= 175;
    stemp /= 0xffff;
    stemp = -45 + stemp;
    temp = stemp;

    double shum = SRH;
    shum *= 100;
    shum /= 0xFFFF;
    hum = shum;

    return 1;
}

uint16_t ReadStatus() {
    WriteCommand(SHT31_READSTATUS);
    Wire.requestFrom((uint8_t)SHT31_DEFAULT_ADDR, (uint8_t)3);
    uint16_t status = Wire.read();
    status <<= 8;
    status |= Wire.read();
    return status;
}

void Heater(int h) {
    if (h)
        WriteCommand(SHT31_HEATEREN);
    else
        WriteCommand(SHT31_HEATERDIS);
}

void Reset() {
    WriteCommand(SHT31_SOFTRESET);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Serial Connected!");

    Wire.begin();
    Serial.println("I2C Connected!");
    Reset();
}

void loop() {
    if(!ReadTempHum()) {
        Serial.println("Failed to call ReadTempHum");
    }
    
    Serial.print("Temperature: ");
    Serial.println(temp);
    Serial.print("Humidity: ");
    Serial.println(hum);
    
    delay(100);
}