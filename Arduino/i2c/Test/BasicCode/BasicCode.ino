#include <Wire.h>

int readData(int reg, uint8_t *buffer, int size) {
    int i, n, error;
    //전송할 Slave 주소값 지정
    Wire.beginTransmission(I2C_ADDRESS);
    //읽을 레지스터 reg
    n = Wire.write(reg);
    if (n != 1)
        return -10;

    n = Wire.endTransmission();        //I2C 버스 지속
    if (n != 0)
        return n;
    //size만큼의 데이터 요청 후 stop 비트 true
    Wire.requestFrom(I2C_ADDRESS, size, true);
    i = 0;
    //사이즈 만큼 비트 읽기
    while(Wire.available() && i < size) {
        buffer[i++] = Wire.read();
    }
    if (i != size)
        return -11;
    return 0;
}

int writeData(int reg, const uint8_t *pData, int size) {
    int n, error;

    Wire.beginTransmission(I2C_ADDRESS);
    n = Wire.write(reg);
    if (n != 1)
        return -20;
    n = Wire.write(pData, size);
    if (n != size)
        return -21;
    error = Wire.endTransmission(true);     //I2C 버스 종료
    if (error != 0)
        return error;
    
    return 0;
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();
}

void loop()
{

}