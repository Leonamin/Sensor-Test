#include <Wire.h>

//레지스터 이름은 데이터시트에서 참고
//데이터들
//습도는 16bit 기압, 온도는 20bit
#define HUM_LSB         0xFE        //RD Only
#define HUM_MSB         0xFD
#define TEMP_XLSB       0xFC
#define TEMP_LSB        0xFB
#define TEMP_MSB        0xFA
#define PRESS_XLSB      0xF9
#define PRESS_LSB       0xF8
#define PRESS_MSB       0xF7        //~RD Only
#define CONFIG          0xF5        //RDWR
#define CTRL_MEAS       0xF4        //RDWR
#define STATUS          0xF3        //do not
#define CTRL_HUM        0xF2        //do not
#define CALIB26         0xE1        //RD Only ~
#define CALIB27         0xE2
#define CALIB28         0xE3
#define CALIB29         0xE4
#define CALIB30         0xE5
#define CALIB31         0xE6
#define CALIB32         0xE7
#define CALIB33         0xE8
#define CALIB34         0xE9
#define CALIB35         0xEA
#define CALIB36         0xEB
#define CALIB37         0xEC
#define CALIB38         0xED
#define CALIB39         0xEE
#define CALIB40         0xEF
#define CALIB41         0xF0        //~RD Only
#define RESET           0xE0        //WR Only
#define ID              0xD0        //do not change~
#define CALIB00         0x88
#define CALIB01         0x89
#define CALIB02         0x8A
#define CALIB03         0x8B
#define CALIB04         0x8C
#define CALIB05         0x8D
#define CALIB06         0x8E
#define CALIB07         0x8F
#define CALIB08         0x90
#define CALIB09         0x91
#define CALIB10         0x92
#define CALIB11         0x93
#define CALIB12         0x94
#define CALIB13         0x95
#define CALIB14         0x96
#define CALIB15         0x97
#define CALIB16         0x98
#define CALIB17         0x99
#define CALIB18         0x9A
#define CALIB19         0x9B
#define CALIB20         0x9C
#define CALIB21         0x9D
#define CALIB22         0x9E
#define CALIB23         0x9F
#define CALIB24         0xA0
#define CALIB25         0xA1        //~RD Only

//CTRL_HUM과 CTRL_MEAS의 오버 샘플링
#define OVS1            001
#define OVS2            010
#define OVS4            011
#define OVS8            100
#define OVS16           101

//모드
#define SLEEP           00
#define FORCED          01
#define NORMAL          11

//config 레지스터 대기 시간
#define TSB05           000
#define TSB625          001
#define TSB125          010
#define TSB250          011
#define TSB500          100
#define TSB1000         101
#define TSB10           110
#define TSB20           111

//필터 세팅
#define FS0             000
#define FS2             001
#define FS4             010
#define FS8             011
#define FS16            100

//BME280 SLVAE bit SDO가 0이면 1110110(0x76) 1이면 1110111(0x77)
#define BME280_I2C_ADDRESSS       0x77

//===================================================================================
//BME280_read
//인자 목록
//reg: 시작 주소, 레지스터 정의에 사용할 주소
//buffer: 데이터를 담으려는 변수
//size: 받으려는 바이트
int BME280_read(int reg, uint8_t *buffer, int size)
{
    int i, n, error;
    //전송할 Slave 주소값 지정
    Wire.beginTransmission(BME280_I2C_ADDRESS);
    Serial.println("a1");
    //읽을 레지스터 reg
    n = Wire.write(reg);
    Serial.println("a2");
    if (n != 1)
        return -10;

    n = Wire.endTransmission(); //I2C 버스 지속
    Serial.println("a3");
    if (n != 0)
        return n;
    //size만큼의 데이터 요청 후 stop 비트 true
    Wire.requestFrom(BME280_I2C_ADDRESS, size, true);
    Serial.println("a4");
    i = 0;
    //사이즈 만큼 비트 읽기
    while (Wire.available() && i < size)
    {
        buffer[i++] = Wire.read();
    }
    if (i != size)
        return -11;
    return 0;
}
//===================================================================================
//BME280_write
//인자 목록
//reg: 시작 주소, 레지스터 정의에 사용할 주소
//pData: 데이터 쓰려는 포인터
//size: 쓰려는 바이트 숫자
int BME280_write(int reg, const uint8_t *pData, int size)
{
    int n, error;

    Wire.beginTransmission(BME280_I2C_ADDRESS);
    n = Wire.write(reg);
    if (n != 1)
        return -20;
    n = Wire.write(pData, size);
    if (n != size)
        return -21;
    error = Wire.endTransmission(true); //I2C 버스 종료
    if (error != 0)
        return error;

    return 0;
}

//====================================================================================
//BME280_write_reg
//레지스터 하나만
//인자목록
//reg: 단일 주소
//data: 데이터
int BME280_write_reg(int reg, uint8_t data)
{
    int error;

    error = BME280_write(reg, &data, 1);

    return error;
}

void setup()
{
    
}

void loop()
{
    
}