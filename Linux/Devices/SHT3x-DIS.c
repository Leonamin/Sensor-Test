/*
컴파일 예시: arm-linux-gnueabihf-gcc -o SHT3x-DIS SHT3x-DIS.c 
*/

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#define SHT_ADDRESS                     0x44
#define SHT_MEAS_HGIHREP_STRETCH        0x2C06
#define SHT_MEAS_MEDREP_STRETCH         0x2C0D
#define SHT_MEAS_LOWREP_STRETCH         0x2C10
#define SHT_MEAS_HIGHREP                0x2400
#define SHT_MEAS_MEDREP                 0x240B
#define SHT_MEAS_LOWREP                 0x2416
#define SHT_READSTATUS                  0xF32D
#define SHT_CLEARSTATUS                 0x3041
#define SHT_SOFTRESET                   0x30A2
#define SHT_HEATEREN                    0x306D
#define SHT_HEATERDIS                   0x3066

int fd;

int WriteData(uint8_t reg_addr, uint8_t *data, int size) {
    uint8_t *buf;
    buf = malloc(size + 1);
    buf[0] = reg_addr;
    memcpy(buf + 1, data, size);
    write(fd, buf, size + 1);
    free(buf);

    return 1;
}

int ReadData(uint8_t reg_addr, uint8_t *data, int size) {
    write(fd, &reg_addr, 1);
    read(fd, data, size);

    return 1;
}

void ReadTempHum() {
    uint8_t reg, cmd;
    uint8_t buf[6];
    double temp, hum;

    while(1) {
        reg = SHT_MEAS_HIGHREP >> 8 & 0xFF;
        WriteData(reg, &cmd, 1);
        //sleep(1);
        sleep(1);
        read(fd, buf, 6);
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
        printf("Temperature: %.3lf, Humidity: %.3lf\r\n", temp, hum);
    }
    
}

uint16_t ReadStatus() {
    uint8_t reg, cmd;
    uint8_t data[3];
    uint16_t status;
    reg = SHT_READSTATUS >> 8 & 0xFF;
    cmd = SHT_READSTATUS & 0xFF;
    WriteData(reg, &cmd, 1);
    read(fd, data, 3);
    status = data[0] << 8;
    status |= data[1];

    return status;
}

void Heater(int h) {
    uint8_t reg, cmd;
    if (h){
        reg = SHT_HEATEREN >> 8 & 0xFF;
        cmd = SHT_HEATEREN & 0xFF;
    } else {
        reg = SHT_HEATERDIS >> 8 & 0xFF;
        cmd = SHT_HEATERDIS & 0xFF;
    }
    WriteData(reg, &cmd, 1);
}

void Reset() {
    uint8_t reg, cmd;
    reg = SHT_SOFTRESET >> 8 & 0xFF;
    cmd = SHT_SOFTRESET & 0xFF;
    WriteData(reg, &cmd, 1);
}

int main(int argc, char *argv[])
{
    if((fd = open(argv[1], O_RDWR)) < 0) {
        perror("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, 0x44) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    printf("Good\n");
    ReadTempHum();

    return 0;
}