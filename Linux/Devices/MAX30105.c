#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#define MAX30105_I2C_ADDRESS        0x57

//상태 레지스터
#define MAX30105_INTR_STATUS_1      0x00
#define MAX30105_INTR_STATUS_2      0x01
#define MAX30105_INTR_ENABLE_1      0x02
#define MAX30105_INTR_ENABLE_2      0x03

//FIFO 레지스터
#define MAX30105_FIFO_WR_PTR        0x04
#define MAX30105_OVF_COUNTER        0x05
#define MAX30105_FIFO_RD_PTR        0x06
#define MAX30105_FIFO_DATA          0x07

//설정 레지스터
#define MAX30105_FIFO_CONFIG        0x08
#define MAX30105_MODE_CONFIG        0x09
#define MAX30105_SPO2_CONFIG        0x0A
#define MAX30105_LED1_PA            0x0C
#define MAX30105_LED2_PA            0x0D
#define MAX30105_LED3_PA            0x0E
#define MAX30105_LED_PROX_AMP       0x10
#define MAX30105_MULTI_LED_CTRL1    0x11
#define MAX30105_MULTI_LED_CTRL2    0x12

//상온 레지스터
#define MAX30105_TEMP_INTR          0x1F
#define MAX30105_TEMP_FRAC          0x20
#define MAX30105_TEMP_CONFIG        0x21

//근접 감지 함수 레지스터
#define MAX30105_PROX_INT_THRESH    0x30

//ID 레지스터
#define MAX30105_REV_ID             0xFE
#define MAX30105_PART_ID            0xFF

#define DEBUG

/* 정의부분 */

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

void RESET() {
    uint8_t data = 0x40;
    WriteData(MAX30105_MODE_CONFIG, &data, 1);
}

int INIT() {
    uint8_t id, data;
    if(!ReadData(MAX30105_PART_ID, &id, 1)) {
        return 0;
    }
#ifdef DEBUG
    printf("MAX30105 ID: %x\n", id);
#endif
    if(id != 0x15) {
        printf("Failed to check device id\n");
        return 0;
    }
    data = 0xC0;
    WriteData(MAX30105_INTR_ENABLE_1, &data, 1);    //A_FULL, New FIFO만 허용
    data = 0x00;
    WriteData(MAX30105_INTR_ENABLE_2, &data, 1);
    data = 0x00;
    WriteData(MAX30105_FIFO_WR_PTR, &data, 1);
    data = 0x00;
    WriteData(MAX30105_OVF_COUNTER, &data, 1);
    data = 0x00;
    WriteData(MAX30105_FIFO_RD_PTR, &data, 1);
    data = 0x0F;
    WriteData(MAX30105_FIFO_CONFIG, &data, 1);      //FIFO 데이터 크기 17
    data = 0x07;
    WriteData(MAX30105_MODE_CONFIG, &data, 1);         //010 심작방동 모드 011 산소포화도 모드 111 멀티LED 모드
    data = 0x27;
    WriteData(MAX30105_SPO2_CONFIG, &data, 1);      //ADC 범위 4096, Sample per sec 100, ADC 해상도 18
    data = 0x1F;
    WriteData(MAX30105_LED1_PA, &data, 1);
    data = 0x1F;
    WriteData(MAX30105_LED2_PA, &data, 1);
    data = 0x1F;
    WriteData(MAX30105_LED3_PA, &data, 1);
    data = 0x1F;
    WriteData(MAX30105_LED_PROX_AMP, &data, 1);
    data = 0x21;
    WriteData(MAX30105_MULTI_LED_CTRL1, &data, 1);
    data = 0x03;
    WriteData(MAX30105_MULTI_LED_CTRL2, &data, 1);

    return 1;
}

int MAX30101_Read_FIFO(uint32_t *red_led, uint32_t *ir_led, uint32_t *green_led) {
    uint8_t buf[9];
    *red_led = 0;
    *ir_led = 0;
    *green_led = 0;
    if ( !ReadData(MAX30105_FIFO_DATA, buf, 9))
        return 0;
    *red_led += (uint32_t)buf[0] << 16;
    *red_led += (uint32_t)buf[1] << 8;
    *red_led += (uint32_t)buf[2];

    *ir_led += (uint32_t)buf[3] << 16;
    *ir_led += (uint32_t)buf[4] << 8;
    *ir_led += (uint32_t)buf[5];

    *green_led += (uint32_t)buf[6] << 16;
    *green_led += (uint32_t)buf[7] << 8;
    *green_led += (uint32_t)buf[8];

    *red_led &= 0x03FFFF;
    *ir_led &= 0x03FFFF;
    *green_led &= 0x03FFFF;

    return 1;
}

int main(int argc, char *argv[]) 
{
    uint32_t red_led, ir_led, green_led;
    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, MAX30105_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    RESET();
    INIT();
    while(1) {
        MAX30101_Read_FIFO(&red_led, &ir_led, &green_led);
        printf("Red: %d, IR: %d, Green: %d\n", red_led, ir_led, green_led);
    }

    return 0;
}