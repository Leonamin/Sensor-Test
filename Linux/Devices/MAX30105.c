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

#define MAX30105_INTR_STATUS_1      0x00
#define MAX30105_INTR_STATUS_2      0x01
#define MAX30105_INTR_ENABLE_1      0x02
#define MAX30105_INTR_ENABLE_2      0x03
#define MAX30105_FIFO_WR_PTR        0x04
#define MAX30105_OVF_COUNTER        0x05
#define MAX30105_FIFO_RD_PTR        0x06
#define MAX30105_FIFO_DATA          0x07
#define MAX30105_FIFO_CONFIG        0x08
#define MAX30105_MODE_CONFIG        0x09
#define MAX30105_SPO2_CONFIG        0x0A
#define MAX30105_LED1_PA            0x0C
#define MAX30105_LED2_PA            0x0D
#define MAX30105_LED3_PA            0x0E
#define MAX30105_LED4_PA            0x0F
#define MAX30105_PILOT_PA           0x10
#define MAX30105_MULTI_LED_CTRL1    0x11
#define MAX30105_MULTI_LED_CTRL2    0x12
#define MAX30105_TEMP_INTR          0x1F
#define MAX30105_TEMP_FRAC          0x20
#define MAX30105_TEMP_CONFIG        0x21
#define MAX30105_PROX_INT_THRESH    0x30
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

int INIT() {
    uint8_t id;
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

    return 1;
}

int main(int argc, char *argv[]) 
{
    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, MAX30105_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    INIT();
}