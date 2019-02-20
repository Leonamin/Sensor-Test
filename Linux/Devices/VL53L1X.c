#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#define VL53L1X_I2C_ADDRESS 0x29

/* 정의부분 */

int fd;

int WriteData(uint16_t reg_addr, uint8_t *data, int size) {
    uint8_t *buf;
    buf = malloc(size + 2);
    buf[0] = reg_addr >> 8;
    buf[1] = reg_addr & 0xFF;
    memcpy(buf + 2, data, size);
    write(fd, buf, size + 2);
    free(buf);

    return 1;
}

int ReadData(uint16_t reg_addr, uint8_t *data, int size) {
    uint8_t addr[2];
    addr[0] = (reg_addr >> 8);
    addr[1] = (reg_addr & 0xFF);
    write(fd, addr, 2);
    read(fd, data, size);

    return 1;
}

void INIT() {
    uint16_t index = 0x010F;
    uint8_t data[3];
    ReadData16(index, data, 3);
    printf("%x %x %x\n", data[0], data[1], data[2]);
}

int main(int argc, char *argv[]) 
{
    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, VL53L1X_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    INIT();
}