#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#define ICM20948_WHO_AM_I               0x00    //R
#define ICM20948_USER_CtRL              0x03    //R/W
#define ICM20948_LP_CONFIG              0x05    //R/W
#define ICM20948_PWR_MGMT_1             0x06    //R/W
#define ICM20948_PWR_MGMT_2             0x07    //R/W
#define ICM20948_INT_PIN_CFG            0x0F    //R/W
#define ICM20948_INT_ENABLE             0x10    //R/W
#define ICM20948_INT_ENABLE_1           0x11
#define ICM20948_INT_ENABLE_2           0x12
#define ICM20948_INT_ENABLE_3           0x13
#define ICM20948_I2C_MST_STATUS         0x17
#define ICM20948_INT_STATUS             0x19
#define ICM20948_INT_STATUS_1           0x1A
#define ICM20948_INT_STATUS_2           0x1B
#define ICM20948_INT_STATUS_3           0x1C
#define ICM20948_DELAY_TIMEH            0x28
#define ICM20948_DELAY_TIMEL            0x29
#define ICM20948_ACCEL_XOUT_H           0x2D
#define ICM20948_ACCEL_XOUT_L           0x2E
#define ICM20948_ACCEL_YOUT_H           0x2F
#define ICM20948_ACCEL_YOUT_L           0x30
#define ICM20948_ACCEL_ZOUT_H           0x31
#define ICM20948_ACCEL_ZOUT_L           0x32
#define ICM20948_GYRO_XOUT_H            0x33
#define ICM20948_GYRO_XOUT_L            0x34
#define ICM20948_GYRO_YOUT_H            0x35
#define ICM20948_GYRO_YOUT_L            0x36
#define ICM20948_GYRO_ZOUT_H            0x37
#define ICM20948_GYRO_ZOUT_L            0x38
#define ICM20948_TEMP_OUT_H             0x39
#define ICM20948_TEMP_OUT_L             0x3A
#define ICM20948_EXT_SLV_SENS_DATA_00   0x3B
#define ICM20948_EXT_SLV_SENS_DATA_01   0x3C
#define ICM20948_EXT_SLV_SENS_DATA_02   0x3D
#define ICM20948_EXT_SLV_SENS_DATA_03   0x3E
#define ICM20948_EXT_SLV_SENS_DATA_04   0x3F
#define ICM20948_EXT_SLV_SENS_DATA_05   0x40
#define ICM20948_EXT_SLV_SENS_DATA_06   0x41
#define ICM20948_EXT_SLV_SENS_DATA_07   0x42
#define ICM20948_EXT_SLV_SENS_DATA_08   0x43
#define ICM20948_EXT_SLV_SENS_DATA_09   0x44
#define ICM20948_EXT_SLV_SENS_DATA_10   0x45
#define ICM20948_EXT_SLV_SENS_DATA_11   0x46
#define ICM20948_EXT_SLV_SENS_DATA_12   0x47
#define ICM20948_EXT_SLV_SENS_DATA_13   0x48
#define ICM20948_EXT_SLV_SENS_DATA_14   0x49
#define ICM20948_EXT_SLV_SENS_DATA_15   0x4A
#define ICM20948_EXT_SLV_SENS_DATA_16   0x4B
#define ICM20948_EXT_SLV_SENS_DATA_17   0x4C
#define ICM20948_EXT_SLV_SENS_DATA_18   0x4D
#define ICM20948_EXT_SLV_SENS_DATA_19   0x4E
#define ICM20948_EXT_SLV_SENS_DATA_20   0x4F
#define ICM20948_EXT_SLV_SENS_DATA_21   0x50
#define ICM20948_EXT_SLV_SENS_DATA_22   0x51
#define ICM20948_EXT_SLV_SENS_DATA_23   0x52
#define ICM20948_FIFO_EN_1              0x66
#define ICM20948_FIFO_EN_2              0x67
#define ICM20948_FIFO_RST               0x68
#define ICM20948_FIFO_MODE              0x69
#define ICM20948_FIFO_COUNTH            0x70
#define ICM20948_FIFO_COUNTL            0x71
#define ICM20948_FIFO_R_W               0x72
#define ICM20948_DATA_RDY_STATUS        0x74
#define ICM20948_RIFO_CFG               0x76                    
#define ICM20948_REG_BANK_SEL           0x7F

//AD0 핀이 low일 경우 I2C slave 주소는 0x68
//AD0 핀이 high일 경우 I2C slvae 주소는 0x69가 된다.  
#define ICM20948_I2C_ADDRESS            0x68

/* 정의부분 */

typedef struct _Accel_Gyro_Temp
{
    /* data */
    int x_accel;
    int y_accel;
    int z_accel;
    int x_gyro;
    int y_gyro;
    int z_gyro;
    int temp;
} Accel_Gyro_Temp;


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

void ReadAccelGyroTemp(Accel_Gyro_Temp *data) {
    uint8_t temp;
    
    data->x_accel = 0;
    data->y_accel = 0;
    data->z_accel = 0;
    data->x_gyro = 0;
    data->y_gyro = 0;
    data->z_gyro = 0;
    data->temp = 0;

    ReadData(ICM20948_ACCEL_XOUT_H, &temp, 1);
    data->x_accel += (int)temp << 8;
    ReadData(ICM20948_ACCEL_XOUT_L, &temp, 1);
    data->x_accel += (int)temp;
    ReadData(ICM20948_ACCEL_YOUT_H, &temp, 1);
    data->y_accel += (int)temp << 8;
    ReadData(ICM20948_ACCEL_YOUT_L, &temp, 1);
    data->y_accel += (int)temp;
    ReadData(ICM20948_ACCEL_ZOUT_H, &temp, 1);
    data->z_accel += (int)temp << 8;
    ReadData(ICM20948_ACCEL_ZOUT_L, &temp, 1);
    data->z_accel += (int)temp;

    ReadData(ICM20948_GYRO_XOUT_H, &temp, 1);
    data->x_gyro += (int)temp << 8;
    ReadData(ICM20948_GYRO_XOUT_L, &temp, 1);
    data->x_gyro += (int)temp;
    ReadData(ICM20948_GYRO_YOUT_H, &temp, 1);
    data->y_gyro += (int)temp << 8;
    ReadData(ICM20948_GYRO_YOUT_L, &temp, 1);
    data->y_gyro += (int)temp;
    ReadData(ICM20948_GYRO_ZOUT_H, &temp, 1);
    data->z_gyro += (int)temp << 8;
    ReadData(ICM20948_GYRO_ZOUT_L, &temp, 1);
    data->z_gyro += (int)temp;
    
    ReadData(ICM20948_TEMP_OUT_H, &temp, 1);
    data->temp += (int)temp << 8;
    ReadData(ICM20948_TEMP_OUT_L, &temp, 1);
    data->temp += (int)temp;
    
    


    /*
    for(int i = 0; i < 14; i++) {
        printf("%6d\t", buf[i]);
    }
    */
}

void INIT() {
    uint8_t id, data;
    ReadData(ICM20948_WHO_AM_I, &id, 1);
    if(id != 0xEA) {
        printf("id: %x\t Different with real ID(0xEA). . .\n", id);
        exit(1);
    }
    data = 0x00;
    WriteData(ICM20948_PWR_MGMT_2, &data, 1);       //자이로 가속도 모두 허용
    WriteData(ICM20948_PWR_MGMT_1, &data, 1);       //Sleep 모드 나오기
}

void sleepmode() {
    uint8_t data = 0x40;
    WriteData(ICM20948_PWR_MGMT_1, &data, 1);       //7번쨰 비트가 sleep 모드 비트
}

void calibrateSensor(Accel_Gyro_Temp *data) {
    int i;
    double x_accel = 0;
    double y_accel = 0;
    double z_accel = 0;
    double x_gyro = 0;
    double y_gyro = 0;
    double z_gyro = 0;

    for(i = 0; i < 10; i++) {
        ReadAccelGyroTemp(data);
        x_accel += data->x_accel;
        y_accel += data->y_accel;
        z_accel += data->z_accel;
        x_gyro += data->x_gyro;
        y_gyro += data->y_gyro;
        z_gyro += data->z_gyro;
        usleep(100000);
    }
    x_accel /= 10;
    y_accel /= 10;
    z_accel /= 10;
    x_gyro /= 10;
    y_gyro /= 10;
    z_gyro /= 10;

    data->x_accel = x_accel;
    data->y_accel = y_accel;
    data->z_accel = z_accel;
    data->x_gyro = x_gyro;
    data->y_gyro = y_gyro;
    data->z_gyro = z_gyro;
}

int main(int argc, char *argv[]) 
{
    Accel_Gyro_Temp data;
    Accel_Gyro_Temp base;

    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, ICM20948_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    INIT();
    calibrateSensor(&base);
    while(1) {
        ReadAccelGyroTemp(&data);
        printf("Accel X: %6d, Y: %6d, Z: %6d\tGyro X: %6d, Y: %6d, Z: %6d\tTemp: %d\n", data.x_accel, data.y_accel, data.z_accel, data.x_gyro, data.y_gyro, data.z_gyro, data.temp);
        /*
        double FS_SEL = 131;
        double gyroX = (data.x_gyro - base.x_gyro) / FS_SEL;
        double gyroY = (data.y_gyro - base.y_gyro) / FS_SEL;
        double gyroZ = (data.z_gyro - base.z_gyro) / FS_SEL;
        
        printf("Gyro X: %6.3lf,Y: %6.3lf,Z: %6.3lf\n", gyroX, gyroY, gyroZ);
        */
        usleep(100000);
    }

    return 0;
}