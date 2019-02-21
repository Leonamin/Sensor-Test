#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#define VL53L1X_I2C_ADDRESS                             0x29
#define SOFT_RESET                                      0X0000
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM              0x001E
#define MM_CONFIG__OUTER_OFFSET_MM                      0x0022
#define DSS_CONFIG__TARGET_TOTAL_RATE_MCPS              0x0024
#define GPIO__TIO_HV_STATUS                             0x0031
#define SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS       0x0036
#define SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS     0x0037
#define ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM    0x0039
#define ALGO__RANGE_IGNORE_VALID_HEIGHT_MM              0x003E
#define ALGO__RANGE_MIN_CLIP                            0x003F
#define ALGO__CONSISTENCY_CHECK__TOLERANCE              0x0040
#define DSS_CONFIG__ROI_MODE_CONTROL                    0x004F
#define SYSTEM__THRESH_RATE_HIGH                        0x0050
#define SYSTEM__THRESH_RATE_LOW                         0x0052
#define DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT       0x0054
#define DSS_CONFIG__APERTURE_ATTENUATION                0x0057
#define RANGE_CONFIG__VCSEL_PERIOD_A                    0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B                    0x0063
#define RANGE_CONFIG__SIGMA_THRESH                      0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS     0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH                  0x0069
#define SYSTEM__GROUPED_PARAMETER_HOLD_0                0x0071
#define SYSTEM__SEED_CONFIG                             0x0077
#define SD_CONFIG__WOI_SD0                              0x0078
#define SD_CONFIG__WOI_SD1                              0x0079
#define SD_CONFIG__INITIAL_PHASE_SD0                    0x007A
#define SD_CONFIG__INITIAL_PHASE_SD1                    0x007B
#define SYSTEM__GROUPED_PARAMETER_HOLD_1                0x007C
#define SD_CONFIG__QUANTIFIER                           0x007E
#define SYSTEM__SEQUENCE_CONFIG                         0x0081
#define SYSTEM__GROUPED_PARAMETER_HOLD                  0x0082

#define VL53L1X_MODEL_ID                                0x010F


/* 정의부분 */

enum DistanceMode {
    Long,
    Medium,
    Short
};

int fd;
uint16_t osc_calibrate_val, fast_osc_frequency;

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

int setDistanceMode(enum DistanceMode mode)
{
    uint8_t data;

    switch (mode)
    {
        case Short:
            // from VL53L1_preset_mode_standard_ranging_short_range()

            // 타이밍 설정
            data = 0x07;   
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_A, &data,1);
            data = 0x05;
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_B, &data, 1);
            data = 0x38;
            WriteData(RANGE_CONFIG__VALID_PHASE_HIGH, &data, 1);

            // 동적 설정
            data = 0x07;
            WriteData(SD_CONFIG__WOI_SD0, &data, 1);
            data = 0x05;
            WriteData(SD_CONFIG__WOI_SD1, &data, 1);
            data = 6;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD0, &data, 1); // tuning parm default
            data = 6;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD1, &data, 1); // tuning parm default

            break;

        case Medium:
            // from VL53L1_preset_mode_standard_ranging()

            // 타이밍 설정
            data = 0x0B;   
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_A, &data,1);
            data = 0x09;
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_B, &data, 1);
            data = 0x78;
            WriteData(RANGE_CONFIG__VALID_PHASE_HIGH, &data, 1);

            // 동적 설정
            data = 0x0B;
            WriteData(SD_CONFIG__WOI_SD0, &data, 1);
            data = 0x09;
            WriteData(SD_CONFIG__WOI_SD1, &data, 1);
            data = 10;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD0, &data, 1); // tuning parm default
            data = 10;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD1, &data, 1); // tuning parm default

            break;

        case Long: // long
        // from VL53L1_preset_mode_standard_ranging_long_range()

            // 타이밍 설정
            data = 0x0F;   
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_A, &data,1);
            data = 0x0D;
            WriteData(RANGE_CONFIG__VCSEL_PERIOD_B, &data, 1);
            data = 0xB8;
            WriteData(RANGE_CONFIG__VALID_PHASE_HIGH, &data, 1);
    
            // 동적 설정
            data = 0x0F;
            WriteData(SD_CONFIG__WOI_SD0, &data, 1);
            data = 0x0D;
            WriteData(SD_CONFIG__WOI_SD1, &data, 1);
            data = 14;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD0, &data, 1); // tuning parm default
            data = 14;
            WriteData(SD_CONFIG__INITIAL_PHASE_SD1, &data, 1); // tuning parm default

            break;

        default:
            return 0;
    }

    return 1;
}

void INIT() {
    uint8_t data[2], id;
    ReadData(VL53L1X_MODEL_ID, &id, 1);
    if(id != 0xEA) {
        printf("Different with model id... (0x%x)\n", id);
        exit(1);
    }

    // Soft Reset
    data[0] = 0x00;
    WriteData(SOFT_RESET, data, 1);
    data[0] = 0x01;
    usleep(100000);
    WriteData(SOFT_RESET, data, 1);

    // 부팅시간
    usleep(1000);

    //고정 설정 기본
    data[0] = 0x0A;
    data[1] = 0x00;
    WriteData(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, data, 2);
    data[0] = 0x02;
    WriteData(GPIO__TIO_HV_STATUS, data, 1);
    data[0] = 8;
    WriteData(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, data, 1); 
    data[0] = 16;
    WriteData(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, data, 1); 
    data[0] = 0x01;
    WriteData(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, data, 1);
    data[0] = 0xFF;
    WriteData(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, data, 1);
    data[0] = 0;
    WriteData(ALGO__RANGE_MIN_CLIP, data, 1); 
    data[0] = 2;
    WriteData(ALGO__CONSISTENCY_CHECK__TOLERANCE, data, 1); 

    //범용 설정
    data[0] = 0x00;
    WriteData(SYSTEM__THRESH_RATE_HIGH, data, 2);
    WriteData(SYSTEM__THRESH_RATE_LOW, data, 2);
    data[0] = 0x38;
    WriteData(DSS_CONFIG__APERTURE_ATTENUATION, data, 1);

    // 타이밍 설정
    data[0] = 0x01;
    data[1] = 0x68;
    WriteData(RANGE_CONFIG__SIGMA_THRESH, data, 2); 
    data[0] = 0x00;
    data[1] = 0xC0;
    WriteData(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, data, 2);

    // 동적 설정
    data[0] = 0x01;
    WriteData(SYSTEM__GROUPED_PARAMETER_HOLD_0, data, 1);
    WriteData(SYSTEM__GROUPED_PARAMETER_HOLD_1, data, 1);
    data[0] = 2;
    WriteData(SD_CONFIG__QUANTIFIER, data, 1); 

    data[0] = 0x00;
    WriteData(SYSTEM__GROUPED_PARAMETER_HOLD, data, 1);
    data[0] = 1;
    WriteData(SYSTEM__SEED_CONFIG, data, 1); 
    data[0] = 0x8B;
    WriteData(SYSTEM__SEQUENCE_CONFIG, data, 1); // VHV, PHASECAL, DSS1, RANGE
    data[0] = 200;
    data[1] = 0;
    WriteData(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, data, 2);
    data[0] = 2;
    WriteData(DSS_CONFIG__ROI_MODE_CONTROL, data, 1); // REQUESTED_EFFFECTIVE_SPADS

    setDistanceMode(Long);

    ReadData(MM_CONFIG__OUTER_OFFSET_MM, data, 2); 
    data[0] = data[0] * 4;
    data[1] = data[1] * 4;
    WriteData(ALGO__PART_TO_PART_RANGE_OFFSET_MM, data, 2);
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