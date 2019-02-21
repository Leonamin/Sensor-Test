#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#define VL53L1X_I2C_ADDRESS                             0x29
#define SOFT_RESET                                      0X0000
#define OSC_MEASURED__FAST_OSC__FREQUENCY               0x0006
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM              0x001E
#define MM_CONFIG__OUTER_OFFSET_MM                      0x0022
#define DSS_CONFIG__TARGET_TOTAL_RATE_MCPS              0x0024
#define PAD_I2C_HV__EXTSUP_CONFIG                       0x002E
#define GPIO__TIO_HV_STATUS                             0x0031
#define SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS       0x0036
#define SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS     0x0037
#define ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM    0x0039
#define ALGO__RANGE_IGNORE_VALID_HEIGHT_MM              0x003E
#define ALGO__RANGE_MIN_CLIP                            0x003F
#define ALGO__CONSISTENCY_CHECK__TOLERANCE              0x0040
#define PHASECAL_CONFIG__TIMEOUT_MACROP                 0x004B
#define DSS_CONFIG__ROI_MODE_CONTROL                    0x004F
#define SYSTEM__THRESH_RATE_HIGH                        0x0050
#define SYSTEM__THRESH_RATE_LOW                         0x0052
#define DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT       0x0054
#define DSS_CONFIG__APERTURE_ATTENUATION                0x0057
#define MM_CONFIG__TIMEOUT_MACROP_A                     0x005A
#define MM_CONFIG__TIMEOUT_MACROP_B                     0x005C
#define RANGE_CONFIG__TIMEOUT_MACROP_A                  0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A                    0x0060
#define RANGE_CONFIG__TIMEOUT_MACROP_B                  0x0061
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
#define RESULT__OSC_CALIBRATE_VAL                       0x00DE

#define IDENTIFICATION__MODEL_ID                        0x010F

/* 정의부분 */

enum DistanceMode {
    Long,
    Medium,
    Short
};

int fd;
uint16_t fast_osc_frequency, osc_calibrate_val;
uint16_t timeout_start_ms;

void writeReg(uint16_t reg, uint8_t value)
{
    uint8_t buf[3];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = value;
    write(fd, buf, 3);
}

void writeReg16Bit(uint16_t reg, uint16_t value)
{
    uint8_t buf[4];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
    write(fd, buf, 4);
}

void writeReg32Bit(uint16_t reg, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    buf[2] = (value >> 24) & 0xFF;
    buf[3] = (value >> 16) & 0xFF;
    buf[4] = (value >> 8) & 0xFF;
    buf[5] = value & 0xFF;
    write(fd, buf, 6);
}

uint8_t readReg(uint16_t reg)
{
    uint8_t value;
    uint8_t buf[2];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    write(fd ,buf, 2);
    read(fd, &value, 1);

    return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(uint16_t reg)
{
    uint16_t value;
    uint8_t buf[2];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    write(fd ,buf, 2);
    read(fd, buf, 2);
    value = buf[0] << 8 | buf[1];

    return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(uint16_t reg)
{
    uint32_t value;
    uint8_t buf[4];
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    write(fd ,buf, 2);
    read(fd, buf, 4);
    value = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];

    return value;
}

unsigned long millis() {
    struct timeval te; 
    gettimeofday(&te, NULL); // 현재 시간 얻어오기
    unsigned long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // 밀리세컨드 계산
    return milliseconds;
}

uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

void startTimeout() { timeout_start_ms = millis(); }

uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint16_t encodeTimeout(uint32_t timeout_mclks)
{
    // encoded format: "(LSByte * 2^MSByte) + 1"

    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
        ls_byte >>= 1;
        ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else { return 0; }
}

uint32_t getMeasurementTimingBudget()
{
    uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
    readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

    return  2 * range_config_timeout_us + 4528;
}

int setMeasurementTimingBudget(uint32_t budget_us)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    if (budget_us <= 4528) { return 1; }

    uint32_t range_config_timeout_us = budget_us -= 4528;
    if (range_config_timeout_us > 1100000) { return 1; } // FDA_MAX_TIMING_BUDGET_US * 2

    range_config_timeout_us /= 2;

    uint32_t macro_period_us;

    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
    writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
        timeoutMicrosecondsToMclks(1, macro_period_us)));

    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
        timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));

    writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
        timeoutMicrosecondsToMclks(1, macro_period_us)));

    writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
        timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    return 1;
}

int setDistanceMode(enum DistanceMode mode)
{
    //시간분배 저장
    uint32_t budget_us = getMeasurementTimingBudget();

    switch (mode)
    {
        case Short:
            // 타이밍 설정
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
            writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

            // 동적 설정
            writeReg(SD_CONFIG__WOI_SD0, 0x07);
            writeReg(SD_CONFIG__WOI_SD1, 0x05);
            writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); 
            writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); 

            break;

        case Medium:
            // 타이밍 설정
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
            writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

            // 동적 설정
            writeReg(SD_CONFIG__WOI_SD0, 0x0B);
            writeReg(SD_CONFIG__WOI_SD1, 0x09);
            writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); 
            writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); 

            break;

        case Long: // long
            // 타이밍 설정
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
            writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
            writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

            // 동적 설정
            writeReg(SD_CONFIG__WOI_SD0, 0x0F);
            writeReg(SD_CONFIG__WOI_SD1, 0x0D);
            writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); 
            writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); 

            break;

        default:
            // 모드 아니면 아무것도 안함
            return 0;
  }

    // 시간 분배 재요청
    setMeasurementTimingBudget(budget_us);

    return 1;
}

int Init(int io_2v8)
{
    if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEACC) { return 0; }

    // 소프트웨어 리셋
    writeReg(SOFT_RESET, 0x00);
    usleep(100);
    writeReg(SOFT_RESET, 0x01);
    //부팅 시간
    usleep(1000);

    startTimeout();
    
    if (io_2v8)
    {
        writeReg(PAD_I2C_HV__EXTSUP_CONFIG,
        readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
    }

    // 추후 사용을 위한 오실레이터 정보 저장
    fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
    osc_calibrate_val = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

    writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00); 
    writeReg(GPIO__TIO_HV_STATUS, 0x02);
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); 
    writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); 
    writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
    writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    writeReg(ALGO__RANGE_MIN_CLIP, 0); 
    writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); 

    // 범용 설정
    writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
    writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
    writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

    // 타이밍 설정
    writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360); 
    writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); 

    // 동적 설정

    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
    writeReg(SD_CONFIG__QUANTIFIER, 2); 

    writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
    writeReg(SYSTEM__SEED_CONFIG, 1); 

    // from VL53L1_config_low_power_auto_mode
    writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
    writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

    setDistanceMode(Long);
    setMeasurementTimingBudget(50000);

    writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
      readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);

    return 1;
}

int main(int argc, char *argv[]) 
{
    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
}