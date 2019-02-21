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
#define SYSTEM__INTERMEASUREMENT_PERIOD                 0x006C
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
#define SYSTEM__INTERRUPT_CLEAR                         0x0086
#define SYSTEM__MODE_START                              0x0087
#define RESULT__RANGE_STATUS                            0x0089
#define RESULT__OSC_CALIBRATE_VAL                       0x00DE

#define IDENTIFICATION__MODEL_ID                        0x010F

/* 정의부분 */

enum DistanceMode {
    Long,
    Medium,
    Short
};

enum RangeStatus
{
    RangeValid                =   0,

    // "sigma estimator check is above the internal defined threshold"
    // (sigma = standard deviation of measurement)
    SigmaFail                 =   1,

    // "signal value is below the internal defined threshold"
    SignalFail                =   2,

    // "Target is below minimum detection threshold."
    RangeValidMinRangeClipped =   3,

    // "phase is out of bounds"
    // (nothing detected in range; try a longer distance mode if applicable)
    OutOfBoundsFail           =   4,

    // "HW or VCSEL failure"
    HardwareFail              =   5,

    // "The Range is valid but the wraparound check has not been done."
    RangeValidNoWrapCheckFail =   6,

    // "Wrapped target, not matching phases"
    // "no matching phase in other VCSEL period timing."
    WrapTargetFail            =   7,

    // "Internal algo underflow or overflow in lite ranging."
// ProcessingFail            =   8: not used in API

    // "Specific to lite ranging."
    // should never occur with this lib (which uses low power auto ranging,
    // as the API does)
    XtalkSignalFail           =   9,

    // "1st interrupt when starting ranging in back to back mode. Ignore
    // data."
    // should never occur with this lib
    SynchronizationInt         =  10, // (the API spells this "syncronisation")

    // "All Range ok but object is result of multiple pulses merging together.
    // Used by RQL for merged pulse detection"
// RangeValid MergedPulse    =  11: not used in API

    // "Used by RQL as different to phase fail."
// TargetPresentLackOfSignal =  12:

    // "Target is below minimum detection threshold."
    MinRangeFail              =  13,

    // "The reported range is invalid"
// RangeInvalid              =  14: can't actually be returned by API (range can never become negative, even after correction)

    // "No Update."
    None                      = 255,
};

typedef struct ResultBuffer_
{
    uint8_t range_status; 
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
}ResultBuffer;

typedef struct RangingData_
{
    uint16_t range_mm;
    enum RangeStatus range_status;
    float peak_signal_count_rate_MCPS;
    float ambient_count_rate_MCPS;
}RangingData;

int fd;
uint16_t fast_osc_frequency, osc_calibrate_val;
uint16_t timeout_start_ms;
ResultBuffer results;
RangingData ranging_data;

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
    uint8_t buf[6];
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

float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }

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

void startContinuous(uint32_t period_ms)
{
    printf("1\n");
    // from VL53L1_set_inter_measurement_period_ms()
    writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
    printf("2\n");
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
    printf("3\n");
    writeReg(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

void readResults()
{
    uint8_t buf[2], data[17];
    buf[0] = (RESULT__RANGE_STATUS >> 8) & 0xFF;
    buf[1] = RESULT__RANGE_STATUS & 0xFF;
    write(fd, buf, 2);
    read(fd, data, 17);
    
    //2, 6, 7, 10, 11, 12, 13번째 데이터는 사용 안함
    results.range_status = buf[0];
    results.stream_count = buf[2];
    results.dss_actual_effective_spads_sd0 = (uint16_t)buf[4] << 8;
    results.dss_actual_effective_spads_sd0 += buf[5];

    results.ambient_count_rate_mcps_sd0 = (uint16_t)buf[8] << 8;
    results.ambient_count_rate_mcps_sd0 += buf[9];

    results.final_crosstalk_corrected_range_mm_sd0 = (uint16_t)buf[13] << 8;
    results.final_crosstalk_corrected_range_mm_sd0 += buf[14];

    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (uint16_t)buf[15] << 8;
    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = buf[16];
}

void updateDSS()
{
    uint16_t spadCount = results.dss_actual_effective_spads_sd0;

    if (spadCount != 0)
    {
        // "Calc total rate per spad"

        uint32_t totalRatePerSpad =
        (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
        results.ambient_count_rate_mcps_sd0;

        // "clip to 16 bits"
        if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

        // "shift up to take advantage of 32 bits"
        totalRatePerSpad <<= 16;

        totalRatePerSpad /= spadCount;

        if (totalRatePerSpad != 0)
        {
            // "get the target rate and shift up by 16"
            uint32_t requiredSpads = ((uint32_t)0x0A00 << 16) / totalRatePerSpad;

            // "clip to 16 bit"
            if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

            // "override DSS config"
            writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
            // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

            return;
        }
    }
    // If we reached this point, it means something above would have resulted in a
    // divide by zero.
    // "We want to gracefully set a spad target, not just exit with an error"

    // "set target to mid point"
    writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

void getRangingData()
{
    // VL53L1_copy_sys_and_core_results_to_range_results() begin

    uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).
    ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    // VL53L1_copy_sys_and_core_results_to_range_results() end

    // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
    // mostly based on ConvertStatusLite()
    switch(results.range_status)
    {
        case 17: // MULTCLIPFAIL
        case 2: // VCSELWATCHDOGTESTFAILURE
        case 1: // VCSELCONTINUITYTESTFAILURE
        case 3: // NOVHVVALUEFOUND
        // from SetSimpleData()
        ranging_data.range_status = HardwareFail;
        break;

        case 13: // USERROICLIP
        // from SetSimpleData()
        ranging_data.range_status = MinRangeFail;
        break;

        case 18: // GPHSTREAMCOUNT0READY
        ranging_data.range_status = SynchronizationInt;
        break;

        case 5: // RANGEPHASECHECK
        ranging_data.range_status =  OutOfBoundsFail;
        break;

        case 4: // MSRCNOTARGET
        ranging_data.range_status = SignalFail;
        break;

        case 6: // SIGMATHRESHOLDCHECK
        ranging_data.range_status = SignalFail;
        break;

        case 7: // PHASECONSISTENCY
        ranging_data.range_status = WrapTargetFail;
        break;

        case 12: // RANGEIGNORETHRESHOLD
        ranging_data.range_status = XtalkSignalFail;
        break;

        case 8: // MINCLIP
        ranging_data.range_status = RangeValidMinRangeClipped;
        break;

        case 9: // RANGECOMPLETE
        // from VL53L1_copy_sys_and_core_results_to_range_results()
        if (results.stream_count == 0)
        {
            ranging_data.range_status = RangeValidNoWrapCheckFail;
        }
        else
        {
            ranging_data.range_status = RangeValid;
        }
        break;

        default:
        ranging_data.range_status = None;
    }

    // from SetSimpleData()
    ranging_data.peak_signal_count_rate_MCPS =
        countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
    ranging_data.ambient_count_rate_MCPS =
        countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}


uint16_t readmm()
{
    readResults();

    updateDSS();

    getRangingData();

    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

    return ranging_data.range_mm;
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
    if(ioctl(fd, I2C_SLAVE, VL53L1X_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    if(Init(0) <= 0) {
        printf("Failed to detect and initialize sensor!\n");
        exit(1);
    }
    setDistanceMode(Long);
    
    setMeasurementTimingBudget(50000);

    startContinuous(50);    //50ms 마다 측정
    while(1) {
        printf("Range: %d\n", readmm());
        usleep(100000);
    }
    

    return 0;
}