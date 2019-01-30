#include <Wire.h>

//APDS-9960 I2C Address
#define APDS9960_I2C_ADDRESS 0x39
 
//제스쳐 인자
#define GESTURE_THRESHOLD_OUT   10
#define GESTURE_SENSITIVITY_1   50
#define GESTURE_SENSITIVITY_2   20

//장치 ID
#define APDS9960_ID_1           0xAB
#define APDS9960_ID_2           0x9C 

//에러코드
#define ERROR                   0xFF

//APDS-9960 레지스터 주소
#define APDS9960_ENABLE         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL        0x8F
#define APDS9960_CONFIG2        0x90
#define APDS9960_ID             0x92
#define APDS9960_STATUS         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PDATA          0x9C
#define APDS9960_POFFSET_UR     0x9D
#define APDS9960_POFFSET_DL     0x9E
#define APDS9960_CONFIG3        0x9F
#define APDS9960_GPENTH         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF

//setMode에 대한 인자
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7


//LED Drive 값
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

//근접 이득 값
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

//ALS 이득 값
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

//제스쳐 이득 값
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

//LED 부스트 값
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3    

//제스쳐 기다리는 시간
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

//기본값
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset      
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost  
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode    
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

int APDS9960_Write(uint8_t reg, uint8_t cmd) {
    Wire.beginTransmission(APDS9960_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(cmd);
    Wire.endTransmission();

    return 1;
}

int APDS9960_Read(uint8_t reg, uint8_t *buf, int size) {
    int i, n;
    Wire.beginTransmission(APDS9960_I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(APDS9960_I2C_ADDRESS, size);
    
    while(Wire.available() && i < size) {
        buf[i++] = Wire.read();
    }

    return 1;
}

int APDS9960_setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;
    
    if( !APDS9960_Read(APDS9960_ENABLE, val, 1) ) {
        return 0;
    }
    
    enable &= 0b00000001;
    enable = enable << 4;
    val &= 0b11101111;
    val |= enable;
    
    if( !APDS9960_Write(APDS9960_ENABLE, val) ) {
        return 0;
    }
    
    return 1;
}

int APDS9960_setAmbientLightGain(uint8_t drive) {
    uint8_t val;

    APDS9960_Read(APDS9960_CONTROL, val, 1);

    drive &= 0b00000011;
    val &= 0b11111100;
    val |= drive;

    APDS9960_Write(APDS9960_CONTROL, val);

    return 1;
}

uint8_t APDS9960_getMode()
{
    uint8_t enable_value;
    
    if( !APDS9960_Read(APDS9960_ENABLE, enable_value, 1) ) {
        return ERROR;
    }
    
    return enable_value;
}

int APDS9960_setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    reg_val = APDS9960_getMode();
    if( reg_val == ERROR ) {
        return 0;
    }
    
    enable = enable & 0x01;
    if( mode >= 0 && mode <= 6 ) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
        
    if( !APDS9960_Write(APDS9960_ENABLE, reg_val) ) {
        return 0;
    }
        
    return 1;
}

int APDS9960_enablePower()
{
    if( !APDS9960_setMode(POWER, 1) ) {
        return 0;
    }
    
    return 1;
}

int APDS9960_enableLightSensor(int interrupt) {
    if (!APDS9960_setAmbientLightGain(DEFAULT_AGAIN)) {
        return 0;
    }
    if (interrupt) {
        if(!APDS9960_setAmbientLightIntEnable(1)) {
            return 0;
        }
    } else {
        if(!APDS9960_setAmbientLightIntEnable(0)) {
            return 0;
        }
    }
    if(!APDS9960_enablePower()) {
        return 0;
    }
    if(!APDS9960_setMode(AMBIENT_LIGHT, 1)) {
        return 0;
    }

    return 1;
}

int APDS9960_Init() {

    //ID 확인
    uint8_t id;
    APDS9960_Read(APDS9960_ID, &id, 1);
    Serial.print("APDS-9960 ID: \t");
    Serial.println(id);
    if(id != 0xAB) {
        return 0;
    }
    //ENABLE 레지스터 전부 끄기
    if(!APDS9960_Write(APDS9960_ENABLE, 0x00))      //MSB는 Reserved 비트
        return 0;
    if(!APDS9960_Write(APDS9960_ATIME, DEFAULT_ATIME))
        return 0;
    if(!APDS9960_Write(APDS9960_CONFIG1, DEFAULT_CONFIG1))
        return 0;
    if(!APDS9960_Write(APDS9960_CONFIG2, DEFAULT_CONFIG2))
        return 0;
    if(!APDS9960_Write(APDS9960_CONFIG3, DEFAULT_CONFIG3))
        return 0;
    if(!APDS9960_Write(APDS9960_CONTROL, 0xCF))     //LED Drive 12.5, P, A Gain 8x, AGAIN_16X
        return 0;
    if(!APDS9960_Write(APDS9960_AILTL, 0xFF))
        return 0;
    if(!APDS9960_Write(APDS9960_AILTH, 0xFF))
        return 0;
    if(!APDS9960_Write(APDS9960_AIHTL, 0x00))
        return 0;
    if(!APDS9960_Write(APDS9960_AIHTH, 0x00))
        return 0;
    
    return 1;
}

int APDS9960_readAmbientLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    if( !APDS9960_Read(APDS9960_CDATAL, &val_byte, 1) ) {
        return 0;
    }

    val = val_byte;

    if( !APDS9960_Read(APDS9960_CDATAH, &val_byte, 1) ) {
        return 0;
    }
    val += ((uint16_t)val_byte << 8);
    
    return 1;
}

int APDS9960_readRedLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    if( !APDS9960_Read(APDS9960_RDATAL, &val_byte, 1) ) {
        return 0;
    }
    
    val = val_byte;

    if( !APDS9960_Read(APDS9960_RDATAH, &val_byte, 1) ) {
        return 0;
    }
    val += ((uint16_t)val_byte << 8);
    
    return 1;
}

int APDS9960_readGreenLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    if( !APDS9960_Read(APDS9960_GDATAL, &val_byte, 1) ) {
        return 0;
    }
    
    val = val_byte;

    if( !APDS9960_Read(APDS9960_GDATAH, &val_byte, 1) ) {
        return 0;
    }
    val += ((uint16_t)val_byte << 8);
    
    return 1;
}

int APDS9960_readBlueLight(uint16_t &val)
{
    uint8_t val_byte;
    val = 0;

    if( !APDS9960_Read(APDS9960_BDATAL, &val_byte, 1) ) {
        return 0;
    }
    
    val = val_byte;

    if( !APDS9960_Read(APDS9960_BDATAH, &val_byte, 1) ) {
        return 0;
    }
    val += ((uint16_t)val_byte << 8);
    
    return 1;
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("Serial Connected");
    Wire.begin();
    Serial.println("I2C Connected");

    if(!APDS9960_Init()) {
        Serial.println("Failed to call Device");
        while(1) {}
    }
    //RGB Ambient Light 허용
    APDS9960_enableLightSensor(0);
}

void loop(void) {
    uint16_t ambientlight, redlight,greenlight, bluelight;
    if(!APDS9960_readAmbientLight(ambientlight) || !APDS9960_readRedLight(redlight) || 
    !APDS9960_readBlueLight(bluelight) || !APDS9960_readGreenLight(greenlight)) {
        Serial.println("ERROR");
    } else {
        Serial.print("Ambient:\t");
        Serial.println(ambientlight);
        Serial.print("Red:\t");
        Serial.println(redlight);
        Serial.print("Green:\t");
        Serial.println(greenlight);
        Serial.print("Blue:\t");
        Serial.println(bluelight);
    }
    delay(1000);
}