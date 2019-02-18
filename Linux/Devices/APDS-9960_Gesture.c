#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

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

#define APDS9960_GVALID         0b00000001
#define FIFO_PAUSE_TIME         30

int fd;
int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;

enum {
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */
enum {
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};


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

uint8_t APDS9960_getMode()
{
    uint8_t enable_value;
    
    if( !ReadData(APDS9960_ENABLE, &enable_value, 1) ) {
        return ERROR;
    }
    
    return enable_value;
}

int APDS9960_setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    reg_val = APDS9960_getMode();
    if( reg_val == ERROR ) {
        printf("ERROR\n");
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
        
    if( !WriteData(APDS9960_ENABLE, &reg_val, 1) ) {
        return 0;
    }
        
    return 1;
}

int APDS9960_setLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    if( !ReadData(APDS9960_CONTROL, &val, 1) ) {
        return 0;
    }
    
    drive &= 0b00000011;
    drive = drive << 6;
    val &= 0b00111111;
    val |= drive;
    
    if( !WriteData(APDS9960_CONTROL, &val, 1) ) {
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

int APDS9960_setGestureGain(uint8_t gain)
{
    uint8_t val;
    
    if(!ReadData(APDS9960_GCONF2, &val, 1))
        return 0;
    
    gain &= 0b00000011;
    gain = gain << 5;
    val &= 0b10011111;
    val |= gain;
    
    if(!WriteData(APDS9960_GCONF2, &val, 1))
        return 0;
    
    return 1;
}

int APDS9960_setGestureLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    if(!ReadData(APDS9960_GCONF2, &val, 1))
        return 0;
    
    drive &= 0b00000011;
    drive = drive << 3;
    val &= 0b11100111;
    val |= drive;
    
    if(!WriteData(APDS9960_GCONF2, &val, 1))
        return 0;
    
    return 1;
}

int APDS9960_Init() {

    //ID 확인
    uint8_t id, data;
    ReadData(APDS9960_ID, &id, 1);
    printf("APDS-9960 ID: %x\n",id);
    if(id != 0xAB) {
        return 0;
    }
    //ENABLE 레지스터 전부 끄기
    if(!APDS9960_setMode(ALL, 0))      //MSB는 Reserved 비트
        return 0;
    data = DEFAULT_ATIME;
    if(!WriteData(APDS9960_ATIME, &data, 1))
        return 0;
    data = DEFAULT_WTIME;
    if(!WriteData(APDS9960_WTIME, &data, 1)) 
        return 0;
    data = DEFAULT_PROX_PPULSE;
    if(!WriteData(APDS9960_PPULSE, &data, 1)) 
        return 0;
    data = DEFAULT_POFFSET_UR;
    if(!WriteData(APDS9960_POFFSET_UR, &data, 1)) 
        return 0;
    data = DEFAULT_POFFSET_DL;
    if(!WriteData(APDS9960_POFFSET_UR, &data, 1)) 
        return 0;
    data = DEFAULT_CONFIG1;
    if(!WriteData(APDS9960_CONFIG1, &data, 1))
        return 0;
    data = DEFAULT_CONFIG2;
    if(!WriteData(APDS9960_CONFIG2, &data, 1))
        return 0;
    data = DEFAULT_CONFIG3;
    if(!WriteData(APDS9960_CONFIG3, &data, 1))
        return 0;
    data = 0x00;
    if(!WriteData(APDS9960_CONTROL, &data, 1))     //LED Drive 12.5, P, A Gain 1x, AGAIN_1X
        return 0;
    data = 0xFF;
    if(!WriteData(APDS9960_AILTL, &data, 1))
        return 0;
    if(!WriteData(APDS9960_AILTH, &data, 1))
        return 0;
    data = 0x00;
    if(!WriteData(APDS9960_AIHTL, &data, 1))
        return 0;
    if(!WriteData(APDS9960_AIHTH, &data, 1))
        return 0;
    data = 0;
    if(!WriteData(APDS9960_PILT, &data, 1))
        return 0;
    data = 50;
    if(!WriteData(APDS9960_PIHT, &data, 1))
        return 0;
    
    //제스쳐 설정
    data = DEFAULT_GPENTH;
    if(!WriteData(APDS9960_GPENTH, &data, 1))
        return 0;
    data = DEFAULT_GEXTH;
    if(!WriteData(APDS9960_GEXTH, &data, 1))
        return 0;
    data = DEFAULT_GCONF1;
    if(!WriteData(APDS9960_GCONF1, &data, 1))
        return 0;
    if(!APDS9960_setGestureGain(DEFAULT_GGAIN))
        return 0;
    if(!APDS9960_setGestureLEDDrive(DEFAULT_GLDRIVE) )
        return 0;
    //기본 오프셋 설정
    data = 0;
    if(!WriteData(APDS9960_GOFFSET_U, &data, 1))
        return 0;
    data = 0;
    if(!WriteData(APDS9960_GOFFSET_D, &data, 1))
        return 0;
    data = 0;
    if(!WriteData(APDS9960_GOFFSET_L, &data, 1))
        return 0;
    data = 0;
    if(!WriteData(APDS9960_GOFFSET_R, &data, 1))
        return 0;
    data = DEFAULT_GPULSE;
    if(!WriteData(APDS9960_GPULSE, &data, 1))
        return 0;

    return 1;
}

int APDS9960_setProximityGain(uint8_t drive)
{
    uint8_t val;
    
    if( !ReadData(APDS9960_CONTROL, &val, 1) ) {
        return 0;
    }
    
    drive &= 0b00000011;
    drive = drive << 2;
    val &= 0b11110011;
    val |= drive;
    
    if( !WriteData(APDS9960_CONTROL, &val, 1) ) {
        return 0;
    }
    
    return 1;
}

int APDS9960_setProximityIntEnable(uint8_t enable)
{
    uint8_t val;
    
    if( !ReadData(APDS9960_ENABLE, &val, 1) ) {
        return 0;
    }
    
    enable &= 0b00000001;
    enable = enable << 5;
    val &= 0b11011111;
    val |= enable;
    
    if( !WriteData(APDS9960_ENABLE, &val, 1) ) {
        return 0;
    }
    
    return 1;
}

int APDS9960_enableProximity(int interrupt) {
    if( !APDS9960_setProximityGain(PGAIN_1X) ) {
        return 0;
    }
    if( !APDS9960_setLEDDrive(DEFAULT_LDRIVE) ) {
        return 0;
    }
    if( interrupt ) {
        if( !APDS9960_setProximityIntEnable(1) ) {
            return 0;
        }
    } else {
        if( !APDS9960_setProximityIntEnable(0) ) {
            return 0;
        }
    }
    if( !APDS9960_enablePower() ){
        return 0;
    }
    if( !APDS9960_setMode(PROXIMITY, 1) ) {
        return 0;
    }
}

//제스쳐 FIFO 레벨이 임계값보다 커지면 GVALID 비트 1
int APDS9960_isGestureAvailable()
{
    uint8_t val;
    
    if( !ReadData(APDS9960_GSTATUS, &val, 1) ) {
        return ERROR;
    }
    
    val &= APDS9960_GVALID;
    
    if( val == 1) {
        return 1;
    } else {
        return 0;
    }
}

int processGestureData(uint8_t *u_data, uint8_t *d_data, uint8_t *l_data, uint8_t *r_data, int total_gesture) {
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;
    //제스쳐 임계를 넘는 초기 값
    for(i = 0; i < total_gesture; i++) {
        if((u_data[i] > GESTURE_THRESHOLD_OUT) &&
            (d_data[i] > GESTURE_THRESHOLD_OUT) &&
            (l_data[i] > GESTURE_THRESHOLD_OUT) &&
            (r_data[i] > GESTURE_THRESHOLD_OUT) ) {
            
            u_first = u_data[i];
            d_first = d_data[i];
            l_first = l_data[i];
            r_first = r_data[i];
            break;
        }
    }
    if( (u_first == 0) || (d_first == 0) || (l_first == 0) || (r_first == 0) ) {
        return 0;
    }
    //제스쳐 임계를 넘는 마지막 값
    for(i = total_gesture - 1; i >= 0; i--) {
        if( (u_data[i] > GESTURE_THRESHOLD_OUT) &&
            (d_data[i] > GESTURE_THRESHOLD_OUT) &&
            (l_data[i] > GESTURE_THRESHOLD_OUT) &&
            (r_data[i] > GESTURE_THRESHOLD_OUT) ) {
            
            u_last = u_data[i];
            d_last = d_data[i];
            l_last = l_data[i];
            r_last = r_data[i];
            break;
        }
    }
    //비율 재기
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
    
    //처음과 마지막의 변화량
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;

    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;

    //변화량과 제스쳐 민감도 비교 
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }

    //절대값이랑 민감도 비교
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }
            //변화량에 따라 먼거리와 가까운 거리 재기
            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return 1;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }
            
            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }

    return 0;
}

int decodeGesture()
{
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        return 1;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        return 1;
    }
    
    //방향 재기
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_UP;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_DOWN;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_motion_ = DIR_RIGHT;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_motion_ = DIR_LEFT;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_UP;
        } else {
            gesture_motion_ = DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_DOWN;
        } else {
            gesture_motion_ = DIR_RIGHT;
        }
    } else {
        return 0;
    }
    
    return 1;
}

void resetGestureParameters()
{
    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;
    
    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;
    
    gesture_near_count_ = 0;
    gesture_far_count_ = 0;
    
    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

int APDS9960_readGesture() {
    uint8_t fifo_level = 0;
    uint8_t byte_read = 0;
    uint8_t fifo_data[128];
    uint8_t gstatus;
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
    int motion;
    
    if( !APDS9960_isGestureAvailable()) 
        return 0;

    while(1) {
        usleep(FIFO_PAUSE_TIME * 1000);

        if(!ReadData(APDS9960_GSTATUS, &gstatus, 1))
            return ERROR;
//FIFO 레벨이 FIFO 임계값에 도달하면 GVALID 발생 GMODE와 GFLVL이 0이되면 리셋
        if((gstatus & APDS9960_GVALID) == APDS9960_GVALID) {
            //GFIFO 레벨 읽기
            if(!ReadData(APDS9960_GFLVL, &fifo_level, 1))
                return ERROR;
//GFIFO level은 현재 읽을 수 있는 데이터 세트 수를 표시
            if(fifo_level > 0) {
                ReadData(APDS9960_GFIFO_U, fifo_data, (fifo_level * 4));
#ifdef DEBUG
                printf("Start\n");
                for(int i = 0; i < (fifo_level * 4); i++) {
                    printf("%d\n",fifo_data[i]); 
                }
#endif
                for(int i = 0, j = 0; i < fifo_level; i++,j += 4) {
                    u_data[i] = fifo_data[j + 0];
                    d_data[i] = fifo_data[j + 1];
                    l_data[i] = fifo_data[j + 2];
                    r_data[i] = fifo_data[j + 3];
                }
                if(processGestureData(u_data, d_data, l_data, r_data, fifo_level)) {
                    if(decodeGesture()) {

                    }
                }
            }

        } else {
            usleep(FIFO_PAUSE_TIME * 1000);
            decodeGesture();
            motion = gesture_motion_;    
            resetGestureParameters();
            return motion;
        }
    }
}



int main(int argc, char *argv[])
{
    uint8_t buf;
    
    if((fd = open(argv[1], O_RDWR)) < 0) {
        perror("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, APDS9960_I2C_ADDRESS) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    if(!APDS9960_Init()) {
        printf("Failed to call Device");
        exit(1);
    }
    APDS9960_enableProximity(0);
    APDS9960_setMode(GESTURE, 1);
    ReadData(APDS9960_ENABLE, &buf, 1);
    printf("%x\nSetup End\n", buf);
    while(1) {
        if(APDS9960_isGestureAvailable()) {
            switch ( APDS9960_readGesture())
            {
                case DIR_UP:
                    printf("UP\n");
                    break;
                case DIR_DOWN:
                    printf("DOWN\n");
                    break;
                case DIR_LEFT:
                    printf("LEFT\n");
                    break;
                case DIR_RIGHT:
                    printf("RIGHT\n");
                    break;
                case DIR_NEAR:
                    printf("NEAR\n");
                    break;
                case DIR_FAR:
                    printf("FAR\n");
                    break;
                default:
                    break;
            }
        }
    }

    return 0;
}