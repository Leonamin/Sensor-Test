#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

//커맨드 레지스터 커맨드
#define SI1145_PARAM_QUERY 0x80
#define SI1145_PARAM_SET 0xA0
#define SI1145_NOP 0x0
#define SI1145_RESET    0x01
#define SI1145_BUSADDR    0x02
#define SI1145_PS_FORCE    0x05
#define SI1145_ALS_FORCE    0x06
#define SI1145_PSALS_FORCE    0x07
#define SI1145_PS_PAUSE    0x09
#define SI1145_ALS_PAUSE    0x0A
#define SI1145_PSALS_PAUSE    0xB
#define SI1145_PS_AUTO    0x0D
#define SI1145_ALS_AUTO   0x0E
#define SI1145_PSALS_AUTO 0x0F
#define SI1145_GET_CAL    0x12

//빠라밋떠
#define SI1145_PARAM_I2CADDR 0x00
#define SI1145_PARAM_CHLIST   0x01
#define SI1145_PARAM_CHLIST_ENUV 0x80
#define SI1145_PARAM_CHLIST_ENAUX 0x40
#define SI1145_PARAM_CHLIST_ENALSIR 0x20
#define SI1145_PARAM_CHLIST_ENALSVIS 0x10
#define SI1145_PARAM_CHLIST_ENPS1 0x01
#define SI1145_PARAM_CHLIST_ENPS2 0x02
#define SI1145_PARAM_CHLIST_ENPS3 0x04

#define SI1145_PARAM_PSLED12SEL   0x02
#define SI1145_PARAM_PSLED12SEL_PS2NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS2LED1 0x10
#define SI1145_PARAM_PSLED12SEL_PS2LED2 0x20
#define SI1145_PARAM_PSLED12SEL_PS2LED3 0x40
#define SI1145_PARAM_PSLED12SEL_PS1NONE 0x00
#define SI1145_PARAM_PSLED12SEL_PS1LED1 0x01
#define SI1145_PARAM_PSLED12SEL_PS1LED2 0x02
#define SI1145_PARAM_PSLED12SEL_PS1LED3 0x04

#define SI1145_PARAM_PSLED3SEL   0x03
#define SI1145_PARAM_PSENCODE   0x05
#define SI1145_PARAM_ALSENCODE  0x06

#define SI1145_PARAM_PS1ADCMUX   0x07
#define SI1145_PARAM_PS2ADCMUX   0x08
#define SI1145_PARAM_PS3ADCMUX   0x09
#define SI1145_PARAM_PSADCOUNTER   0x0A
#define SI1145_PARAM_PSADCGAIN 0x0B
#define SI1145_PARAM_PSADCMISC 0x0C
#define SI1145_PARAM_PSADCMISC_RANGE 0x20
#define SI1145_PARAM_PSADCMISC_PSMODE 0x04

#define SI1145_PARAM_ALSIRADCMUX   0x0E
#define SI1145_PARAM_AUXADCMUX   0x0F

#define SI1145_PARAM_ALSVISADCOUNTER   0x10
#define SI1145_PARAM_ALSVISADCGAIN 0x11
#define SI1145_PARAM_ALSVISADCMISC 0x12
#define SI1145_PARAM_ALSVISADCMISC_VISRANGE 0x20

#define SI1145_PARAM_ALSIRADCOUNTER   0x1D
#define SI1145_PARAM_ALSIRADCGAIN 0x1E
#define SI1145_PARAM_ALSIRADCMISC 0x1F
#define SI1145_PARAM_ALSIRADCMISC_RANGE 0x20

#define SI1145_PARAM_ADCCOUNTER_511CLK 0x70

#define SI1145_PARAM_ADCMUX_SMALLIR  0x00
#define SI1145_PARAM_ADCMUX_LARGEIR  0x03

//리지스따
//
#define SI1145_REG_PARTID  0x00
#define SI1145_REG_REVID  0x01
#define SI1145_REG_SEQID  0x02

#define SI1145_REG_INTCFG  0x03
#define SI1145_REG_INTCFG_INTOE 0x01
#define SI1145_REG_INTCFG_INTMODE 0x02

#define SI1145_REG_IRQEN  0x04
#define SI1145_REG_IRQEN_ALSEVERYSAMPLE 0x01
#define SI1145_REG_IRQEN_PS1EVERYSAMPLE 0x04
#define SI1145_REG_IRQEN_PS2EVERYSAMPLE 0x08
#define SI1145_REG_IRQEN_PS3EVERYSAMPLE 0x10


#define SI1145_REG_IRQMODE1 0x05
#define SI1145_REG_IRQMODE2 0x06

#define SI1145_REG_HWKEY  0x07
#define SI1145_REG_MEASRATE0 0x08
#define SI1145_REG_MEASRATE1  0x09
#define SI1145_REG_PSRATE  0x0A
#define SI1145_REG_PSLED21  0x0F
#define SI1145_REG_PSLED3  0x10
#define SI1145_REG_UCOEFF0  0x13
#define SI1145_REG_UCOEFF1  0x14
#define SI1145_REG_UCOEFF2  0x15
#define SI1145_REG_UCOEFF3  0x16
#define SI1145_REG_PARAMWR  0x17
#define SI1145_REG_COMMAND  0x18
#define SI1145_REG_RESPONSE  0x20
#define SI1145_REG_IRQSTAT  0x21
#define SI1145_REG_IRQSTAT_ALS  0x01

#define SI1145_REG_ALSVISDATA0 0x22
#define SI1145_REG_ALSVISDATA1 0x23
#define SI1145_REG_ALSIRDATA0 0x24
#define SI1145_REG_ALSIRDATA1 0x25
#define SI1145_REG_PS1DATA0 0x26
#define SI1145_REG_PS1DATA1 0x27
#define SI1145_REG_PS2DATA0 0x28
#define SI1145_REG_PS2DATA1 0x29
#define SI1145_REG_PS3DATA0 0x2A
#define SI1145_REG_PS3DATA1 0x2B
#define SI1145_REG_UVINDEX0 0x2C
#define SI1145_REG_UVINDEX1 0x2D
#define SI1145_REG_PARAMRD 0x2E
#define SI1145_REG_CHIPSTAT 0x30

#define SI1145_ADDR 0x60

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
//p는 쓰려는 인자 RAM, v는 쓰려는 인자
void WriteParam(uint8_t p, uint8_t v) {
    uint8_t data = v;
    WriteData(SI1145_REG_PARAMWR, &v, 1);
    data = p | SI1145_PARAM_SET;
    WriteData(SI1145_REG_COMMAND, &data, 1);
}

void RESET() {
    uint8_t data;
    data = 0x00;
    WriteData(SI1145_REG_MEASRATE0, &data, 1);
    WriteData(SI1145_REG_MEASRATE1, &data, 1);
    WriteData(SI1145_REG_IRQEN, &data, 1);
    WriteData(SI1145_REG_IRQMODE1, &data, 1);
    WriteData(SI1145_REG_IRQMODE2, &data, 1);
    WriteData(SI1145_REG_INTCFG, &data, 1);
    data = 0xFF;
    WriteData(SI1145_REG_IRQSTAT, &data, 1);
    data = SI1145_RESET;
    WriteData(SI1145_REG_COMMAND, &data, 1);    //Reset 명령어 0x01;
    usleep(10 * 1000);
    data = 0x17;
    WriteData(SI1145_REG_HWKEY, &data, 1);      //HWKEY 레지스터에 0x17을 입력해야 Standby 모드로 진입

    usleep(10 * 1000);
}

void INIT() {
    uint8_t id, data;
    ReadData(SI1145_REG_PARTID, &id, 1);
    if(id != 0x45) {
        printf("Different with part id . . . (%x)\n", id);
        exit(1);
    }
    printf("Success Check ID\n");
    RESET();
    
    // enable UVindex measurement coefficients!
    //UV인덴스 측정 계수 허용 데이터시트상의 기본값은 차례대로 0x29, 0x89, 0x02, 0x00
    data = 0x29;
    WriteData(SI1145_REG_UCOEFF0, &data, 1);
    data = 0x89;
    WriteData(SI1145_REG_UCOEFF1, &data ,1);
    data = 0x02;
    WriteData(SI1145_REG_UCOEFF2, &data, 1);
    data = 0x00;
    WriteData(SI1145_REG_UCOEFF3, &data, 1);

    // UV 센서 허용
    WriteParam(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV |
    SI1145_PARAM_CHLIST_ENALSIR | SI1145_PARAM_CHLIST_ENALSVIS |
    SI1145_PARAM_CHLIST_ENPS1);

    // 매 샘플마다 인터럽트 허용
    data = SI1145_REG_INTCFG_INTOE;
    WriteData(SI1145_REG_INTCFG, &data, 1);
    data = SI1145_REG_IRQEN_ALSEVERYSAMPLE;
    WriteData(SI1145_REG_IRQEN, &data, 1);  

    /*근점 감지 1번*/

    // LED 전류 설정
    data = 0x03;
    WriteData(SI1145_REG_PSLED21, &data, 1); // LED 1에만 20mA
    WriteParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
    // 근접 감지 1센서 LED 1번 사용
    WriteParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
    // 게인값
    WriteParam(SI1145_PARAM_PSADCGAIN, 0);
    //
    WriteParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    WriteParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE|
        SI1145_PARAM_PSADCMISC_PSMODE);
    //ADC 입력 선택
    WriteParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
    //ADC 게인값
    WriteParam(SI1145_PARAM_ALSIRADCGAIN, 0);
    //ADC 클럭값
    WriteParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    //ADC 범위
    WriteParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);

    WriteParam(SI1145_PARAM_ALSVISADCGAIN, 0);
    WriteParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
    WriteParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);


    /************************/

    // 자동 측정
    data = 0xFF;
    WriteData(SI1145_REG_MEASRATE0, &data, 1); // 255 * 31.25uS = 8ms
    
    // 자동 실행
    data = SI1145_PSALS_AUTO;
    WriteData(SI1145_REG_COMMAND, &data, 1);
}

int readUV() {
    int data;
    uint8_t buf[2];
    ReadData(SI1145_REG_UVINDEX0, buf, 2);
    data = buf[0] + (buf[1] << 8);
    return data;
}

int readIR() {
    int data;
    uint8_t buf[2];
    ReadData(SI1145_REG_ALSIRDATA0, buf, 2);
    data = buf[0] + (buf[1] << 8);
    return data;
}

int readVisible() {
    int data;
    uint8_t buf[2];
    ReadData(SI1145_REG_ALSVISDATA0, buf, 2);
    data = buf[0] + (buf[1] << 8);
    return data;
}

int readProx() {
    int data;
    uint8_t buf[2];
    ReadData(SI1145_REG_PS1DATA0, buf, 2);
    data = buf[0] + (buf[1] << 8);
    return data;
}

int main(int argc, char *argv[]) 
{
    int uv, visible, ir, prox;
    if((fd = open(argv[1], O_RDWR)) < 0) {
        printf("Failed to open i2c-0");
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, SI1145_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave\n");
        exit(1);
    }
    INIT();

    while(1) {
        uv = readUV();
        visible = readVisible();
        ir = readIR();
        prox = readProx();
        printf("UV: %d, Visible Light: %d, IR: %d, Proximity: %d\n", uv, visible, ir, prox);
        sleep(1);
    }

    return 0;
}