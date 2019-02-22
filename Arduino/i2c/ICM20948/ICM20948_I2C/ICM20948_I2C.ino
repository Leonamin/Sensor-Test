#include <Wire.h>


//레지스터 이름은 데이터시트에서 참고
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

typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
    uint8_t t_h;
    uint8_t t_l;
  } reg;
  struct 
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int x_gyro;
    int y_gyro;
    int z_gyro;
    int temperature;
  } value;
};

float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}


//===================================================================================
//ICM20948_read
//인자 목록
//reg: 시작 주소, 레지스터 정의에 사용할 주소
//buffer: 데이터를 담으려는 변수
//size: 받으려는 바이트
int ICM20948_read(int reg, uint8_t *buffer, int size) {
    int i, n, error;
    //전송할 Slave 주소값 지정
    Wire.beginTransmission(ICM20948_I2C_ADDRESS);
    //읽을 레지스터 reg
    n = Wire.write(reg);
    if (n != 1)
        return -10;

    n = Wire.endTransmission();        //I2C 버스 지속
    if (n != 0)
        return n;
    //size만큼의 데이터 요청 후 stop 비트 true
    Wire.requestFrom(ICM20948_I2C_ADDRESS, size, true);
    i = 0;
    //사이즈 만큼 비트 읽기
    while(Wire.available() && i < size) {
        buffer[i++] = Wire.read();
    }
    if (i != size)
        return -11;
    return 0;
}
//===================================================================================
//ICM20948_write
//인자 목록
//reg: 시작 주소, 레지스터 정의에 사용할 주소
//pData: 데이터 쓰려는 포인터
//size: 쓰려는 바이트 숫자
int ICM20948_write(int reg, const uint8_t *pData, int size) {
    int n, error;

    Wire.beginTransmission(ICM20948_I2C_ADDRESS);
    n = Wire.write(reg);
    if (n != 1)
        return -20;
    n = Wire.write(pData, size);
    if (n != size)
        return -21;
    error = Wire.endTransmission(true);     //I2C 버스 종료
    if (error != 0)
        return error;
    
    return 0;
}

//====================================================================================
//ICM20948_write_reg
//레지스터 하나만
//인자목록
//reg: 단일 주소
//data: 데이터
int ICM20948_write_reg(int reg, uint8_t data) {
    int error;
    
    error = ICM20948_write(reg, &data, 1);

    return error;
}

int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value
  
  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
   
  int error = ICM20948_read (ICM20948_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);

  return error;
}

void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  
  //Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}

void setup() {
    int error;
    uint8_t c;
    Serial.begin(115200);
    Serial.println("Serial Connected");
    Wire.begin();
    Serial.println("I2C Bus Clear");

    error = ICM20948_read(ICM20948_WHO_AM_I, &c, 1);
    Serial.print("WHO AM I : ");
    Serial.print(c,HEX);
    Serial.print("ERROR : ");
    Serial.println(error, DEC);
    
    //sleep 비트 읽기 1이면 비활성 0이면 활성
    //5:3 가속도 2:0 자이로 
    error = ICM20948_read(ICM20948_PWR_MGMT_2, &c, 1);
    Serial.print("PWR_MGMT_2 : ");
    Serial.print(c,HEX);
    Serial.print("ERROR : ");
    Serial.println(error, DEC);
    //sleep 비트 0으로 클리어하여 센서 동작 시작
    error = ICM20948_write_reg(ICM20948_PWR_MGMT_1, 0);
    calibrate_sensors();
    set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);

}

void loop() {

  
    int error;
    double dT;
    accel_t_gyro_union accel_t_gyro;


    error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);

    unsigned int t_now = millis();

    //자이로로 각 구하기

    //각속도
    float FS_SEL = 131;
    float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro)/FS_SEL;
    float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro)/FS_SEL;
    float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro)/FS_SEL;

    // Compute the (filtered) gyro angles
    float dt =(t_now - get_last_time())/1000.0;
    float gyro_angle_x = gyro_x*dt + get_last_x_angle();
    float gyro_angle_y = gyro_y*dt + get_last_y_angle();
    float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

    //가속도로 각도 구하기
    float accel_x = accel_t_gyro.value.x_accel;
    float accel_y = accel_t_gyro.value.y_accel;
    float accel_z = accel_t_gyro.value.z_accel;
    //라디안
    float RADIANS_TO_DEGREES = 180/3.14159;
    
    //가속도 구하는 식 z축은 고정
    float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

    float accel_angle_z = 0;

    //상보필터
    float alpha = 0.96;
    float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
    float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
    float angle_z = gyro_angle_z;
    
    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    //Serial.println(t_now - get_last_time());
    /*
    Serial.print(accel_t_gyro.value.x_gyro);
    Serial.print("\t");
    Serial.print(accel_t_gyro.value.y_gyro);
    Serial.print("\t");
    Serial.print(accel_t_gyro.value.z_gyro);
    Serial.print("\n");
    delay(100);
    */
   /*
    Serial.print("Accel X, Y, Z: \t");
    Serial.print(accel_angle_x);
    Serial.print("\t");
    Serial.print(accel_angle_y);
    Serial.print("\t");
    Serial.print(accel_angle_z);
    Serial.println();
    Serial.print(dt);
    Serial.println();
    delay(100);
    */

    Serial.print(accel_t_gyro.value.x_accel);
    Serial.print("\t");
    Serial.print(accel_t_gyro.value.y_accel);
    Serial.print("\t");
    Serial.print(accel_t_gyro.value.z_accel);
    Serial.print("\n");
    delay(100);
    /*
    Serial.print("Angle X, Y, Z: \t");
    Serial.print(angle_x);
    Serial.print("\t");
    Serial.print(angle_y);
    Serial.print("\t");
    Serial.print(angle_z);
    Serial.println();
    delay(100);
    */
    
}
