# 1. APDS-9960
## 1.1 간단 소개
### APDS9960은 AVAGO Technologies에서 만든 근접 감지, 조도, RGB 그리고 제스쳐 감지 센서이다. 4개의 광다이오드로 측정하며 I2C 통신을 지원한다.
## 1.2 핀 배치
| 핀 | 이름 | 타입 | 상세 |
|:---|:---:|:---:|:---|
| 1 | SDA | I/O | I2C 시리얼 데이터 I/O 터미널 |
| 2 | INT | O | 인터럽트핀(오픈드레인) |
| 3 | LDR | | 근접 IR LED용 LED 드라이버 입력, 정전류 소스 LED 드라이버 |
| 4 | LEDK | | LED 음극, 내부 LED 드라이버 회로 사용 시 LDR 핀에 연결 |
| 5 | LEDA | | LED 양극, PCB의 V LEDA에 연결 |
| 6 | GND | | GND |
| 7 | SCL | | I2C 시리얼 클락 입력 터미널 |
| 8 | VDD | | 전원공급 | 
***
# 2. 센서 구조
## 2.1 I2C 버스 프로토콜
### APDS-9960의 슬레이브 주소는 7비트의 0x39이다. Write같은 경우 슬레이브 주소, 레지스터 주소, 데이터를 보내면 된다. 예를 들면 0x39, 0x80, 0x40 순으로 보내게 되면 슬레이브 주소가 0x39인 디바이스, APDS-9960의 0x80 레지스터의 값을 0x40으로 바꾸는 것이다. Read같은 경우 슬레이브 주소만 보내면 되며 Combined 포맷의 경우 슬레이브 주소, 레지스터 주소, 슬레이브 주소 순으로 보낸 후 데이터를 읽으면 된다.
| | |
|---|:---|
|A|Acknowledge(0)|
|N|Not Acknowledge(1)|
|P|Stop Condition|
|R|Read(1)|
|S|Start Condition|
|W|Write|
|...|Continuation of Protocol|

`Write`
|1|7|1|1|8|1|8|1| |1|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|S|Slave Address|W|A|Register Address|A|Data|A|...|P|
`Read`
|1|7|1|1|8|1|8|1| |1|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|S|Slave Address|W|A|Data|Data|A|...|P|
`Read-Combined Format`
|1|7|1|1|8|1|1|7|1|1|8|1|8|1| |1|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|S|Slave Address|W|A|Register Address|A|Sr|Slave Addresss |R|A|Data|A|Data|A|...|P

## 2.2 센서 동작
### APDS-9960은 전원이 들어올 경우 즉시 저전력 모드인 Sleep 상태로 들어가게 된다. 이 때 ENABLE 레지스터의 PON 비트가 1로 설정되면 Sleep Mode에서 나오게 되고 PEN, GEN, AEN 비트가 설정될 때 까지 다시 전력소모를 적게 소모하며 대기 상태로 있게된다. 기능 엔진이 설정되면 Sleep 상태에서 빠져나오게 되고 선택한 엔진으로 바로 입력된다. 엔진이 여러개가 활성화 되면 IDLE, 근접, 제스처(GMODE=1), 대기, 색상/ALS 및 절전(SAI가 1이고 INT핀이 0인 경우) 순으로 동작한다. 
## 2.3 근접 감지 동작
### 근접 감지 결과는 IR LED 방출, IR 수신, 목표 거리와 표면 반사율을 포함한 환경 요인의 3가지 기본 요인에 의해 영향을 받는다. <br>IR 수신 신호 경로는 4개의 [방향 제스처] 광다이오드로부터 IR 검출로 시작되며, PDATA 레지스터에서 8비트 근접 결과로 종료된다.<br> 광다이오드의 신호를 조합, 증폭, 오프셋을 조정하여 성능을 최적화한다.<br> 근접 조작뿐만 아니라 제스처 조작에도 동일한 4개의 광다이오드를 사용한다.<br> 다이오드는 UP/RIGHT와 DOWN/LEFT의 두 가지 신호 경로를 형성하기 위해 쌍을 이룬다.<br> 페어링에 관계 없이 광다이오드는 근위부 결과에 대한 기여를 배제하기 위해 가릴 수 있다.<br>페어링된 다이오드 중 하나를 마스킹하면 신호가 효과적으로 절반으로 감소하며, 풀 스케일 결과가 255에서 127로 감소한다.<br> 이러한 감소를 본격화하기 위해, F.S.를 255로 되돌리는 근접 이득 보상 비트인 PCMP를 설정할 수 있다.<br> 게인은 PGAIN 제어 비트를 사용해 1배에서 8배까지 조정할 수 있다.<br> 오프셋 보정 또는 크로스 토크 보상은 POFFSET_UR 및 POFSET_DL 레지스터에 대한 조정으로 이루어진다.<br>장치의 아날로그 회로는 오프셋 값을 신호 축적의 하위 트레이스로 적용하므로 양의 오프셋 값은 결과를 감소시키는 효과를 갖는다.<br> 시각적으로 IR배출은 맥박열차로 나타난다. 펄스 수는 PPULSE 비트에 의해 설정되며, 각 펄스의 주기는 PPLEN 비트를 사용하여 조정할 수 있다.<br> IR 배출물의 인텐시티는 LDRIVE 제어 비트를 사용하여 선택할 수 있으며, 이는 4개의 공장 교정 전류 레벨에 해당한다.<br> 더 높은 강도(예: 더 긴 탐지 거리 또는 어두운 유리 아래 장치 배치)가 필요한 경우 LEDBOOST 비트를 사용하여 전류를 300%까지 증가시킬 수 있다.
### 인터럽트는 각각의 새로운 근접 결과로 또는 근접 결과가 PIHT 및/또는 PILT 임계값 레지스터에 설정된 레벨을 초과하거나 이하로 떨어질 때마다 발생할 수 있다.<br>조기/잘못된 인터럽트를 방지하기 위해 인터럽트 지속성 필터도 포함되었다. <br>인터럽트는 연속적인 임계치 초과 결과 수가 PPERS가 설정한 값과 같거나 더 큰 경우에만 주장될 것이다. <br>각 "임계값" 근접 결과, PDATA는 지속성 카운트를 재설정한다. <br>아날로그 회로가 포화 상태가 되면 PGSAT 비트는 PDATA 결과가 정확하지 않을 수 있음을 나타낸다. <br>PINT와 PGSAT 비트는 항상 I2C 폴링에 사용할 수 있지만, PIEN 비트는 PINT가 INT 핀에서 하드웨어 인터럽트를 주장하도록 설정되어야 한다. <br>마찬가지로, PGSAT 비트를 폴링하여 아날로그 데이터 컨버터의 포화도를 감지할 수 있다. <br>이 기능을 활성화하려면 PSIEN 비트가 설정되어야 한다. PVALID는 PDATA를 읽음으로써 삭제된다. <br>PGSAT와 PINT는 "주소 액세스"(즉, 칩 주소, R/W=1) PICLEAR 또는 AICLEAR의 레지스터 주소의 2바이트로 구성된 I2C 트랜잭션)에 의해 지워진다.

## 2.4 ALS/컬러 감지 동작
### 색상과 주변 광 감지 검출 기능은 일련의 색상과 IR 필터링된 광다이오드를 사용하여 빛의 적색, 녹색 및 청색 함량과 비색 필터링된 클리어 채널을 측정한다.<br>색상/ALS 수신 신호 경로는 광다이오드에서의 필터링된 RGBC 검출로 시작하여 RGBC 데이터 레지스터의 16비트 결과로 끝난다. <br>광다이오드 어레이의 신호는 결과가 RGBCDATA 레지스터에 배치되기 전에 ATIME의 값에 의해 설정된 시간 동안 누적된다. <br>게인은 1배에서 64배까지 조정할 수 있으며, CONTROL <AGAIN>의 설정에 따라 결정된다. <br>정확도, 해상도, 변환 속도, 전력 소비량 등의 성능 특성을 어플리케이션의 요구에 맞게 조정할 수 있다. 컬러/ALS 엔진에 진입(재입력)하기 전에 조정 가능한 저전력 소비, 지연을 입력한다. <br>이 지연에 대한 대기 시간은 WEN, WTIME 및 WLONG 제어 비트를 사용하여 선택할 수 있으며 범위는 0-8.54s이다. <br>이 기간 동안 내부 오실레이터는 여전히 작동 중이지만, 다른 모든 회로는 비활성화된다. <br>Clear Channel 결과가 AILTL/AIHTL 및/또는 AILTH/AIHTH 임계값 레지스터에 설정된 수준을 초과하거나 이하로 떨어질 때마다 인터럽트가 발생할 수 있다. <br>조기/거짓 간섭을 방지하기 위해 지속성 필터도 포함되었다. <br>인터럽트는 연속적인 임계치 초과 결과 수가 APERS가 설정한 값과 같거나 더 큰 경우에만 주장될 것이다. <br>각 "임계값" 클리어 채널 결과, CDATA는 지속성 카운트를 재설정한다. <br>아날로그 회로가 포화 상태가 되면 ASAT 비트는 RGBCDATA 결과가 정확하지 않을 수 있음을 나타낸다. <br>AINT와 CPSAT 비트는 I2C 폴링에 항상 사용 가능하지만, AIEN 비트는 AINT가 INT 핀에 하드웨어 인터럽트를 나타내기 위해 설정되어야 한다. <br>마찬가지로, 아날로그 데이터 변환기의 포화는 CPSAT 비트를 폴링하여 감지할 수 있다. <br>이 기능을 활성화하려면 CPSIEN 비트를 설정해야 한다. <br>AVALID는 RGBCDATA. ASAT를 읽음으로써 지워지고, AINT는 "주소 액세스"(즉, 칩 어드레스, R/W=1로 구성된 레지스터 어드레스, CICLEAR 또는 AICLEAR)로 구성된 I2C 트랜잭션에 의해 지워진다. <br>RGBC 결과를 사용해 주변 조도(예: 럭스)와 색온도(예: Kvine)를 계산할 수 있다.

## 제스쳐 감지 동작
### 제스쳐 감지 동작은 통합 LED에 의해 소싱된 반사 IR 에너지를 감지하기 위해 방향적으로 민감한 광다이오드를 활용하여 동작 검출 기능을 제공한다.<br> 제스처 결과는 IR LED 방출, IR 수신 및 움직임을 포함한 환경 요인의 세 가지 기본 요소에 의해 영향을 받는다. <br>작동 중, 제스쳐 엔진은 활성화 비트, GEN과 작동 모드 비트인 GMODE가 모두 설정되면 입력된다. <br>GMODE는 I2C를 통해 수동으로 설정/재설정할 수 있으며, 근접 결과 PDATA가 제스처 근접 입력 임계값인 GPENTH와 같다면 설정된다. <br>제스처 엔진의 종료는 GMODE가 0으로 재설정될 때까지 발생하지 않는다. <br>정상 작동 중, GEXPER 시간에 대해 모든 4바이트 제스쳐 데이터 세트가 종료 임계값인 GEXTH 아래로 떨어지면 GMODE가 재설정된다. <br>또한 이 종료 조건은 마스크가 없는 모든 기준점(단일 1바이트 U, D, L, R 지점)을 포함하는 제스처 종료 마스크 GEXMSK에 의해 영향을 받는다. <br>조기 종료를 방지하기 위해 지속성 필터도 포함되며, 연속적인 임계치 이하의 결과가 지속성 값인 GEXPERS와 같거나 큰 경우에만 종료가 발생한다. <br>임계값 이상인 각 데이터 집합 결과는 지속성 카운트를 재설정한다. <br>거짓 또는 불완전한 제스쳐(GVALID가 높게 전환되지 않은 엔진 입출구)는 혼내, GINT, FIFO 데이터가 자동으로 제거된다. <br>일단 제스처 엔진 내부에서 작동하면 IR 수신 신호 경로는 포토디에서 IR 검출로 시작하여 각 다이오드의 누적 신호 강도로 4개의 8비트 제스처 결과로 끝난다. 4개의 광다이오드로부터의 신호를 증폭하고, 성능 최적화를 위해 오프셋을 조정한다. <br>포토다이오드는 UP/DOWN과 LEFT/RIGHT의 두 가지 신호 경로를 형성하기 위해 쌍을 구성한다. 포토다이오드 쌍은 제스처 FIFO 데이터에서 결과를 제외하기 위해 마스킹할 수 있다. <br>예를 들어 UP-DOWN 동작 검출만 필요한 경우 제스처 치수 제어 비트인 GDIMS를 0x01로 설정할 수 있다. <br>FIFO 데이터는 RIGHT/LEFT 결과에 대해 0이 되며 축적/ADC 통합 시간은 대략 절반으로 단축된다. 
### 게인은 GGAIN 제어 비트를 사용해 1배에서 8배까지 조정할 수 있다. 오프셋 보정은 크로스 토크 성능을 개선하기 위해 GOFFSET_U, GOFFSET_D, GOFFSET_L, GOFFSET_R 레지스터를 개별적으로 조정하여 이루어진다. 장치의 아날로그 회로는 신호 발생에 대한 감산으로서 오프셋 값을 적용하므로 양의 오프셋 값은 결과를 감소시키는 효과를 갖는다. 시각적으로 IR배출은 맥박열차로 나타난다. 펄스 수는 GPULSE 비트에 의해 설정되며, 각 펄스의 주기는 GPLEN 비트를 사용하여 조정할 수 있다. 펄스열 반복(즉, 제스처 대기 시간 비트, GWTIME에서 0이 아닌 값을 설정함으로써 동작 상태 기계 내부의 순환 작동 흐름)을 지연시킬 수 있다. 대기 상태를 포함하면 전력 소산과 데이터 속도가 모두 감소한다. IR 배출 강도는 4개의 공장 교정 전류 레벨에 해당하는 GLDRIVE 제어 비트를 사용하여 선택할 수 있다. 더 높은 강도(예: 더 긴 탐지 거리 또는 어두운 유리 아래 장치 배치)가 필요한 경우 LEDBOOST 비트를 사용하여 전류를 300%까지 증가시킬 수 있다. 통합 IR LED의 현재 소비량은 표 5(다양한 LED 드라이브 설정에서 3가지 예) FIFO에 배치된 제스처 "데이터베이스" 결과의 수에 기초하여 인터럽트가 생성된다. 데이터 세트는 U-D-L-R에 해당하는 4바이트 방향 데이터로 정의된다.FIFO는 넘치기 전에 최대 32개의 데이터셋을 버퍼링할 수 있다. FIFO가 오버플로되면(호스트가 충분히 빨리 읽지 않음) 가장 최근의 데이터는 손실된다. FIFO 레벨인 GFLVL이 GFIFOTH에 의해 설정된 임계값보다 크거나 같아지면 GVALID 비트가 설정되고 유효한 데이터가 있음을 나타낸다. 제스처 인터럽트 비트인 GINT가 강조 표시되며, GIEN 비트가 INT 핀에 하드웨어 인터럽트를 설정하는 경우 INT 핀의 하드웨어 인터럽트도 주장하게 된다. 제스처 엔진을 종료하기 전에 GVALID가 주장될 경우 항상 최종 중단이 발생하며, 신호 데이터가 FIFO에 남아 있다. Gesture 인터럽트 플래그: GINT, GVALID 및 GFLVL은 e를 비우면 지워진다. 모든 데이터가 읽힌다.) FIFO 데이터와 방향 차동학과의 상관관계는 얼핏 보면 명확하지 않다. 그림 12에서 묘사한 바와 같이 4개의 방향 센서는 직교 패턴의 광학렌즈 구멍에 배치된다. 다이오드는 U, D, L, R로 지정되며, 각 다이오드에 해당하는 8비트 결과는 0xFC, 0xFD, 0xFE 및 0xFF의 순차 FIFO 위치에서 사용할 수 있다. 이상적으로 제스처 검출은 정밀 센서 결과 사이의 진폭과 위상 차이를 캡처하고 비교함으로써 작동한다. 방향 센서는 방향 운동과 반대되는 다이오드가 진입 시 반사된 IR 신호에서 더 큰 부분을 수신하고, 종료 시 더 작은 부분을 수신하도록 배치된다. 일광욕의 예에서, 표적의 아래 또는 오른쪽 움직임은 그림 11의 각 화살표에 의해 도해되어 있다. Gesture 모드에서 RAM 영역은 32 x 4 바이트 FIFO로 용도가 변경된다. 데이터는 네 개의 바이트 블록으로 저장된다. 데이터 집합이라 불리는 각 블록에는 UP, DOWN, LEFT, 및 RIGHT 제스처 데이터가 하나의 통합 사이클로 포함되어 있다. 32개의 별도 데이터세트가 FIFO 내에 저장된다. FIFO가 오버플로우하면(즉, 호스트/시스템이 FIFO를 비우기 전 33개 데이터셋) 새로운 데이터셋이 기존 데이터셋을 대체하지 않는다. 대신에 오버플로 플래그가 설정되고 새로운 데이터가 손실된다. 호스트/시스템은 UP, DOWN, LEFT 및 RIGHT 데이터 포인트에 직접 해당하는 0xFC, 0xFD, 0xFE 및 0xFF의 주소를 읽음으로써 제스처 데이터를 획득한다. 데이터는 한 번에 하나의 바이트(4개의 연속 I2C 트랜잭션)를 읽거나 페이지 판독을 사용하여 읽을 수 있다. 내부 FIFO 읽기 포인터와 FIFO Level 레지스터인 GFLVL은 주소 0xFF에 액세스하거나 주소 0xFF에 해당하는 4바이트마다 페이지 모드로 액세스할 때 값이 업데이트된다. GFLVL 레지스터가 0인 후에도 FIFO에 계속 접속하면 데이터셋은 0 값으로 판독된다. FIFO에 저장된 데이터를 읽기 위한 권장 절차는 제스처 인터럽트가 생성될 때 시작한다(GFLVL > GIFOTH). 다음으로, 호스트는 FIFO Level 레지스터 GFLVL을 읽어 FIFO의 유효한 데이터 양을 결정한다. 마지막으로, 호스트는 주소 0xFC(읽기 페이지)를 읽기 시작하고, FIFO가 비어 있을 때까지(바이트 수는 4배 GFLVL) 읽기(클록아웃 데이터)를 계속한다. 예를 들어 GFLVL이 2인 경우 호스트는 주소 0xFC에서 판독을 시작하고 8바이트를 순차적으로 모두 읽어야 한다. 4바이트 블록을 읽으면 GFLVL 레지스터가 감소되고 내부 FIFO 포인터가 업데이트된다.