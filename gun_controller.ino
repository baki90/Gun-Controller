#include <Keyboard.h>
#include <Mouse.h>

/* 
 * 0x30 DDRF 
 * 0x2F PINF
 * 0x2B PORTD
 * 0x2A DDR
 * 0x29 PIND
 * 0x28 PORTC
 * 0x27 DDRC
 * 0x26 PINC
 */


#define F_CPU 16000000UL //16MHZ

//about flag
#define INPUT 10
#define OUTPUT 11
#define HIGH 12
#define LOW 13
#define R 14
#define W 15

/* START_STATE
 * button이 입력되면 해당 기기가 작동을 시작한다. 
 * 이는 토글 형식으로, 다시 button을 click하면 state 0으로 돌아간다.
 * 0 : not start
 * 1 : 상보 필터 init, 각 data 초기값 read
 * 2 : 기기 작동
 */
 
int START_STATE = 0; 

//pinMode, read, write method
void _pinMode(unsigned int addr, unsigned int offset, int mode); 
int _read(unsigned int addr, unsigned int offset); //PORTC 
void _write(unsigned int addr, unsigned int offset, int mode);

//about i2c (MPU_6050)

volatile byte gyro_addr = 0x68; //MPU6050 WHO_AM_I value
int16_t a_x, a_y, a_z, temp, g_x, g_y, g_z; //초기에 i2c로 받아오는 값을 정제한 것을 담아두는 변수
float init_data[6];
float aAngle_x, aAngle_z, gAngle_x, gAngle_z, filter_x =0, filter_z=0, gX, gZ;
uint64_t t_now, t_prev;

void i2c_init();
void i2c_start();
void i2c_stop();
void i2c_addr(unsigned int flag);
byte i2c_read(byte addr);
void i2c_send(byte addr, int flag);
void i2c_write(byte addr, char data);
void getData();

//about ADC (joystick)
uint16_t joy[2]; 
void readJoystick();
void adc_init();
void adc_clear();
uint16_t adc_read(uint8_t offset);

//about interrupt, delay, timer
uint64_t timer_cnt = 0;
void timer_setup();
void interrupt_setup();
void delay_us(unsigned int time_us);
void delay_ms(unsigned int time_ms);
uint64_t _millis();

void filter_init();
void filter_cal();
void filter_axis();
void filter_gyro();
void calcDT();

float dt = 0;

int8_t btn_prev = 0x00;
int R_light_flag = 0;

void setup() {
  Serial.begin(115200);
  Mouse.begin();
  Keyboard.begin();

  //YELLO LED ON 전원 판단, LED Y C7 , G D4, R D6 
  DDRD |= (1<<4);
  DDRD |= (1<<6);
  PORTC |= (1<<7);
  adc_init(); //adc init
  i2c_init(); //i2c init
  i2c_write(0x6B, 0);
  timer_setup();
  interrupt_setup();
  filter_init();
  t_prev = 2.048 * timer_cnt;
}

void loop() {
 
  switch(START_STATE) {
    case 0: 
       if((PINC & (1<<6)) != 0){
        START_STATE++;
       }
       break;
    case 1: //상보필터 초기값 설정 단계
      filter_init();
      t_prev = 2.048 * timer_cnt; //4.0 곱 X 1000ms와 비교하여 2배 나눔
      delay_ms(1000);
      START_STATE++;
      break;
      
    case 2:
      PORTD |= (1<<4);
      if(R_light_flag == 1) {
        PORTD &= ~(1<<6);
        R_light_flag = 0;
      }
      Keyboard.releaseAll();
      
      //PORTD |= (1<<4); //작동 모드 돌입 시 green led on
      getData();
      calcDT();
      filter_cal();
      Mouse.move(filter_x/200, filter_z/200);

      //read joystick data
      joy[0] = adc_read(1);
      //adc_clear(); adc 재입력 오류
      //joy[1] = adc_read(2);

       if(joy[0] > 700) {
        Keyboard.press('W');
      } else if(joy[0] < 400) {
        Keyboard.press('S');
      }

      //button 한 번 더 누르면 state 0으로 복귀
      if((PINC & (1<<6) != 0)) {
        START_STATE = 0;
      }

      break;
  }
}


/* readJoystck: 조이스틱의 x, y축 adc 값을 가져온다.
 *  joy[0] = x축 data (전진 정보)
 *  joy[1] = y축 data (좌우 이동)
*/
void readJoystick() {
  joy[0] = adc_read(MUX0);
  joy[1] = adc_read(MUX1);
}

//compare with previous state
boolean btnClicked(int pin, int pin_prev) {
  int mask = 1 ;
  if(pin != 0) { pin = 1; }
  if(((pin & mask) == 1) && (pin_prev & mask) != 1){  
    return true;
  }
  else return false;
}

/* _pinMode : DDR pin setup (data enable)
 * mode flag에 따라 값을 0으로 쓸 것인지 1로 쓸 것인지 결정한다.
 * INPUT: 해당 addr, offset 0으로 setting
 * OUTPUT: 해당 addr, offset 1으로 setting
 */

void _pinMode(unsigned int addr, volatile unsigned int offset, int mode) {
   unsigned int *reg = (unsigned int *)addr;
   if(mode == INPUT) *reg &= ~offset; 
   else *reg |= offset;
}

// _read : 해당 addr의 data 값을 읽어온다.
int _read(unsigned int addr,unsigned int offset){
  unsigned int *reg = (unsigned int *)addr; //PDSR
  return (*reg & (offset));
  //if 0 -> pull down, not pushed / 2^ -> pushed
}

// _write : mode에 따라 data를 PORT register에 쓰게 된다. LOW = 0, HIGH = 1 flag
void _write(unsigned int addr, volatile unsigned int offset, int mode) {
   unsigned int *reg = (unsigned int *)addr;
  if(mode == LOW) *reg &= ~offset;
  else *reg |= offset; 
}
/*
 * i2c_init : master send init, the code from atmega32u4 datasheet at 236 page
 * SCL, SDA pin의 DDR 값을 0으로 setting하고, TWBR= 72 (100KHz at F_CPU = 16 MHZ)
 * TWI start register를 0으로 setting한다.
 */
void i2c_init() {
  _pinMode(0x2A, (1<<0), INPUT); //SCL
  _pinMode(0x2A, (1<<1), INPUT); //SDA setting 0
          
  TWBR = 72; //bit speed setting, frequency = 100kHz at CPU 16MHZ
  TWSR = 0; //TWI state register, set 0 
}
/*
 * i2c_start : send start condition
 * TWCR 레지스터의 TWINT, TWSTA, TWEN을 1로 setting하고, 
 * TWINT가 0으로 setting되면 slave로부터 ACK을 받았음을 알 수 있다.
 */
void i2c_start() {
  TWCR = (1<<TWINT) | (1 <<TWSTA) | (1<<TWEN); //send START condition
  while(!(TWCR & (1 <<TWINT))) ; //wait for TWINT flag set 0 (ACK received)
}

/*
 * i2c_send : 해당하는 addr에 read, write flag로 데이터를 전송한다.
 * Write = 0 / Read = 1이며, TWCR -> TWINT, TWEN(Send) -> TWINT 0 (ACK 수신 확인)
 */

void i2c_send(byte addr, int flag) {
  if(flag == W) {
    TWDR = (addr << 1); //address MPU_6050
  } else {
    TWDR = (addr << 1) + 1;
  }
  
  TWCR = (1 << TWINT) | (1<<TWEN);
  while(!(TWCR & (1 <<TWINT))) ; //wait for TWINT flag set 
}

/* 
 * i2c_stop : STOP
 * TWCR의 TWSTA flag 대신 TWSTO flag를 설정하고
 * STOP bit가 0으로 바뀌길 기다린다.
 */

void i2c_stop() {
  TWCR = (1<<TWINT) | (1 <<TWSTO) | (1<<TWEN); 
  while(!(TWCR & (1 <<TWSTO))) ;
}

/*
 * i2c_read : 해당하는 addr에서 data를 읽은 뒤, 읽은 데이터를 data 변수에 넣고, 반환한다.
 * <DATA READ 통신 과정>
 * START -> SLAVE ADDR+0 -> ACK -> ADDR -> RESTART -> SLAVE ADDR+ 1 -> ACK
 * -> ACK(READ DATA) -> STOP -> DELAY
*/
byte i2c_read(byte addr) {

  byte data;
 
  i2c_start(); //start
  i2c_send(gyro_addr, W); //slave addr + 0

  TWDR = addr; //slave의 접근할 register의 어드레스 값
  TWCR = (1 << TWINT) | (1<<TWEN); 
  while( !(TWCR & (1 <<TWINT)));
  if((TWSR & 0xF8) != 0x28) { //check value of twi status register 
    //Serial.println("Error at TW_MT_DATA_ACK"); //at ACK
    //0x28 = TW_MT_DATA_ACK FLAG
  }
  
  i2c_start(); //restart
  i2c_send(gyro_addr, R); //slave addr + 1 

  //receive data ack (before Ack, Ack (2))
  TWCR = (1 << TWINT) | (1<<TWEN); 
  while( !(TWCR & (1 <<TWINT))); //여기서 수신받은 정보가 TWDR에 쓰임
  
  data = TWDR; 
  i2c_stop();

  delay_us(50); //바로 데이터 읽으면 오류 발생, 이를 방지하기 위한 딜레이
  //TWI이므로 에러가 발생했을 것이라 유추

  return data;
}

/*
 * i2c_write : master write data to slave
 * <통신 과정> 
 * START -> slave addr + 0 -> ACK -> ADDR -> ACK -> data -> ACK -> STOP
 */
void i2c_write(byte addr, char data) {
  i2c_start(); //start
  i2c_send(gyro_addr, W); //addr

  //ADDR 전송 -> ACK
  TWDR = addr; 
  TWCR = (1 << TWINT) | (1<<TWEN);
  while( !(TWCR & (1 <<TWINT)));

  //data 전송 -> ACK
  TWDR = data; //data addr
  TWCR = (1 << TWINT) | (1<<TWEN);
  while( !(TWCR & (1 <<TWINT)));

  i2c_stop(); //stop
  delay_us(50);
}

/*
 * getData : i2c 통신을 통해 MPU_6050의 레지스터에 접근하여
 * 값을 읽고, 이를 정제하여 a_x, ..., g_z 변수에 저장해 놓는다.
 */
void getData(){
  byte gyro_data[14];

  for(int i = 0 ; i < 14 ; i++) {
    gyro_data[i] = i2c_read(0x3B + i);
  }

  a_x = (int)gyro_data[0]<<8|(int)gyro_data[1];
  a_y = (int)gyro_data[2]<<8|(int)gyro_data[3];
  a_z = (int)gyro_data[4]<<8|(int)gyro_data[5];
  temp = (int)gyro_data[6]<<8|(int)gyro_data[7];
  g_x = (int)gyro_data[8]<<8|(int)gyro_data[9];
  g_y = (int)gyro_data[10]<<8|(int)gyro_data[11];
  g_z = (int)gyro_data[12]<<8|(int)gyro_data[13];
}
  
/* adc_init : adc 초기화 함수
 * 해당 함수 내부에서 adc prescaler size를 조정해 변환 속도를 제어하고
 * using voltage를 정의한 뒤, ADC enable pin을 setting해준다.
 */
void adc_init() {  
    //to set ADC Prescaler 128 16MHz/128 -> 
    //따라서 ADC 변환 속도는 8MHZ로 세팅되어 있다.
    ADCSRA |= (1<<ADPS0) |(1<<ADPS1) | (1<<ADPS2)  | (1<<ADATE);
    
    _write(0x7C,(1<<6), HIGH); //setting adc voltage (using AVCC) REFS0
    _write(0x7A,(1<<7), HIGH); //ADC enable setting  
}  

/* adc_read : adc read 함수
 * ADC로 변환시킬 데이터의 offset을 받아온다. X축 :MUX0, Y축: MUX1
 * ADMUX register에서 해당 FLAG를 1로 setting하고 ADCSRA register의 
 * ADSC를 1로 setting하여 변환을 시작한다. 변환이 종료되면 ADIF -> 0
 * 아까의 MUX FLAG를 0으로 바꾸어 주고, ADCL + ADCH = ADCW를 반환한다.
 */   
uint16_t adc_read(uint8_t offset)
{  
   if(offset == 1) {
    ADMUX |= (MUX0);
   } else {
    ADMUX |= (MUX1);
   }
   ADCSRA |= (1<<ADSC) | (1<<ADEN); //adc start
   while(ADCSRA&(1<<ADIF)); //wait (after conversion, register / ADIF flag set zero)
  //   ADMUX &= ~offset; //clear select bit
   return ADCW;  //return ADC value (ADCL + ADCH)
}  


 /* interrupt_setup : 초기 인터럽트 setup
  * PCICR, PCIFR의 PCIE0, PCIF0 flag를 setting하여 interrupt 사용을 알리고
  * PCMSK0에서 7, 6, 5, 4 (PCINT num) 인터럽트에 사용한다는 것을 setting한다.
  * pin change mode 인터럽트로서 vector는 PCINT0_vect이며, 해당 line에 bit값이 변하면 실행된다.
  */
void interrupt_setup() {

  PCICR |= (1 << PCIE0);
  PCIFR |= (1 << PCIF0); //Interrupt flag is set for 
  PCMSK0 |= (1 << PCINT7) | (1<<PCINT6) | (1<<PCINT5) | (1<<PCINT4); 
  sei();
}

//PB7 PB6 PB5 PB4 순
//interrupt routine 실행되는 곳, 버튼이 눌렸을 때 적합한 일처리를 시행한다.
ISR(PCINT0_vect) { //interrupt button control
  R_light_flag = 1; 
  PORTD |= (1<<6); //R led ON
  if((PINB & (1<<4)) != 0) {
    Keyboard.print('R');
  } else if((PINB & (1<<5)) != 0) {
    Keyboard.print('V');
  } else if((PINB & (1<<6)) != 0) {
    Keyboard.print('E');
  } else if((PINB & (1<<7)) != 0) {
    Mouse.click();
  }
}
 



/*  fitler_init
 *  해당 함수는 자이로, 가속도 센서 setup 부에서 사용되는 함수로서,
 *  값을 여러 번(현재 10번) 읽고 sum을 구한 뒤, 이를 통해 도출된 평균값을
 *  init_data에 저장한다. initial data 값과 변경된 상태 데이터 값의 
 *  차이를 통해 mouse의 변화량을 제어하게 된다.
 */
void filter_init() {
  double sum[6]; //sum을 담아 놓을 변수
  for(int i = 0; i < 10; i++) {
    getData(); //MPU_6050으로부터 data read
    sum[0] += (double) a_x; 
    sum[1] += (double) a_y;
    sum[2] += (double) a_z;
    sum[3] += (double) g_x;
    sum[4] += (double) g_y;
    sum[5] += (double) g_z; 
  }
  for(int i = 0; i < 6; i++) {
    init_data[i] = sum[i] / (float)10;
  }
}

/* filter_axis
 * 마우스를 제어하기 위해 x, z 부분의 angle이 필요하다. 
 * 해당 함수는 axis를 이용해 각을 구하는 것이다.
 * angel(x) = 중력 가속도의 영향을 받아 해당 부분의 angle은 구할 수 없다. 
 * angle(z) = arctan(AcY / sqrt(AcX*AcX + AcZ*AcZ)) * 180 / PI 
*/
void filter_axis() {
  float AcX, AcY, AcZ;

  //now data에서 init data의 차를 바탕으로 계산한다.
  AcX = (float) a_x - init_data[0] + 16384; //중력가속도 제외
  AcY = (float) a_y - init_data[1];
  AcZ = (float) a_z - init_data[2];

  aAngle_z = atan(AcY / sqrt(pow(AcX,2)+pow(AcZ,2))) * (180/PI);
  aAngle_x = atan(sqrt(pow(AcY,2)+pow(AcZ,2))/ AcX) * (180/PI);
}


void filter_gyro() {
  gX = (float) g_x - init_data[3] / 131;
  gZ = (float) g_z - init_data[5] / 131; //about MPU6050 datasheet 1sec

  gAngle_x += gX * dt; //적분값
  gAngle_z += gZ * dt;
}

//센서 계산값
void filter_cal() {
  filter_axis();
  filter_gyro();
  
  float prev_angle_x, prev_angle_z;

  prev_angle_x = filter_x + gX * dt;
  prev_angle_z = filter_z + gZ * dt;

  filter_x = 0.96 * prev_angle_x + 0.04 * aAngle_x; //alpha 0.96
  filter_z = 0.96 * prev_angle_z + 0.04 * aAngle_z; 

}

void calcDT() {
  t_now = 2.048 * timer_cnt;
  dt = (t_now - t_prev) / 1000.00;
  t_prev = t_now;
}



 // for 16MHz
void delay_us(unsigned int time_us)    // Time delay for 1us
{           
   unsigned int i;
   for(i=0; i<time_us*2; i++) //4
   {
      asm volatile(" PUSH R0 "); //2
      asm volatile(" POP R0 "); //2 = 8*2 = 16 cycles
   }                                  
}

void delay_ms(unsigned int time_ms)
{
   unsigned int i;
   for(i=0; i<time_ms; i++) 
   {
      delay_us(1000);
   }
}

void timer_setup() {
  TCCR1B |= (1<<CS10); //분주비
  TIMSK1 |= (1<<TOIE1); //timer가 overflow가 일어나면 interrupt를 거는 bit set
  sei();
}

ISR(TIMER1_OVF_vect){
   timer_cnt++;
}

uint64_t _millis() {
  return timer_cnt;
}
