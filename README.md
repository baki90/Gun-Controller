# Gun-Controller
- Embedded System, MCU Atmega32u4

# FPS 게임 연동 컨트롤러
- 컨트롤러를 통한 PC 제어가 이루어진다.
  1. 가속도, 자이로 센서 상보 필터링을 통해 PC의 마우스 제어
  2. 조이스틱, 버튼을 이용하여 키보드 입력 제어

  <img src="https://user-images.githubusercontent.com/30331087/85937594-0455d180-b940-11ea-866b-a10554be8027.gif" width="50%" height="50%">

## Block Diagram
  <img src="https://user-images.githubusercontent.com/30331087/85937394-fe5ef100-b93d-11ea-9a01-c0ffec915f4f.png" width="80%" height="80%">

## Schematic
  <img src="https://user-images.githubusercontent.com/30331087/85937408-323a1680-b93e-11ea-91b5-aeff994bcc1c.png" width="80%" height="80%">
  - button을 RC회로로 구성하여 채터링을 방지함
  - MPU6050 PULL-UP

## Circuit
  <img src="https://user-images.githubusercontent.com/30331087/85937531-5518fa80-b93f-11ea-9576-c1d37a74b84d.png" width="80%" height="80%">

## Code

### TWI
 - MPU 6050과 TWI로 통신하여 Raw Data를 받아오고, 이를 상보 필터로 정제한 후 마우스 움직임에 사용했다.
 - SCL 100kHZ

### 상보 필터
 ![image](https://user-images.githubusercontent.com/30331087/85937431-7c22fc80-b93e-11ea-99c7-6531280d2467.png)
 - 초기 filter-init으로 10번 읽어들인 후 평균값을 이록한다. 이후 Axis 값을 삼각비 법칙을 통해 x와 z의 angle 값으로 구한다.
 - 구분구적법을 통해 Gyro 각도값을 추출한다.
 - 시간이 지남에 따라 Gyro의 drift가 발생하는데, 이를 방지하기 위해 Axis의 angle과 상보 필터링을 진행한다.

### ADC
- Division 64 -> 16/64 = 250kHZ (110)
- 5V의 입력 전압을 사용하여 10bit 값을 나타낸다.

### INTERRUPT
 1. Timer Interrupt
   - Timer 1을 사용하여 timer 값을 카운팅한다. 분주를 사용하지 않아 1/16MHZ*256*64 = 2.048ms마다 overflow가 발생한다. 따라서 counting하는 값에 2.048을 곱해 주면 ms를 취득할 수 있다.
 2. Pin Change Interrupt
   - 같은 라인의 pin state의 변화에 따라서 interrupt가 발생한다.
