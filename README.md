# FPGA 보드를 활용한 CNN 연산 가속기 설계
FPGA 보드에서 특정 행렬을 연산하는 연산기를 설계하고, 이를 검증하는 프로젝트
HPS-FPGA 시스템을 이용했다.

## 개발 보드
DE1-SoC 보드를 사용함.

## 구현 방법
Quartus에서 제공하는 platform designer인 QSYS를 사용해서 Intel의 Avalon bus system을 사용했다.
### Overall Architecture
<img width="996" alt="스크린샷 2024-11-13 오전 9 18 59" src="https://github.com/user-attachments/assets/1395b1cd-8bfb-4241-b3d6-151bf4737642">

### HPS의 검증 코드 동작 원리: C언어 사용
On-chip SRAM 0의 0번지 주소를 통해 HPS와 FPGA가 hand-shanking(0 이면 IDLE, 1이면 Working)

1. On-chip SRAM 0의 0번째 하위 32비트 값이 0이면 IDLE -> 동작 안함
2. Write (SW)
    + HPS는 on-chip SRAM 0의 4-15번지에 weight 값을 쓴다.(weight_ptr)
    + HPS는 on-chip SRAM 1의 0-11번지에 fmapA 값을 쓴다. (fmap_ptr)
    + HPS는 on-chip SRAM 2의 0-11번지에 fmapB 값을 쓴다. (fmap_ptr)
3. Write (HW)
   + HW는 on-chip SRAM1의 12-15번지에 mac result a를 쓴다.
   + HW는 on-chip SRAM2의 12-15번지에 mac result b를 쓴다.
4. HW는 On-chip SRAM 0의 0번지 하위 32비트의 1값을 다시 0으로 쓴다.
5. HPS는 지속적으로 on-chip SRAM 0의 0번지를 읽으며, 해당 값이 0으로 바뀌면 matrix operation이 끝난 것을 인지한다.

### Module Hierarchy
<img width="851" alt="스크린샷 2024-11-13 오전 9 25 54" src="https://github.com/user-attachments/assets/a3d5f4c6-e95f-4c5c-9554-eadc058b55da">

### 각 모듈별 설명
#### DE1_SoC_Computer
IP Generation을 통한 computer_system & FPGA 연결해주는 Top Module
#### mat_ops.v
+ 기존의 M10K_read_write.v 모듈을 확장
+ M10K_read_write.v에서 READ-WRITE state 사이에 CNN_RUN state를 추가했다 (다른 연산으로 변경 가능)
+ 이전에 작성한 cnn_topCore 모듈 인스턴스 추가 (+관련 start, done 신호 추가)
+ 3개의 M10K_read_buffer 존재 (SRAM0, SRAM1, SRAM2)
+ 2개의 M10K_write 존재 (SRAM1, SRAM2)
+ M10K_read_buffer에서 받아온 데이터를 다시 setup 해주어야 한다.
+ cnn_topCore 에서 나온 data도 마찬가지로 setup 해서 M10K_write에서 write 해줘야 한다.
#### defines_computer.vh
프로젝트의 변수들을 저장하고 있다. (const처럼 사용하기 위함)

## Challenge Point
DE1-SoC 보드의 자원한계로 기존에 `Generate` 구문으로 작성했던 것을 FSM으로 바꿔 작성해야 했다.
또한, 이에 맞춰 control signal을 다시 입력하는 문제, 타이밍 불일치 문제를 해결해야 했고, 이에 상당한 시간이 소요되었다.

## Result
### 자원 소요
#### DSP Blocks
<img width="364" alt="image" src="https://github.com/user-attachments/assets/b61d8af9-dffd-43ef-b236-228537afbd9e">

#### M10K Blocks
<img width="377" alt="image" src="https://github.com/user-attachments/assets/dcc622a8-588f-4cbf-a044-16cc9c7f131f">

#### LABs
<img width="385" alt="image" src="https://github.com/user-attachments/assets/9e267979-5512-49b8-a15a-ad2ecd399467">

### 시뮬레이션 결과
<img width="207" alt="image" src="https://github.com/user-attachments/assets/fcb1e8ca-87f3-4d9c-9268-5a68c71eb140">

연산 결과가 일치한 모습을 볼 수 있다.

### SoC 보드에 Linux로 검증 프로그램으로 검증한 결과
<img width="451" alt="image" src="https://github.com/user-attachments/assets/1c40c2dd-012f-4bf0-853c-944db2cac8c0">

Cnn_core 시뮬레이션에서는 문제가 없었지만, FPGA 보드에 linux로 실행할 때 cnn_core에서 잘된 것과 대비되게 잘 실행되지 않았고 원인을 몇가지 분석했다.
1. Timing 문제
    + Cnn_core에서도 타이밍 때문에 고생했지만, mat_ops.v에서 역시 cnn_core를 instance로 사용하기 때문에 값이 오가는 타이밍을 잘 맞추지 못해 연산 결과가 다르게 나왔을 수도 있다.
2. 비트 분산 문제
    + Linux의 cnn을 수행하는 코드가 c로 짜여진 탓에 실제 메모리 공간을 활용할 때 데이터 사이에 빈 공간이 생겼다. 이를 벡터로 가져와 전처리, 후처리를 진행할 때 LSB, MSB를 헷갈려 틀렸을 수도 있다.

## 총평
Generate 구문을 활용하는 것이 병렬 연산이 가능하기 때문에 Verilog 코드를 작성할 때 편했다.

하지만, M10K의 자원적 제약 때문에 각 모듈을 pipelining을 진행했을 때, 이론적으로는 완벽했지만, 각 모듈들의 timing을 맞추는 문제가 제일 어려웠다.

특히, cnn_core에서는 in_valid 신호가 generate일 때는 한 번만 넣어주면 되었기에 pipeline으로 바꿀 때 각 스테이지마다 새로운 in_valid 신호를 만들고, 이를 타이밍 맞게 넣는 것이 쉽지 않았다.

이번 프로젝트를 통해 자원 소모량과 소모 시간 간에 trade off가 있음을 느낄 수 있었고 실제 FPGA와 HPS 통신이 쉽지 않은 일임을 깨달을 수 있었다.
