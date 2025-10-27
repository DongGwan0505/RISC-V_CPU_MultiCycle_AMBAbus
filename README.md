RV32I Multi-Cycle MCU with AMBA/APB Peripherals (UART/GPIO/GPO/GPI/RAM)

Top-level MCU integrating a multi-cycle RV32I CPU, an APB bus (Master/Decoder/Mux), and peripherals (RAM, GPO, GPI, GPIO, UART). Includes self-checking UART loopback testbenches and firmware snippets.

Board: (예시) Basys3 100 MHz
Tool: Vivado / ModelSim(Questa) / Verilator (시뮬)
RTL: SystemVerilog

✨ Features

CPU: RV32I 멀티사이클 코어
– FSM: FETCH→DECODE→R_EXE→I_EXE→B_EXE→LU_EXE→AU_EXE→J_EXE→JL_EXE→S_EXE→S_MEM→L_EXE→L_MEM→L_WB
– RegFile x32, immExtend, 5-way RFWD Mux, 분기/JAL/JALR, 하드와이어드 x0

AMBA/APB:
– APB_Master(IDLE/SETUP/ACCESS), APB_Decoder, APB_Mux
– CPU의 워드 주소→바이트 주소 변환(하위 2비트 00) 내장

Peripherals:
– RAM: APB slave, 1-cycle ready (word addressing PADDR[11:2])
– GPO/GPI/GPIO: 간단한 CR/ODR/IDR 인터페이스, tri-state 포함
– UART: 16× oversampling, RX/TX FIFO(8-entry), rx_done 1-클록 펄스, tx_overrun sticky, 디버그 바이트(tx_byte_dbg)

MCU Top 데모:
– uart_tx_byte_dbg를 **외부 LED(gpo)**로 바로 표시 (프레임 시작에 래치)
– ROM $readmemh("code.mem")

📁 Repository Layout
rtl/
 ├─ MCU.sv                 # Top (ROM, CPU_RV32I, APB Master, RAM, GPO/GPI/GPIO/UART)
 ├─ CPU_RV32I.sv           # ControlUnit + DataPath
 ├─ ControlUnit.sv
 ├─ DataPath.sv
 ├─ ROM.sv
 ├─ APB_Master.sv
 ├─ APB_Decoder.sv
 ├─ APB_Mux.sv
 ├─ RAM.sv
 ├─ GPO_Periph.sv, GPI_periph.sv, GPIO_Periph.sv (+ APB_SlaveIntf_*)
 ├─ UART_Periph.sv         # APB_SlaveIntf_UART + uart_apb_core + FIFOs + baud/UART
 ├─ fifo.sv, register_file.sv, fifo_ctrl_unit.sv
 └─ common (defines.sv, 작은 유틸)
sim/
 ├─ tb_UART_Periph.sv            # 순수 TB (함수/태스크형)
 └─ tb_UART_Periph_onefile.sv    # interface+class all-in-one TB
fw/
 └─ examples/uart_min.c          # 간단한 펌웨어 예제
constraints/
 └─ basys3.xdc                   # 100 MHz clock 외 I/O (보드 사용 시)
code.mem                         # ROM 초기화 파일


파일 명은 실제 레포 구조에 맞게 조정하세요.

🗺 Memory Map (APB)

상위 12비트 0x100 고정 (0x1000_0000 대역). 디코더는 바이트 주소 기준.

Region	Address Range	Device
0x0xxx	0x1000_0000–0x1000_0FFF	RAM
0x1xxx	0x1000_1000–0x1000_1FFF	GPO
0x2xxx	0x1000_2000–0x1000_2FFF	GPI
0x3xxx	0x1000_3000–0x1000_3FFF	GPIO
0x4xxx	0x1000_4000–0x1000_4FFF	UART

APB 마스터 내부에서 CPU의 워드 주소 → 바이트 주소로 변환:
addr_byte = {a_reg[31:2], 2'b00}

🧾 Register Maps
UART (UART_BASE = 0x1000_4000)
Offset	Name	Access	Description
0x00	CR	R/W	CR[0]=uart_en (reset=1)
0x04	SR	R/W1C	{…, tx_overrun_sticky[4], rx_done_sticky[3], tx_busy[2], tx_full[1], rx_empty[0]}
읽으면 sticky 클리어
0x08	TXD	W	TX FIFO write (LSB 8b 유효)
0x0C	RXD	R	RX FIFO read (LSB 8b 유효)

디버깅: uart_apb_core.tx_byte_dbg는 이번 프레임에 실제 송신 시작한 바이트를 래치 → MCU에서 gpo로 매핑하여 LED로 확인.

GPO (0x1000_1000)
Offset	Name	Access	Description
0x00	CR	R/W	각 비트 1=출력 활성
0x04	ODR	R/W	출력 데이터
GPI (0x1000_2000)
Offset	Name	Access	Description
0x00	CR	R/W	각 비트 1=입력 허용
0x04	IDR	R	입력 데이터 (읽을 때 라치된 값 반환)
GPIO (0x1000_3000)
Offset	Name	Access	Description
0x00	CR	R/W	1=출력, 0=입력
0x04	ODR	R/W	출력 값
0x08	IDR	R	입력 값
🔧 Build & Run
Simulation (Vivado/Questa)

sim/tb_UART_Periph.sv 또는 sim/tb_UART_Periph_onefile.sv를 Top으로 설정

빠른 시뮬을 위해 UART 보오레이트를 defparam으로 조정:

defparam U_DUT.U_CORE.U_BAUD_TICK.CLK_HZ = 50_000_000;
defparam U_DUT.U_CORE.U_BAUD_TICK.BAUD   = 1_000_000;
defparam U_DUT.U_CORE.U_BAUD_TICK.OS     = 16;


Run. TB는 랜덤 바이트를 loopback으로 보내 PASS/FAIL 요약을 출력합니다.

Synthesis on FPGA

rtl/MCU.sv를 Top으로 설정하고 constraints/basys3.xdc 적용

보드 핀 매핑: clk=100 MHz, rx/tx, LED/SW에 맞게 업데이트

code.mem을 프로젝트에 포함 (기본 ROM 프로그램)

🧪 Firmware Mini Example
#include <stdint.h>
#define APB_BASE_ADDR  0x10000000u
#define UART_BASE      (APB_BASE_ADDR + 0x00004000u)
#define UART_CR        (*(volatile uint32_t*)(UART_BASE + 0x00))
#define UART_SR        (*(volatile uint32_t*)(UART_BASE + 0x04))
#define UART_TXD       (*(volatile uint32_t*)(UART_BASE + 0x08))
// SR bit1 = tx_full

static inline void delay(volatile uint32_t n){ while(n--){} }

int main(void){
    UART_CR = 1u;            // enable (reset default도 1)
    delay(50000);

    for(;;){
        while (UART_SR & (1u<<1)) { /* tx_full==1이면 대기 */ }
        UART_TXD = 'U';      // 0x55가 LED에 깜빡이며 표시됨 (tx_byte_dbg)
        delay(200000);
    }
}


보드에서 ‘U’가 터미널에 찍히지 않으면:
① CR=1 확인 ② 터미널의 CR/LF 옵션 정리 ③ 보오레이트/STOP_BITS 일치 ④ TX FIFO tx_full 대기 로직 확인

🧰 Testbenches

tb_UART_Periph.sv: 태스크 기반, APB 1-사이클 ACCESS 보장, loopback로 50바이트 송수신/검증

tb_UART_Periph_onefile.sv: interface + class 패키지 한 파일, 동일 시나리오

공통 유틸: purge_rx_fifo, wait_rx_empty/nonempty, SR helper 함수

🧠 Design Notes

주소 정렬: CPU는 워드 주소로 생각하고, APB 마스터가 바이트 주소로 변환하여 슬레이브들에 전달

RAM: PADDR[11:2]로 워드 인덱싱 (1-cycle ready)

UART RX: 16× 오버샘플, START 중간에서 정렬, 각 비트 말미(==15) 샘플 → 중앙 샘플링 보장

rx_done: 1-클록 펄스. APB 인터페이스에서 sticky status로 미러/클리어 (read 또는 W1C)

tx_byte_dbg: w_tx_start 시점의 바이트를 래치 → 프레임 전체 동안 안정적으로 LED에 노출

🧩 Known Issues / Troubleshooting

보드/시뮬 차이

터미널 개행 옵션(CR/LF) 차이로 에코/LED 패턴 상이 → 펌웨어에서 개행 처리 고정

UART_CR=1 초기화 필수 (reset 기본 1이지만 명시 추천)

TX가 안 나감

tx_full 상태에서 쓰면 tx_overrun_sticky 세트 → SR 읽기 또는 W1C로 클리어

보오레이트/STOP_BITS, 클럭 정확도 확인

LED 미표시

MCU에서 최종 assign gpo = uart_tx_byte_dbg; 확인 (LED Active-Low이면 ~ 필요)

🚧 Roadmap

RV32M(곱셈/나눗셈) 확장

데이터/제어 hazard 대응 파이프라인 코어 (forwarding/HDU)

캐시 또는 TCM, DMA

UART 파라미터 런타임 설정 (BAUD, STOP_BITS) + 프레이밍 에러 검출

📜 License

MIT (원하면 다른 라이선스로 변경)

🙌 Acknowledgements

테스트/디버깅 로그 및 파형 분석, 보드 bring-up 이슈 정리: @LeeDongGwan

빠른 시작 체크리스트

 code.mem 포함되었는지

 보드 핀/XDC 최신화

 터미널 BAUD/개행 옵션 일치

 UART_CR=1 설정

 시뮬에선 baud defparam으로 가속
