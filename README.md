# 🧠 RV32I Multi-Cycle MCU with AMBA/APB Peripherals

**Author:** 이동관  
**Language:** SystemVerilog  
**Tool:** Vivado / QuestaSim / Verilator  
**Board:** (예시) Basys3 (100 MHz)

---

## ✨ Overview

Top-level MCU integrating a **multi-cycle RV32I CPU**, an **AMBA APB bus**, and multiple **peripherals** (RAM, GPO, GPI, GPIO, UART).  
Includes **self-checking UART loopback testbenches** and **firmware examples**.

---

## ⚙️ Key Features

- **CPU**: RV32I 멀티사이클 코어  
  - FSM: `FETCH → DECODE → R_EXE → I_EXE → B_EXE → LU_EXE → AU_EXE → J_EXE → JL_EXE → S_EXE → S_MEM → L_EXE → L_MEM → L_WB`
  - 32 register file, immediate extension, 5-way RFWD mux, JAL/JALR/branch logic  
  - x0 hardwired to zero

- **AMBA APB Bus**
  - Modules: `APB_Master`, `APB_Decoder`, `APB_Mux`
  - Automatic **word→byte address** translation (`{addr[31:2], 2'b00}`)

- **Peripherals**
  - **RAM**: APB slave (1-cycle ready), word-indexed
  - **GPO/GPI/GPIO**: CR/ODR/IDR registers, tri-state implementation
  - **UART**: 16× oversampling, RX/TX FIFOs, `rx_done` pulse, `tx_overrun` sticky flags,  
    and a **debug byte output** (`tx_byte_dbg`) for LED display

- **MCU Top Demo**
  - Displays the transmitted UART byte on LEDs (`gpo <= uart_tx_byte_dbg`)
  - ROM initialized from `code.mem`

---

## 📂 Repository Structure

rtl/
├─ MCU.sv # Top: CPU + APB + peripherals
├─ CPU_RV32I.sv, ControlUnit.sv, DataPath.sv
├─ ROM.sv, RAM.sv
├─ APB_Master.sv, APB_Decoder.sv, APB_Mux.sv
├─ GPO_Periph.sv, GPI_periph.sv, GPIO_Periph.sv
├─ UART_Periph.sv (includes FIFO, baud_gen, RX/TX)
├─ fifo.sv, register_file.sv, fifo_ctrl_unit.sv
└─ defines.sv
sim/
├─ tb_UART_Periph.sv # classic task-based TB
└─ tb_UART_Periph_onefile.sv # class+interface unified TB
fw/
└─ uart_min.c # simple UART firmware example
constraints/
└─ basys3.xdc # FPGA I/O constraints
code.mem # ROM initialization

ruby
코드 복사

---

## 🗺 Memory Map

| Region | Address Range | Device |
|-------:|----------------|--------|
| 0x0xxx | 0x1000_0000 – 0x1000_0FFF | RAM |
| 0x1xxx | 0x1000_1000 – 0x1000_1FFF | GPO |
| 0x2xxx | 0x1000_2000 – 0x1000_2FFF | GPI |
| 0x3xxx | 0x1000_3000 – 0x1000_3FFF | GPIO |
| 0x4xxx | 0x1000_4000 – 0x1000_4FFF | UART |

> APB Master outputs **byte addresses**.  
> CPU internally uses **word addresses**, converted by `{addr[31:2], 2'b00}`.

---

## 📑 Register Maps

### 📨 UART (`UART_BASE = 0x1000_4000`)

| Offset | Name | Access | Description |
|-------:|------|:------:|-------------|
| 0x00 | **CR** | R/W | Control (`CR[0]=uart_en`, default = 1) |
| 0x04 | **SR** | R/W1C | `{…, tx_overrun[4], rx_done[3], tx_busy[2], tx_full[1], rx_empty[0]}` |
| 0x08 | **TXD** | W | TX FIFO Write |
| 0x0C | **RXD** | R | RX FIFO Read |

> `tx_byte_dbg` = latched byte when transmission starts → used for LED debug.

---

### 💡 GPO (`0x1000_1000`)

| Offset | Name | Access | Description |
|-------:|------|:------:|-------------|
| 0x00 | **CR** | R/W | Bit 1 = Output Enable |
| 0x04 | **ODR** | R/W | Output Data Register |

---

### 🎛 GPI (`0x1000_2000`)

| Offset | Name | Access | Description |
|-------:|------|:------:|-------------|
| 0x00 | **CR** | R/W | Bit 1 = Input Enable |
| 0x04 | **IDR** | R | Input Data Register |

---

### 🔌 GPIO (`0x1000_3000`)

| Offset | Name | Access | Description |
|-------:|------|:------:|-------------|
| 0x00 | **CR** | R/W | 1 = Output, 0 = Input |
| 0x04 | **ODR** | R/W | Output Data |
| 0x08 | **IDR** | R | Input Data |

---

## 🧪 Simulation Guide

1. Set `tb_UART_Periph.sv` (or `tb_UART_Periph_onefile.sv`) as **Top**.  
2. Accelerate UART by defparam:

```verilog
defparam U_DUT.U_CORE.U_BAUD_TICK.CLK_HZ = 50_000_000;
defparam U_DUT.U_CORE.U_BAUD_TICK.BAUD   = 1_000_000;
defparam U_DUT.U_CORE.U_BAUD_TICK.OS     = 16;
Run simulation → prints PASS/FAIL summary for 50 loopback bytes.

🧰 Firmware Example (C)
c
코드 복사
#include <stdint.h>
#define APB_BASE_ADDR  0x10000000u
#define UART_BASE      (APB_BASE_ADDR + 0x00004000u)
#define UART_CR        (*(volatile uint32_t*)(UART_BASE + 0x00))
#define UART_SR        (*(volatile uint32_t*)(UART_BASE + 0x04))
#define UART_TXD       (*(volatile uint32_t*)(UART_BASE + 0x08))
// SR bit1 = tx_full

static inline void delay(volatile uint32_t n){ while(n--){} }

int main(void){
    UART_CR = 1u;            // enable UART
    delay(50000);

    for(;;){
        while (UART_SR & (1u<<1)) { /* wait if tx_full */ }
        UART_TXD = 'U';
        delay(200000);
    }
}
If nothing appears in the terminal:
① Check UART_CR=1 ② Confirm terminal CR/LF settings
③ Verify BAUD/STOPS ④ Ensure TX FIFO not full

🧩 Testbenches
File	Description
tb_UART_Periph.sv	Simple procedural testbench (task-based)
tb_UART_Periph_onefile.sv	Unified interface + class version (UVM-lite style)
Both	Loopback 50 bytes → Self-checking PASS/FAIL summary

🧠 Design Notes
CPU outputs word address, APB Master converts to byte address

RAM: PADDR[11:2] → word indexing

UART RX: 16× oversampling, half-bit align, mid-bit sampling

RX rx_done = 1-cycle pulse → mirrored/sticky in SR

tx_byte_dbg latched at tx_start → stable LED display

🧯 Troubleshooting
Issue	Cause / Fix
UART not transmitting	Check CR[0]=1, BAUD mismatch, or FIFO full
LEDs not blinking	assign gpo = uart_tx_byte_dbg; or invert for Active-Low
Simulation OK, board mismatch	Adjust terminal newline (CR/LF) & initial UART enable
TX overrun flag	Writing TXD while full → clear via SR read or W1C

🚀 Future Work
Add RV32M (mul/div) extension

Pipeline CPU with hazard detection & forwarding

Add cache / DMA / configurable UART BAUD

APB interrupt support

📜 License
MIT License (free for modification & educational use)

🙌 Acknowledgements
Design & verification by Lee DongGwan
Includes debugging logs, waveform captures, and board bring-up notes.

