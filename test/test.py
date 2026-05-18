# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start test for PVEH Optimizer ASIC")

    # クロックを 32.768 kHz (約30.5us周期) に設定
    clock = Clock(dut.clk, 30.5, unit="us")
    cocotb.start_soon(clock.start())

    # 入力の初期化
    dut.ena.value = 1
    dut.ui_in.value = 0     # ext_trigger = 0, cfg = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    # リセットシーケンス
    dut._log.info("Applying Reset")
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # -----------------------------------------------------
    # テスト 1: ST_IDLE (待機状態) の出力チェック
    # -----------------------------------------------------
    dut._log.info("Checking ST_IDLE state")
    # 期待される uo_out (8ビット) の内訳:
    # bit 0: spi_cs_n       = 1 (Active Low なので High待機)
    # bit 1: spi_sck        = 0
    # bit 2: uart_tx_pin    = 1 (UARTのアイドル状態は High)
    # bit 3: pulse_parallel = 0
    # bit 4: pulse_series   = 0
    # bit 7:5: led          = 111 (ST_IDLE(0)の反転)
    # -> 2進数: 1110 0101 = 0xE5 (229)
    assert int(dut.uo_out.value) == 0xE5, f"Expected 0xE5, but got {hex(dut.uo_out.value)}"

    # -----------------------------------------------------
    # テスト 2: トリガ入力による状態遷移のチェック
    # -----------------------------------------------------
    dut._log.info("Triggering measurement sequence")
    # ext_trigger ピン(ui_in[0]) を High にしてトリガをかける
    dut.ui_in.value = 1 
    await ClockCycles(dut.clk, 2) # エッジ検出を待つ
    dut.ui_in.value = 0           # トリガを戻す
    await ClockCycles(dut.clk, 1)

    # 期待される uo_out (8ビット) の内訳:
    # ST_PULSE_INIT (状態=1) に遷移し、Parallel側のパルスが出る
    # bit 0: spi_cs_n       = 1
    # bit 1: spi_sck        = 0
    # bit 2: uart_tx_pin    = 1 
    # bit 3: pulse_parallel = 1 (出力ON)
    # bit 4: pulse_series   = 0
    # bit 7:5: led          = 110 (ST_PULSE_INIT(1)の反転)
    # -> 2進数: 1100 1101 = 0xCD (205)
    dut._log.info("Checking ST_PULSE_INIT state")
    assert int(dut.uo_out.value) == 0xCD, f"Expected 0xCD, but got {hex(dut.uo_out.value)}"

    dut._log.info("All tests passed! Ready for Tapeout!")
