/*
 * Copyright (c) 2024 Takaharu Yamada
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// =======================================================================
// [トップモジュール] Tiny Tapeout 向けラッパー
// =======================================================================
module tt_um_nezumi_tech_adc_sq_compare (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire       ext_trigger = ui_in[0];
    wire       spi_sdo     = ui_in[1];
    wire [2:0] cfg_stable  = ui_in[4:2];
    wire [2:0] cfg_charge  = ui_in[7:5];

    wire       spi_cs_n;
    wire       spi_sck;
    wire       uart_tx_pin;
    wire       pulse_parallel;
    wire       pulse_series;
    wire [2:0] led;

    assign uo_out[0]   = spi_cs_n;
    assign uo_out[1]   = spi_sck;
    assign uo_out[2]   = uart_tx_pin;
    assign uo_out[3]   = pulse_parallel;
    assign uo_out[4]   = pulse_series;
    assign uo_out[7:5] = led;

    assign uio_oe  = 8'b0000_0000;
    assign uio_out = 8'b0000_0000;

    wire _unused = &{ena, uio_in, 1'b0};

    pveh_optimizer_core u_core (
        .clk(clk),
        .rst_n(rst_n),
        .ext_trigger(ext_trigger),
        .spi_sdo(spi_sdo),
        .cfg_stable(cfg_stable),
        .cfg_charge(cfg_charge),
        .pulse_parallel(pulse_parallel),
        .pulse_series(pulse_series),
        .spi_cs_n(spi_cs_n),
        .spi_sck(spi_sck),
        .uart_tx_pin(uart_tx_pin),
        .led(led)
    );

endmodule


// =======================================================================
// [サブモジュール 1] 環境発電最適化 コアロジック (面積超・最適化版)
// =======================================================================
module pveh_optimizer_core (
    input  wire clk,
    input  wire rst_n,
    input  wire ext_trigger,
    input  wire spi_sdo,
    input  wire [2:0] cfg_stable,
    input  wire [2:0] cfg_charge,
    output reg  pulse_parallel,
    output reg  pulse_series,
    output wire spi_cs_n,
    output wire spi_sck,
    output wire uart_tx_pin,
    output wire [2:0] led
);

    parameter CLK_FREQ = 32_768; 
    localparam COUNT_5MS = 164; 
    localparam [22:0] COUNT_1S = CLK_FREQ;
    
    wire [22:0] wait_stable_max = COUNT_1S << cfg_stable;
    wire [22:0] wait_charge_max = COUNT_1S << cfg_charge;

    // ステート定義
    reg [4:0] state;
    localparam ST_IDLE          = 5'd0;
    localparam ST_PLS_PAR_INIT  = 5'd1;
    localparam ST_WAIT_STAB_P   = 5'd2;
    localparam ST_READ_A        = 5'd3;
    localparam ST_WAIT_CHG_P    = 5'd4;
    localparam ST_READ_B        = 5'd5;
    localparam ST_WAIT_REC      = 5'd6;
    localparam ST_PLS_SER_INIT  = 5'd7;
    localparam ST_WAIT_STAB_S   = 5'd8;
    localparam ST_READ_C        = 5'd9;
    localparam ST_WAIT_CHG_S    = 5'd10;
    localparam ST_READ_D        = 5'd11;
    
    // 省面積・乗算器用ステート
    localparam ST_CALC_PREP_P   = 5'd12;
    localparam ST_CALC_MULT_P   = 5'd13;
    localparam ST_CALC_PREP_S   = 5'd14;
    localparam ST_CALC_MULT_S   = 5'd15;
    
    localparam ST_CALC_CMP      = 5'd16;
    localparam ST_PLS_WINNER    = 5'd17;
    localparam ST_TX_SEND       = 5'd18;
    localparam ST_TX_WAIT       = 5'd19;

    assign led = ~state[2:0]; 
    reg [22:0] timer; 
    
    reg        spi_start;
    wire       spi_busy, spi_done;
    wire [15:0] spi_data;
    
    ltc2450_spi_read_sync_32k u_spi (
        .clk(clk), .rst_n(rst_n), .start(spi_start),
        .busy(spi_busy), .done(spi_done), .data_out(spi_data),
        .spi_cs_n(spi_cs_n), .spi_sck(spi_sck), .spi_sdo(spi_sdo)
    );

    // データレジスタ
    reg [15:0] val_A, val_B, val_C, val_D;
    
    // 【面積削減】順次乗算(Shift & Add)用の共通レジスタ
    // 計算式: (B-A) * (B+A) を計算する
    reg [33:0] diff_P, diff_S;
    reg [16:0] mult_a;
    reg [33:0] mult_b;
    reg [33:0] mult_acc;
    reg [4:0]  mult_cnt;
    
    reg [7:0]  cmp_char;

    // UART制御用
    reg         uart_start;
    reg [7:0]   uart_data;
    wire        uart_busy;
    reg [5:0]   tx_idx; // 最大22なので6ビットに縮小

    uart_tx_32k u_uart (
        .clk(clk), .rst_n(rst_n), .tx_start(uart_start),
        .tx_data(uart_data), .tx_busy(uart_busy), .uart_txd(uart_tx_pin)
    );

    function [7:0] hex2ascii(input [3:0] hex);
        begin
            if (hex < 4'd10) hex2ascii = 8'h30 + hex;
            else             hex2ascii = 8'h37 + hex;
        end
    endfunction

    reg trig_d1, trig_d2;
    wire trig_pulse = (trig_d1 && !trig_d2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {trig_d1, trig_d2} <= 2'b00;
        else        {trig_d1, trig_d2} <= {ext_trigger, trig_d1};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            timer <= 23'd0;
            pulse_parallel <= 1'b0;
            pulse_series   <= 1'b0;
            spi_start <= 1'b0;
            uart_start <= 1'b0;
            tx_idx <= 6'd0;
            {val_A, val_B, val_C, val_D} <= 64'd0;
            diff_P <= 34'd0; diff_S <= 34'd0;
            mult_a <= 17'd0; mult_b <= 34'd0; mult_acc <= 34'd0; mult_cnt <= 5'd0;
            cmp_char <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    pulse_parallel <= 1'b0; pulse_series <= 1'b0;
                    if (trig_pulse) begin timer <= 23'd0; state <= ST_PLS_PAR_INIT; end
                end

                ST_PLS_PAR_INIT: begin
                    pulse_parallel <= 1'b1;
                    if (timer >= COUNT_5MS - 1) begin pulse_parallel <= 1'b0; timer <= 23'd0; state <= ST_WAIT_STAB_P; end
                    else timer <= timer + 1'b1;
                end
                ST_WAIT_STAB_P: begin
                    if (timer >= wait_stable_max - 1) begin spi_start <= 1'b1; state <= ST_READ_A; end
                    else timer <= timer + 1'b1;
                end
                ST_READ_A: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_A <= spi_data; timer <= 23'd0; state <= ST_WAIT_CHG_P; end
                end
                ST_WAIT_CHG_P: begin
                    if (timer >= wait_charge_max - 1) begin spi_start <= 1'b1; state <= ST_READ_B; end
                    else timer <= timer + 1'b1;
                end
                ST_READ_B: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_B <= spi_data; timer <= 23'd0; state <= ST_WAIT_REC; end
                end

                ST_WAIT_REC: begin
                    if (timer >= (COUNT_1S * 2) - 1) begin timer <= 23'd0; state <= ST_PLS_SER_INIT; end
                    else timer <= timer + 1'b1;
                end

                ST_PLS_SER_INIT: begin
                    pulse_series <= 1'b1;
                    if (timer >= COUNT_5MS - 1) begin pulse_series <= 1'b0; timer <= 23'd0; state <= ST_WAIT_STAB_S; end
                    else timer <= timer + 1'b1;
                end
                ST_WAIT_STAB_S: begin
                    if (timer >= wait_stable_max - 1) begin spi_start <= 1'b1; state <= ST_READ_C; end
                    else timer <= timer + 1'b1;
                end
                ST_READ_C: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_C <= spi_data; timer <= 23'd0; state <= ST_WAIT_CHG_S; end
                end
                ST_WAIT_CHG_S: begin
                    if (timer >= wait_charge_max - 1) begin spi_start <= 1'b1; state <= ST_READ_D; end
                    else timer <= timer + 1'b1;
                end
                ST_READ_D: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_D <= spi_data; state <= ST_CALC_PREP_P; end
                end

                // ----------------------------------------
                // 【面積削減】順次乗算回路 (Shift & Add)
                // ----------------------------------------
                // (B - A) * (B + A) の準備
                ST_CALC_PREP_P: begin
                    mult_a <= (val_B > val_A) ? (val_B - val_A) : 17'd0;
                    mult_b <= {17'd0, (val_B + val_A)};
                    mult_acc <= 34'd0;
                    mult_cnt <= 5'd17; // 17回のシフト加算で乗算を完了
                    state <= ST_CALC_MULT_P;
                end

                ST_CALC_MULT_P: begin
                    if (mult_cnt == 5'd0) begin
                        diff_P <= mult_acc; // 乗算完了
                        state <= ST_CALC_PREP_S;
                    end else begin
                        if (mult_a[0]) mult_acc <= mult_acc + mult_b; // 最下位ビットが1なら足す
                        mult_a <= mult_a >> 1; // 右にシフト
                        mult_b <= mult_b << 1; // 左にシフト
                        mult_cnt <= mult_cnt - 1'b1;
                    end
                end

                // (D - C) * (D + C) の準備
                ST_CALC_PREP_S: begin
                    mult_a <= (val_D > val_C) ? (val_D - val_C) : 17'd0;
                    mult_b <= {17'd0, (val_D + val_C)};
                    mult_acc <= 34'd0;
                    mult_cnt <= 5'd17;
                    state <= ST_CALC_MULT_S;
                end

                ST_CALC_MULT_S: begin
                    if (mult_cnt == 5'd0) begin
                        diff_S <= mult_acc; // 乗算完了
                        state <= ST_CALC_CMP;
                    end else begin
                        if (mult_a[0]) mult_acc <= mult_acc + mult_b;
                        mult_a <= mult_a >> 1;
                        mult_b <= mult_b << 1;
                        mult_cnt <= mult_cnt - 1'b1;
                    end
                end

                // ----------------------------------------
                // 勝敗判定
                // ----------------------------------------
                ST_CALC_CMP: begin
                    if (diff_P > diff_S)       cmp_char <= 8'h3E; // '>'
                    else if (diff_P < diff_S)  cmp_char <= 8'h3C; // '<'
                    else                       cmp_char <= 8'h3D; // '='

                    if (diff_P >= diff_S) pulse_parallel <= 1'b1;
                    else                  pulse_series   <= 1'b1;

                    timer <= 23'd0;
                    state <= ST_PLS_WINNER;
                end

                ST_PLS_WINNER: begin
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_parallel <= 1'b0;
                        pulse_series   <= 1'b0;
                        tx_idx <= 6'd0;
                        state <= ST_TX_SEND;
                    end else timer <= timer + 1'b1;
                end

                // ----------------------------------------
                // 【面積削減】不要な送信データを削ぎ落としたUART
                // ----------------------------------------
                ST_TX_SEND: begin
                    if (!uart_busy && !uart_start) begin
                        uart_start <= 1'b1;
                        case (tx_idx)
                            0: uart_data <= hex2ascii(val_A[15:12]); 1: uart_data <= hex2ascii(val_A[11:8]);
                            2: uart_data <= hex2ascii(val_A[7:4]);   3: uart_data <= hex2ascii(val_A[3:0]);
                            4: uart_data <= 8'h2C;
                            5: uart_data <= hex2ascii(val_B[15:12]); 6: uart_data <= hex2ascii(val_B[11:8]);
                            7: uart_data <= hex2ascii(val_B[7:4]);   8: uart_data <= hex2ascii(val_B[3:0]);
                            9: uart_data <= 8'h2C;
                            10: uart_data <= hex2ascii(val_C[15:12]); 11: uart_data <= hex2ascii(val_C[11:8]);
                            12: uart_data <= hex2ascii(val_C[7:4]);   13: uart_data <= hex2ascii(val_C[3:0]);
                            14: uart_data <= 8'h2C;
                            15: uart_data <= hex2ascii(val_D[15:12]); 16: uart_data <= hex2ascii(val_D[11:8]);
                            17: uart_data <= hex2ascii(val_D[7:4]);   18: uart_data <= hex2ascii(val_D[3:0]);
                            19: uart_data <= 8'h2C;
                            20: uart_data <= cmp_char;  
                            21: uart_data <= 8'h0D;
                            22: uart_data <= 8'h0A;
                            default: uart_data <= 8'h00;
                        endcase
                    end else if (uart_start) begin
                        uart_start <= 1'b0;
                        state      <= ST_TX_WAIT;
                    end
                end

                ST_TX_WAIT: begin
                    if (!uart_start && !uart_busy) begin
                        if (tx_idx == 6'd22) begin // 最大インデックスを22に変更
                            state <= ST_IDLE; 
                        end else begin
                            tx_idx <= tx_idx + 1'b1;
                            state  <= ST_TX_SEND;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule


// =======================================================================
// [サブモジュール 2] LTC2450 SPI 読み出し (32.768kHz 駆動版)
// =======================================================================
module ltc2450_spi_read_sync_32k (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         busy,
    output reg         done,
    output reg [15:0]  data_out,
    output reg         spi_cs_n,
    output reg         spi_sck,
    input  wire        spi_sdo
);
    reg       sck_tick;
    reg [2:0] state;
    reg [4:0] bit_cnt;
    reg [15:0] shift_reg;
    reg [11:0] wait_cnt; 

    reg start_d1, start_d2;
    wire start_pulse = (start_d1 && !start_d2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {start_d1, start_d2} <= 2'b00;
        else        {start_d1, start_d2} <= {start, start_d1};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sck_tick <= 1'b0;
        else        sck_tick <= 1'b1; 
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 3'd0; busy <= 1'b0; done <= 1'b0; data_out <= 16'd0;
            spi_cs_n <= 1'b1; spi_sck <= 1'b0; bit_cnt <= 5'd0; shift_reg <= 16'd0; wait_cnt <= 12'd0;
        end else begin
            case (state)
                3'd0: begin 
                    done <= 1'b0;
                    if (start_pulse) begin busy <= 1'b1; spi_cs_n <= 1'b0; state <= 3'd1; end
                end
                3'd1: begin spi_sck <= 1'b0; bit_cnt <= 5'd0; state <= 3'd2; end 
                3'd2: begin 
                    if (sck_tick) begin
                        if (spi_sck == 1'b0) spi_sck <= 1'b1;
                        else begin
                            spi_sck <= 1'b0;
                            if (bit_cnt == 5'd15) begin
                                spi_cs_n <= 1'b1;
                                wait_cnt <= 12'd1311; // 40ms待機
                                state    <= 3'd3;
                            end else bit_cnt <= bit_cnt + 1'b1;
                        end
                    end
                end
                3'd3: begin 
                    if (wait_cnt == 12'd0) begin spi_cs_n <= 1'b0; state <= 3'd4; end
                    else wait_cnt <= wait_cnt - 1'b1;
                end
                3'd4: begin spi_sck <= 1'b0; bit_cnt <= 5'd0; shift_reg <= 16'd0; state <= 3'd5; end
                3'd5: begin 
                    if (sck_tick) begin
                        if (spi_sck == 1'b0) begin spi_sck <= 1'b1; shift_reg <= {shift_reg[14:0], spi_sdo}; end
                        else begin
                            spi_sck <= 1'b0;
                            if (bit_cnt == 5'd15) state <= 3'd6;
                            else bit_cnt <= bit_cnt + 1'b1;
                        end
                    end
                end
                3'd6: begin spi_cs_n <= 1'b1; busy <= 1'b0; done <= 1'b1; data_out <= shift_reg; state <= 3'd0; end
            endcase
        end
    end
endmodule


// =======================================================================
// [サブモジュール 3] UART 送信 (32.768kHz, 1200bps版)
// =======================================================================
module uart_tx_32k (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx_busy,
    output reg        uart_txd
);
    reg [4:0] clk_cnt; 
    reg [3:0] bit_cnt;
    reg [7:0] tx_reg;
    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 2'd0; tx_busy <= 1'b0; uart_txd <= 1'b1;
            clk_cnt <= 5'd0; bit_cnt <= 4'd0; tx_reg <= 8'd0;
        end else begin
            case (state)
                2'd0: begin 
                    tx_busy <= 1'b0; uart_txd <= 1'b1;
                    if (tx_start) begin
                        tx_reg <= tx_data; tx_busy <= 1'b1; uart_txd <= 1'b0;
                        clk_cnt <= 5'd0; state <= 2'd1;
                    end
                end
                2'd1: begin 
                    if (clk_cnt == 5'd26) begin clk_cnt <= 5'd0; uart_txd <= tx_reg[0]; bit_cnt <= 4'd0; state <= 2'd2; end
                    else clk_cnt <= clk_cnt + 1'b1;
                end
                2'd2: begin 
                    if (clk_cnt == 5'd26) begin
                        clk_cnt <= 5'd0;
                        if (bit_cnt == 4'd7) begin uart_txd <= 1'b1; state <= 2'd3; end
                        else begin tx_reg <= {1'b0, tx_reg[7:1]}; uart_txd <= tx_reg[1]; bit_cnt <= bit_cnt + 1'b1; end
                    end else clk_cnt <= clk_cnt + 1'b1;
                end
                2'd3: begin 
                    if (clk_cnt == 5'd26) state <= 2'd0;
                    else clk_cnt <= clk_cnt + 1'b1;
                end
            endcase
        end
    end
endmodule
