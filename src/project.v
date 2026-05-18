/*
 * Copyright (c) 2024 Takaharu Yamada
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// =======================================================================
// [トップモジュール] Tiny Tapeout 向けラッパー
// =======================================================================
module tt_um_nezumi_tech_adc_sq_compare (
    input  wire [7:0] ui_in,    // 専用入力
    output wire [7:0] uo_out,   // 専用出力
    input  wire [7:0] uio_in,   // 双方向IO (入力パス)
    output wire [7:0] uio_out,  // 双方向IO (出力パス)
    output wire [7:0] uio_oe,   // 双方向IO 出力イネーブル (1=出力, 0=入力)
    input  wire       ena,      // デザインイネーブル
    input  wire       clk,      // システムクロック (32.768 kHz)
    input  wire       rst_n     // アクティブローリセット
);

    // ----------------------------------------------------
    // ピンの割り当て
    // ----------------------------------------------------
    wire       ext_trigger = ui_in[0];    // 計測開始トリガ
    wire       spi_sdo     = ui_in[1];    // LTC2450からのデータ入力
    wire [2:0] cfg_stable  = ui_in[4:2];  // 安定待ち時間設定 (2^N 秒)
    wire [2:0] cfg_charge  = ui_in[7:5];  // 充電待ち時間設定 (2^N 秒)

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

    // 今回、双方向ピン(uio)は使用しないためすべて入力モード(0)に固定
    assign uio_oe  = 8'b0000_0000;
    assign uio_out = 8'b0000_0000;

    // 未使用入力信号のワーニング回避 (Linter対策)
    wire _unused = &{ena, uio_in, 1'b0};

    // ----------------------------------------------------
    // コアロジックのインスタンス化
    // ----------------------------------------------------
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
// [サブモジュール 1] 環境発電最適化 コアロジック (32.768kHz 駆動版)
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
    
    // 5msのクロック数: 32,768 * 0.005 ≒ 164
    localparam COUNT_5MS = 164; 
    // 1秒のカウント値 (シフト演算用)
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
    localparam ST_CALC_SQ       = 5'd12;
    localparam ST_CALC_DIFF     = 5'd13;
    localparam ST_CALC_CMP      = 5'd14;
    localparam ST_PLS_WINNER    = 5'd15;
    localparam ST_TX_SEND       = 5'd16;
    localparam ST_TX_WAIT       = 5'd17;

    assign led = ~state[2:0]; // デバッグ用LED

    reg [22:0] timer; 
    
    // SPI通信制御用
    reg        spi_start;
    wire       spi_busy, spi_done;
    wire [15:0] spi_data;
    
    ltc2450_spi_read_sync_32k u_spi (
        .clk(clk), .rst_n(rst_n), .start(spi_start),
        .busy(spi_busy), .done(spi_done), .data_out(spi_data),
        .spi_cs_n(spi_cs_n), .spi_sck(spi_sck), .spi_sdo(spi_sdo)
    );

    // データ・演算レジスタ
    reg [15:0] val_A, val_B, val_C, val_D;
    reg signed [32:0] diff_P, diff_S;
    
    // UART送信用 絶対値・符号レジスタ
    reg [31:0] abs_diff_P, abs_diff_S;
    reg [7:0]  sign_P, sign_S, cmp_char;

    // UART制御用
    reg         uart_start;
    reg [7:0]   uart_data;
    wire        uart_busy;
    reg [6:0]   tx_idx;

    uart_tx_32k u_uart (
        .clk(clk), .rst_n(rst_n), .tx_start(uart_start),
        .tx_data(uart_data), .tx_busy(uart_busy), .uart_txd(uart_tx_pin)
    );

    // ユーティリティ関数
    function [7:0] hex2ascii(input [3:0] hex);
        begin
            if (hex < 4'd10) hex2ascii = 8'h30 + hex;
            else             hex2ascii = 8'h37 + hex;
        end
    endfunction

    // 外部トリガのエッジ検出
    reg trig_d1, trig_d2;
    wire trig_pulse = (trig_d1 && !trig_d2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {trig_d1, trig_d2} <= 2'b00;
        else        {trig_d1, trig_d2} <= {ext_trigger, trig_d1};
    end

    // メインステートマシン
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            timer <= 23'd0;
            pulse_parallel <= 1'b0;
            pulse_series   <= 1'b0;
            spi_start <= 1'b0;
            uart_start <= 1'b0;
            tx_idx <= 7'd0;
            {val_A, val_B, val_C, val_D} <= 64'd0;
            {sq_A, sq_B, sq_C, sq_D} <= 128'd0;
            diff_P <= 33'd0; diff_S <= 33'd0;
            abs_diff_P <= 32'd0; abs_diff_S <= 32'd0;
            sign_P <= 8'd0; sign_S <= 8'd0; cmp_char <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    pulse_parallel <= 1'b0;
                    pulse_series   <= 1'b0;
                    if (trig_pulse) begin
                        timer <= 23'd0;
                        state <= ST_PLS_PAR_INIT;
                    end
                end

                ST_PLS_PAR_INIT: begin
                    pulse_parallel <= 1'b1;
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_parallel <= 1'b0;
                        timer <= 23'd0;
                        state <= ST_WAIT_STAB_P;
                    end else timer <= timer + 1'b1;
                end

                ST_WAIT_STAB_P: begin
                    if (timer >= wait_stable_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_A;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_A: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_A <= spi_data; timer <= 23'd0; state <= ST_WAIT_CHG_P; end
                end

                ST_WAIT_CHG_P: begin
                    if (timer >= wait_charge_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_B;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_B: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_B <= spi_data; timer <= 23'd0; state <= ST_WAIT_REC; end
                end

                ST_WAIT_REC: begin
                    if (timer >= (COUNT_1S * 2) - 1) begin
                        timer <= 23'd0; state <= ST_PLS_SER_INIT;
                    end else timer <= timer + 1'b1;
                end

                ST_PLS_SER_INIT: begin
                    pulse_series <= 1'b1;
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_series <= 1'b0; timer <= 23'd0; state <= ST_WAIT_STAB_S;
                    end else timer <= timer + 1'b1;
                end

                ST_WAIT_STAB_S: begin
                    if (timer >= wait_stable_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_C;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_C: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_C <= spi_data; timer <= 23'd0; state <= ST_WAIT_CHG_S; end
                end

                ST_WAIT_CHG_S: begin
                    if (timer >= wait_charge_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_D;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_D: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_D <= spi_data; state <= ST_CALC_SQ; end
                end

// ----------------------------------------
                // 演算・最終決定 (因数分解による乗算器の削減)
                // ----------------------------------------
                // sq_A ~ sq_D のステートは削除し、差分の計算に直行します。
                
                ST_CALC_SQ: begin
                    // (B - A) * (B + A) を計算 (乗算器を1つだけ使用)
                    // ※オーバーフローを防ぐため、16bitを18bit符号付きに拡張して計算
                    diff_P <= ($signed({2'b00, val_B}) - $signed({2'b00, val_A})) * 
                              ($signed({2'b00, val_B}) + $signed({2'b00, val_A}));
                    state <= ST_CALC_DIFF;
                end

                ST_CALC_DIFF: begin
                    // (D - C) * (D + C) を計算 (上の乗算器を使い回す)
                    diff_S <= ($signed({2'b00, val_D}) - $signed({2'b00, val_C})) * 
                              ($signed({2'b00, val_D}) + $signed({2'b00, val_C}));
                    timer  <= 23'd0;
                    state  <= ST_CALC_CMP;
                end

                ST_CALC_CMP: begin
                    if (diff_P > diff_S)       cmp_char <= 8'h3E; // '>'
                    else if (diff_P < diff_S)  cmp_char <= 8'h3C; // '<'
                    else                       cmp_char <= 8'h3D; // '='

                    if (diff_P >= diff_S) pulse_parallel <= 1'b1;
                    else                  pulse_series   <= 1'b1;

                    // diff_P, diff_S は最大33bitなので、32bit分を絶対値として抽出
                    if (diff_P[32]) begin abs_diff_P <= -diff_P[31:0]; sign_P <= 8'h2D; /*-*/ end
                    else            begin abs_diff_P <=  diff_P[31:0]; sign_P <= 8'h2B; /*+*/ end

                    if (diff_S[32]) begin abs_diff_S <= -diff_S[31:0]; sign_S <= 8'h2D; /*-*/ end
                    else            begin abs_diff_S <=  diff_S[31:0]; sign_S <= 8'h2B; /*+*/ end

                    state <= ST_PLS_WINNER;
                end

                ST_PLS_WINNER: begin
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_parallel <= 1'b0;
                        pulse_series   <= 1'b0;
                        tx_idx <= 7'd0;
                        state <= ST_TX_SEND;
                    end else timer <= timer + 1'b1;
                end

                // ----------------------------------------
                // UART送信 (文字数を削減しマルチプレクサを小型化)
                // ----------------------------------------
                ST_TX_SEND: begin
                    if (!uart_busy && !uart_start) begin
                        uart_start <= 1'b1;
                        case (tx_idx)
                            // A, B, C, D (各4文字 + カンマ)
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

                            // diff_P (符号 + 8文字 + カンマ)
                            20: uart_data <= sign_P;
                            21: uart_data <= hex2ascii(abs_diff_P[31:28]); 22: uart_data <= hex2ascii(abs_diff_P[27:24]);
                            23: uart_data <= hex2ascii(abs_diff_P[23:20]); 24: uart_data <= hex2ascii(abs_diff_P[19:16]);
                            25: uart_data <= hex2ascii(abs_diff_P[15:12]); 26: uart_data <= hex2ascii(abs_diff_P[11:8]);
                            27: uart_data <= hex2ascii(abs_diff_P[7:4]);   28: uart_data <= hex2ascii(abs_diff_P[3:0]);
                            29: uart_data <= 8'h2C;

                            // diff_S (符号 + 8文字 + カンマ)
                            30: uart_data <= sign_S;
                            31: uart_data <= hex2ascii(abs_diff_S[31:28]); 32: uart_data <= hex2ascii(abs_diff_S[27:24]);
                            33: uart_data <= hex2ascii(abs_diff_S[23:20]); 34: uart_data <= hex2ascii(abs_diff_S[19:16]);
                            35: uart_data <= hex2ascii(abs_diff_S[15:12]); 36: uart_data <= hex2ascii(abs_diff_S[11:8]);
                            37: uart_data <= hex2ascii(abs_diff_S[7:4]);   38: uart_data <= hex2ascii(abs_diff_S[3:0]);
                            39: uart_data <= 8'h2C;

                            // 比較結果 & CRLF
                            40: uart_data <= cmp_char;  
                            41: uart_data <= 8'h0D;     // \r
                            42: uart_data <= 8'h0A;     // \n
                            default: uart_data <= 8'h00;
                        endcase
                    end else if (uart_start) begin
                        uart_start <= 1'b0;
                        state      <= ST_TX_WAIT;
                    end
                end

                ST_TX_WAIT: begin
                    if (!uart_start && !uart_busy) begin
                        if (tx_idx == 7'd42) begin // 最大インデックスを42に変更
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

    // 32.768kHzでは分周不要なため、クロックをそのまま利用
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
                3'd0: begin // IDLE
                    done <= 1'b0;
                    if (start_pulse) begin busy <= 1'b1; spi_cs_n <= 1'b0; state <= 3'd1; end
                end
                3'd1: begin spi_sck <= 1'b0; bit_cnt <= 5'd0; state <= 3'd2; end 
                3'd2: begin // DUMMY_DATA (空読み)
                    if (sck_tick) begin
                        if (spi_sck == 1'b0) spi_sck <= 1'b1;
                        else begin
                            spi_sck <= 1'b0;
                            if (bit_cnt == 5'd15) begin
                                spi_cs_n <= 1'b1;
                                wait_cnt <= 12'd1311; // 40ms待機 (32768 * 0.04)
                                state    <= 3'd3;
                            end else bit_cnt <= bit_cnt + 1'b1;
                        end
                    end
                end
                3'd3: begin // WAIT_CONV
                    if (wait_cnt == 12'd0) begin spi_cs_n <= 1'b0; state <= 3'd4; end
                    else wait_cnt <= wait_cnt - 1'b1;
                end
                3'd4: begin spi_sck <= 1'b0; bit_cnt <= 5'd0; shift_reg <= 16'd0; state <= 3'd5; end
                3'd5: begin // REAL_DATA (本番読み)
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
    // 32768 / 1200 ≒ 27
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
                2'd0: begin // IDLE
                    tx_busy <= 1'b0; uart_txd <= 1'b1;
                    if (tx_start) begin
                        tx_reg <= tx_data; tx_busy <= 1'b1; uart_txd <= 1'b0;
                        clk_cnt <= 5'd0; state <= 2'd1;
                    end
                end
                2'd1: begin // START
                    if (clk_cnt == 5'd26) begin clk_cnt <= 5'd0; uart_txd <= tx_reg[0]; bit_cnt <= 4'd0; state <= 2'd2; end
                    else clk_cnt <= clk_cnt + 1'b1;
                end
                2'd2: begin // DATA
                    if (clk_cnt == 5'd26) begin
                        clk_cnt <= 5'd0;
                        if (bit_cnt == 4'd7) begin uart_txd <= 1'b1; state <= 2'd3; end
                        else begin tx_reg <= {1'b0, tx_reg[7:1]}; uart_txd <= tx_reg[1]; bit_cnt <= bit_cnt + 1'b1; end
                    end else clk_cnt <= clk_cnt + 1'b1;
                end
                2'd3: begin // STOP
                    if (clk_cnt == 5'd26) state <= 2'd0;
                    else clk_cnt <= clk_cnt + 1'b1;
                end
            endcase
        end
    end
endmodule
