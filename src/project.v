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
// [サブモジュール 1] 環境発電最適化 コアロジック (面積極限・最適化版)
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

    // --- サブルーチン化による統合ステート定義 ---
    reg [3:0] state; // 12ステートなので4bitに削減
    localparam ST_IDLE         = 4'd0;
    localparam ST_PULSE_INIT   = 4'd1;
    localparam ST_WAIT_STAB    = 4'd2;
    localparam ST_READ_A       = 4'd3;
    localparam ST_WAIT_CHG     = 4'd4;
    localparam ST_READ_B       = 4'd5;
    localparam ST_CALC_PREP    = 4'd6;
    localparam ST_CALC_MULT    = 4'd7;
    localparam ST_TX_CHUNK     = 4'd8;
    localparam ST_TX_WAIT      = 4'd9;
    localparam ST_WAIT_REC     = 4'd10;
    localparam ST_PLS_WINNER   = 4'd11;

    assign led = ~state[2:0]; 

    reg [22:0] timer; 
    reg        is_series;   // 0: Parallel計測中, 1: Series計測中
    reg        is_negative; // 減算結果の符号フラグ
    
    reg        spi_start;
    wire       spi_busy, spi_done;
    wire [15:0] spi_data;
    
    ltc2450_spi_read_sync_32k u_spi (
        .clk(clk), .rst_n(rst_n), .start(spi_start),
        .busy(spi_busy), .done(spi_done), .data_out(spi_data),
        .spi_cs_n(spi_cs_n), .spi_sck(spi_sck), .spi_sdo(spi_sdo)
    );

    // --- ADCレジスタ (C, Dを廃止し、A, Bを使い回す) ---
    reg [15:0] val_A, val_B;
    
    // --- 順次乗算(Shift & Add)用レジスタ ---
    reg signed [33:0] diff_P, diff_S;
    reg [16:0] mult_a;
    reg [33:0] mult_b;
    reg [33:0] mult_acc;
    reg [4:0]  mult_cnt;
    
    // --- UART制御用 (シフトレジスタ化) ---
    reg         uart_start;
    reg [7:0]   uart_data;
    wire        uart_busy;
    reg [4:0]   tx_step;     // 0〜22のステップカウンタ
    reg [31:0]  hex_sr;      // データ押し出し用シフトレジスタ

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

    // 外部トリガエッジ検出
    reg trig_d1, trig_d2;
    wire trig_pulse = (trig_d1 && !trig_d2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {trig_d1, trig_d2} <= 2'b00;
        else        {trig_d1, trig_d2} <= {ext_trigger, trig_d1};
    end

    // --- オンザフライ計算 (組み合わせ回路) ---
    // レジスタを消費せず、UART送信の瞬間に動的に計算します
    wire [31:0] w_abs_diff = is_series ? 
                             (diff_S[33] ? -diff_S[31:0] : diff_S[31:0]) :
                             (diff_P[33] ? -diff_P[31:0] : diff_P[31:0]);

    wire [7:0]  w_sign     = is_series ? 
                             (diff_S[33] ? 8'h2D /*-*/ : 8'h2B /*+*/) :
                             (diff_P[33] ? 8'h2D /*-*/ : 8'h2B /*+*/);

    wire [7:0]  w_cmp_char = (diff_P > diff_S) ? 8'h3E /*>*/ : 
                            ((diff_P < diff_S) ? 8'h3C /*<*/ : 8'h3D /*=*/);

    // --- メインステートマシン ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            timer <= 23'd0;
            is_series <= 1'b0;
            is_negative <= 1'b0;
            pulse_parallel <= 1'b0;
            pulse_series <= 1'b0;
            spi_start <= 1'b0;
            uart_start <= 1'b0;
            tx_step <= 5'd0;
            hex_sr <= 32'd0;
            {val_A, val_B} <= 32'd0;
            diff_P <= 34'd0; diff_S <= 34'd0;
            mult_a <= 17'd0; mult_b <= 34'd0; mult_acc <= 34'd0; mult_cnt <= 5'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    pulse_parallel <= 1'b0;
                    pulse_series   <= 1'b0;
                    if (trig_pulse) begin
                        is_series <= 1'b0; // Parallelから開始
                        timer <= 23'd0;
                        state <= ST_PULSE_INIT;
                    end
                end

                ST_PULSE_INIT: begin
                    pulse_parallel <= !is_series;
                    pulse_series   <=  is_series;
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_parallel <= 1'b0;
                        pulse_series   <= 1'b0;
                        timer <= 23'd0;
                        state <= ST_WAIT_STAB;
                    end else timer <= timer + 1'b1;
                end

                ST_WAIT_STAB: begin
                    if (timer >= wait_stable_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_A;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_A: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_A <= spi_data; timer <= 23'd0; state <= ST_WAIT_CHG; end
                end

                ST_WAIT_CHG: begin
                    if (timer >= wait_charge_max - 1) begin
                        spi_start <= 1'b1; state <= ST_READ_B;
                    end else timer <= timer + 1'b1;
                end

                ST_READ_B: begin
                    spi_start <= 1'b0;
                    if (spi_done) begin val_B <= spi_data; state <= ST_CALC_PREP; end
                end

                // --- 面積極限削減: Shift & Add 乗算 ---
                // (B - A) * (B + A) を計算
                ST_CALC_PREP: begin
                    mult_a <= (val_B > val_A) ? (val_B - val_A) : (val_A - val_B); // 絶対値
                    mult_b <= {17'd0, (val_B + val_A)};
                    mult_acc <= 34'd0;
                    mult_cnt <= 5'd17;
                    is_negative <= (val_A > val_B); // 符号フラグ
                    state <= ST_CALC_MULT;
                end

                ST_CALC_MULT: begin
                    if (mult_cnt == 5'd0) begin
                        // 計算完了、符号を適用して保存
                        if (!is_series) diff_P <= is_negative ? -$signed({1'b0, mult_acc[32:0]}) : $signed({1'b0, mult_acc[32:0]});
                        else            diff_S <= is_negative ? -$signed({1'b0, mult_acc[32:0]}) : $signed({1'b0, mult_acc[32:0]});
                        
                        tx_step <= 5'd0;
                        hex_sr  <= {val_A, val_B}; // 送信用のシフトレジスタにA,Bをロード
                        state   <= ST_TX_CHUNK;
                    end else begin
                        if (mult_a[0]) mult_acc <= mult_acc + mult_b;
                        mult_a <= mult_a >> 1;
                        mult_b <= mult_b << 1;
                        mult_cnt <= mult_cnt - 1'b1;
                    end
                end

                // --- 配線混雑解消: シフトレジスタ方式のUART ---
                ST_TX_CHUNK: begin
                    if (!uart_busy && !uart_start) begin
                        uart_start <= 1'b1;
                        // 送信と同時にレジスタをシフトし、巨大なMUXを消去
                        if (tx_step == 4 || tx_step == 9 || tx_step == 19) begin
                            uart_data <= 8'h2C; // ',' カンマ
                            if (tx_step == 9) hex_sr <= w_abs_diff; // 差分の絶対値をロード
                        end else if (tx_step == 10) begin
                            uart_data <= w_sign;
                        end else if (tx_step == 20) begin
                            uart_data <= w_cmp_char;
                        end else if (tx_step == 21) begin
                            uart_data <= 8'h0D; // '\r'
                        end else if (tx_step == 22) begin
                            uart_data <= 8'h0A; // '\n'
                        end else begin
                            uart_data <= hex2ascii(hex_sr[31:28]);
                            hex_sr <= hex_sr << 4;
                        end
                        state <= ST_TX_WAIT;
                    end
                end

                ST_TX_WAIT: begin
                    if (!uart_start && !uart_busy) begin
                        if (!is_series && tx_step == 19) begin
                            // Parallel側の送信完了 (カンマで終わる)。次はSeriesへ。
                            is_series <= 1'b1;
                            timer <= 23'd0;
                            state <= ST_WAIT_REC;
                        end else if (is_series && tx_step == 22) begin
                            // Series側の送信完了 (改行で終わる)。勝者決定へ。
                            timer <= 23'd0;
                            state <= ST_PLS_WINNER;
                        end else begin
                            tx_step <= tx_step + 1'b1;
                            state <= ST_TX_CHUNK;
                        end
                    end
                end

                ST_WAIT_REC: begin
                    // 次の回路(Series)へ移る前に2秒待機して放電/安定化
                    if (timer >= (COUNT_1S * 2) - 1) begin
                        timer <= 23'd0; state <= ST_PULSE_INIT;
                    end else timer <= timer + 1'b1;
                end

                ST_PLS_WINNER: begin
                    // 勝者回路へ5msの切り替えパルスを出力して終了
                    pulse_parallel <= (diff_P >= diff_S);
                    pulse_series   <= (diff_P <  diff_S);
                    if (timer >= COUNT_5MS - 1) begin
                        pulse_parallel <= 1'b0;
                        pulse_series   <= 1'b0;
                        state <= ST_IDLE;
                    end else timer <= timer + 1'b1;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule


// =======================================================================
// [サブモジュール 2] LTC2450 SPI 読み出し
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
// [サブモジュール 3] UART 送信 (1200bps)
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
