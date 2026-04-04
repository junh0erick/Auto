//`#start header` -- edit after this line, do not edit this line
// ========================================
//
// Copyright YOUR COMPANY, THE YEAR
// All Rights Reserved
// UNPUBLISHED, LICENSED SOFTWARE.
//
// CONFIDENTIAL AND PROPRIETARY INFORMATION
// WHICH IS THE PROPERTY OF your company.
//
// ========================================
`include "cypress.v"
//`#end` -- edit above this line, do not edit this line
// Generated on 01/05/2026 at 00:18
// Component: controladorLed
module controladorLed (
    output  Q,
    input   clk,
    input  [7:0] inp,
    input   next_state
);

    parameter lenContadorOff = 7;   // cantidad de ticks en OFF entre ráfagas

//`#start body` -- edit after this line, do not edit this line

    reg q_r;
    assign Q = q_r;

    // FSM states
    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_BURST = 2'd1;
    localparam [1:0] S_WAIT  = 2'd2;

    reg [1:0] state;
    reg [3:0] pulse_pos;  // posición dentro de 1010...
    reg [3:0] pulse_len;  // = 2*N - 1 (N>=1)
    reg [7:0] wait_cnt;   // cuenta de espera (ampliada por si lenContadorOff > 15)

    // Secuencia avanza con flanco de next_state
    always @(posedge next_state) begin
        case (state)

            // ------------------------------
            // IDLE
            // ------------------------------
            S_IDLE: begin
                q_r <= 1'b0;
                pulse_pos <= 0;

                if (inp != 0) begin
                    pulse_len <= (inp * 2) - 1;
                    state     <= S_BURST;
                end
            end

            // ------------------------------
            // BURST: genera 1010... patrón
            // ------------------------------
            S_BURST: begin
                q_r <= ~pulse_pos[0]; // par=1, impar=0

                if (pulse_pos == (pulse_len - 1)) begin
                    state    <= S_WAIT;
                    wait_cnt <= lenContadorOff; // usa el parámetro
                end else begin
                    pulse_pos <= pulse_pos + 1;
                end
            end

            // ------------------------------
            // WAIT: LED apagado lenContadorOff ticks
            // ------------------------------
            S_WAIT: begin
                q_r <= 1'b0;

                if (wait_cnt == 1) begin
                    // reinicia ciclo
                    if (inp != 0) begin
                        pulse_len <= (inp * 2) - 1;
                        pulse_pos <= 0;
                        state     <= S_BURST;
                    end else begin
                        state <= S_IDLE;
                    end
                end else begin
                    wait_cnt <= wait_cnt - 1;
                end
            end

            default: begin
                state <= S_IDLE;
                q_r   <= 1'b0;
            end

        endcase
    end

//`#end` -- edit above this line, do not edit this line
endmodule
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line
