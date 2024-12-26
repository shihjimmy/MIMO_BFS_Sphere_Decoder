`timescale 1ns / 1ps
module sphere_decoder #(
    parameter WI = 10
)(
    input Clk,
    input Reset,
    input in_valid,
    input flagChannelorData,
    input [ (4*2*WI)-1 : 0] InData,
    output [11 : 0] OutData,
    output out_valid
);

    localparam S_IDLE = 3'd0;
    localparam S_GET = 3'd1;
    localparam S_LAYER4 = 3'd2;
    localparam S_LAYER3 = 3'd3;
    localparam S_LAYER2 = 3'd4;
    localparam S_LAYER1 = 3'd5;
    localparam S_OUT = 3'd6;
    localparam S_PREP = 3'd7;
    
    reg [2:0] state_r, state_w;
    reg [2:0] count_PSK_r, count_PSK_w;
    reg [3:0] count_best_r, count_best_w;

    // 12 bits for index and 10 bits for PED
    reg [ (WI+11) : 0] k_best_r[0:7], k_best_w[0:7];
    reg [ (WI+11) : 0] k_prev_r[0:7], k_prev_w[0:7];

    // R11, R12, R13, R14, R22, R23, R24, R33, R34, R44
    reg [ (2*WI-1) : 0] channel_r[0:9], channel_w[0:9];
    // received data from RX
    reg [ (2*WI-1) : 0] z_r[0:3], z_w[0:3];

    // pipelined-1
    reg [ (2*WI-1) : 0] pipelined_1_data1_r;
    reg [ (2*WI-1) : 0] pipelined_1_data2_r;
    reg [ (2*WI-1) : 0] pipelined_1_data3_r;
    reg [ (2*WI-1) : 0] pipelined_1_data4_r;
    reg [2:0] pipelined_1_PSK_r;
    reg [ (WI+11) : 0] pipelined_1_prev_r;

    // pipelined-2
    reg [ (2*WI-1) : 0] pipelined_2_data1_r;
    reg [2:0] pipelined_2_PSK_r;
    reg [ (WI+11) : 0] pipelined_2_prev_r;

    wire [WI-1 : 0] real_1, imag_1;
    wire [WI-1 : 0] real_2, imag_2;
    wire [WI-1 : 0] real_3, imag_3;
    wire [WI-1 : 0] real_4, imag_4;
    wire [WI-1 : 0] temp1, temp2, temp3, temp4, temp5, temp6;
    wire [WI-1 : 0] final_dat_real, final_dat_imag;
    reg  [WI-1 : 0] current_z_real, current_z_imag;
    

    // pipelined -3
    reg [ (WI-1) : 0] pipelined_3_data1_r;
    reg [11:0] pipelined_3_index_r;
    reg [2:0] pipelined_3_PSK_r;

    wire [WI-1 : 0] real_abs, imag_abs;
    wire [WI-1 : 0] temp9;
    wire [WI-1 : 0] final_dat_pipeleine3;

    // for selection
    reg [ (WI+11) : 0] choose_for_kbest[0:7];
    reg [WI+11 :0] next_best_data;

    reg comp1, comp2, comp3, comp4, comp5, comp6, comp7, comp8;
    wire big1, big2, big3, big4, big5, big6, big7, big8;
    wire can_start_compare;
    wire [3:0] pos;

    // for complex mult units
    reg [2*WI-1 : 0] Rij_1, Rij_2, Rij_3, Rij_4;
    reg [2:0] PSK_1, PSK_2, PSK_3, PSK_4;
    wire [2*WI-1 : 0] result_1, result_2, result_3, result_4;


    // Input buffer
    reg flagChannelorData_r;
    reg in_valid_r;
    reg [ (4*2*WI)-1 : 0] InData_r;

    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            in_valid_r <= 1'd0;
            flagChannelorData_r <= 1'd0;
            InData_r <= {(8*WI){1'd0}};
        end
        else begin
            flagChannelorData_r <= flagChannelorData;
            in_valid_r <= in_valid;
            InData_r <= InData;
        end
    end


    /*--------------------------------------------*/
    // output
    reg out_valid_r;
    wire out_valid_w;

    assign out_valid = out_valid_r;
    assign out_valid_w = state_w==S_OUT; 
    assign OutData = k_prev_r[0][WI +: 12];
    
    /*--------------------------------------------*/
    // GET DATA

    integer i;
    always@ (*) begin
        for(i=0 ; i<=9; i=i+1) begin
            channel_w[i] = channel_r[i];
        end

        if(in_valid_r && flagChannelorData_r) begin
            if(count_best_r != 3'd2) begin
                // get R11 R12 R13 R14 R22 R23 R24 R33 within 2 cycles
                channel_w[0] = channel_r[4];
                channel_w[1] = channel_r[5];
                channel_w[2] = channel_r[6];
                channel_w[3] = channel_r[7];
                channel_w[4] = channel_r[8];
                channel_w[5] = channel_r[9];
                channel_w[6] = InData_r[0 +: (2*WI)];
                channel_w[7] = InData_r[(2*WI) +: (2*WI)];
                channel_w[8] = InData_r[(4*WI) +: (2*WI)];
                channel_w[9] = InData_r[(6*WI) +: (2*WI)];
            end
            else begin
                // get R34 R44 in last cycle
                channel_w[0] = channel_r[2];
                channel_w[1] = channel_r[3];
                channel_w[2] = channel_r[4];
                channel_w[3] = channel_r[5];
                channel_w[4] = channel_r[6];
                channel_w[5] = channel_r[7];
                channel_w[6] = channel_r[8];
                channel_w[7] = channel_r[9];
                channel_w[8] = InData_r[0 +: (2*WI)];
                channel_w[9] = InData_r[(2*WI) +: (2*WI)];
            end
        end
    end

    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            for(i=0 ; i<=9; i=i+1) begin
                channel_r[i] <= {(2*WI){1'd0}};
            end
        end
        else begin
            for(i=0 ; i<=9; i=i+1) begin
                channel_r[i] <= channel_w[i];
            end
        end
    end

    always@ (*) begin
        for(i=0 ; i<=3; i=i+1) begin
            z_w[i] = z_r[i];
        end

        if(in_valid_r && (!flagChannelorData_r)) begin
            z_w[0] = InData_r[0 +: (WI*2)];
            z_w[1] = InData_r[(2*WI) +: (WI*2)];
            z_w[2] = InData_r[(4*WI) +: (WI*2)];
            z_w[3] = InData_r[(6*WI) +: (WI*2)];
        end
    end
    
    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            for(i=0 ; i<=3; i=i+1) begin
                z_r[i] <= {(2*WI){1'd0}};
            end
        end
        else begin
            for(i=0 ; i<=3; i=i+1) begin
                z_r[i] <= z_w[i];
            end
        end
    end

    /*--------------------------------------------*/

    /*--------------------------------------------*/
    // state transition

    always@ (*) begin
        case(state_r)
            S_IDLE: state_w = (in_valid_r) ? S_GET : S_IDLE;
            S_GET:  state_w = (!in_valid_r) ? S_LAYER4 : S_GET;
            S_LAYER4: state_w = (count_best_r==4'b0100) ? S_LAYER3 : S_LAYER4;
            S_LAYER3: state_w = (count_best_r==4'b1011) ? S_LAYER2 : S_LAYER3;
            S_LAYER2: state_w = (count_best_r==4'b1011) ? S_LAYER1 : S_LAYER2;
            S_LAYER1: state_w = (count_best_r==4'b1011) ? S_OUT : S_LAYER1;
            S_OUT: state_w = (in_valid && (!flagChannelorData)) ? S_PREP : S_OUT; 
            S_PREP: state_w = S_LAYER4;
            default: state_w = state_r;
        endcase
    end

    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            state_r <= S_IDLE;
            out_valid_r <= 1'd0;
        end
        else begin
            state_r <= state_w;
            out_valid_r <= out_valid_w;
        end
    end

    /*--------------------------------------------*/

    /*--------------------------------------------*/
    // counters

    // count_PSK is used for counting current 8PSK possibility
    // count_best is used for counting current best node under test and current data flow

    always@ (*) begin
        count_PSK_w = 3'd0;

        if(state_r == S_LAYER4) begin
            if(state_w == S_LAYER3)
                count_PSK_w = 3'd0;
            else if(&count_PSK_r)
                count_PSK_w = count_PSK_r;
            else if (&(~count_best_r)) // all zero
                count_PSK_w = count_PSK_r + 3'd1;  
        end
        else if(state_r >= 3'd3 && state_r <= 3'd5) begin //Layer3 ~ 1
            if (!count_best_r[3])
                count_PSK_w = count_PSK_r + 3'd1;
        end
    end

    wire [3:0] temp10;
    assign temp10 = count_best_r + 4'd1;

    always@ (*) begin
        if((in_valid_r && flagChannelorData_r)) begin
            count_best_w = temp10;
        end
        else if(state_r != state_w) begin
            count_best_w = 4'd0;
        end
        else if( (&count_PSK_r) || (count_best_r[3] && count_best_r!=4'b1011) ) begin
            count_best_w = temp10;
        end
        else begin
            count_best_w = count_best_r;
        end
    end

    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            count_PSK_r <= 3'd0;
            count_best_r <= 4'd0;
        end
        else begin
            count_PSK_r <= count_PSK_w;
            count_best_r <= count_best_w;
        end
    end

    /*--------------------------------------------*/

    /*--------------------------------------------*/
    complex_mult_approx_unit #(.WI(WI)) u1 (.Rij(Rij_1), .PSK(PSK_1), .result(result_1));
    complex_mult_approx_unit #(.WI(WI)) u2 (.Rij(Rij_2), .PSK(PSK_2), .result(result_2));
    complex_mult_approx_unit #(.WI(WI)) u3 (.Rij(Rij_3), .PSK(PSK_3), .result(result_3));
    complex_mult_approx_unit #(.WI(WI)) u4 (.Rij(Rij_4), .PSK(PSK_4), .result(result_4));
    /*--------------------------------------------*/


    always@ (*) begin
        if(state_r == S_LAYER4) begin
            Rij_1 = channel_r[9]; //R44
            Rij_2 = {(2*WI){1'd0}}; 
            Rij_3 = {(2*WI){1'd0}}; 
            Rij_4 = {(2*WI){1'd0}}; 
        end
        else if(state_r == S_LAYER3) begin
            Rij_1 = channel_r[7];
            Rij_2 = channel_r[8];
            Rij_3 = {(2*WI){1'd0}}; 
            Rij_4 = {(2*WI){1'd0}}; 
        end
        else if(state_r == S_LAYER2) begin
            Rij_1 = channel_r[4];
            Rij_2 = channel_r[5];
            Rij_3 = channel_r[6];
            Rij_4 = {(2*WI){1'd0}}; 
        end
        else if(state_r == S_LAYER1) begin
            Rij_1 = channel_r[0];
            Rij_2 = channel_r[1];
            Rij_3 = channel_r[2];
            Rij_4 = channel_r[3];
        end
        else begin
            Rij_1 = {(2*WI){1'd0}}; 
            Rij_2 = {(2*WI){1'd0}}; 
            Rij_3 = {(2*WI){1'd0}}; 
            Rij_4 = {(2*WI){1'd0}}; 
        end
    end

    always@ (*) begin
        if(state_r == S_LAYER4) begin
            PSK_1 = count_PSK_r;
            PSK_2 = 3'd0;
            PSK_3 = 3'd0;
            PSK_4 = 3'd0;
        end
        else if(state_r == S_LAYER3) begin
            PSK_1 = count_PSK_r;
            PSK_2 = k_prev_r[count_best_r[2:0]][WI +: 3];
            PSK_3 = 3'd0;
            PSK_4 = 3'd0;
        end
        else if(state_r == S_LAYER2) begin
            PSK_1 = count_PSK_r;
            PSK_2 = k_prev_r[count_best_r[2:0]][(WI+3) +: 3];
            PSK_3 = k_prev_r[count_best_r[2:0]][WI +: 3];
            PSK_4 = 3'd0;
        end
        else if(state_r == S_LAYER1) begin
            PSK_1 = count_PSK_r;
            PSK_2 = k_prev_r[count_best_r[2:0]][(WI+6) +: 3];
            PSK_3 = k_prev_r[count_best_r[2:0]][(WI+3) +: 3];
            PSK_4 = k_prev_r[count_best_r[2:0]][WI +: 3];
        end
        else begin
            PSK_1 = 3'd0;
            PSK_2 = 3'd0;
            PSK_3 = 3'd0;
            PSK_4 = 3'd0;
        end
    end

    always@ (*) begin
        case(state_r)
            S_LAYER4: begin
                current_z_real = z_r[3][WI +: WI];
                current_z_imag = z_r[3][0 +: WI];
            end

            S_LAYER3: begin
                current_z_real = z_r[2][WI +: WI];
                current_z_imag = z_r[2][0  +: WI];
            end

            S_LAYER2: begin
                current_z_real = z_r[1][WI +: WI];
                current_z_imag = z_r[1][0  +: WI];
            end
            
            S_LAYER1: begin
                current_z_real = z_r[0][WI +: WI];
                current_z_imag = z_r[0][0  +: WI];
            end

            default: begin
                current_z_real = {WI{1'd0}};
                current_z_imag = {WI{1'd0}};
            end
        endcase
    end

    assign real_1 = pipelined_1_data1_r[WI +: (WI)];
    assign imag_1 = pipelined_1_data1_r[0  +: (WI)];
    assign real_2 = pipelined_1_data2_r[WI +: (WI)];
    assign imag_2 = pipelined_1_data2_r[0  +: (WI)];
    assign real_3 = pipelined_1_data3_r[WI +: (WI)];
    assign imag_3 = pipelined_1_data3_r[0  +: (WI)];
    assign real_4 = pipelined_1_data4_r[WI +: (WI)];
    assign imag_4 = pipelined_1_data4_r[0  +: (WI)];


    assign temp1 = real_1 + real_2;
    assign temp2 = imag_1 + imag_2;
    assign temp3 = real_3 + real_4;
    assign temp4 = imag_3 + imag_4;
    assign temp5 = temp1 + temp3;
    assign temp6 = temp2 + temp4;
    assign final_dat_real = current_z_real - temp5;
    assign final_dat_imag = current_z_imag - temp6;


    assign real_abs = (pipelined_2_data1_r[2*WI-1]) ? (~pipelined_2_data1_r[WI +: WI] + { {(WI-1){1'd0}}, 1'd1}) : pipelined_2_data1_r[WI +: WI];
    assign imag_abs = (pipelined_2_data1_r[WI-1])   ? (~pipelined_2_data1_r[0 +:  WI] + { {(WI-1){1'd0}}, 1'd1}) : pipelined_2_data1_r[0 +: WI];
    assign temp9 = real_abs + imag_abs;
    assign final_dat_pipeleine3 = temp9 + pipelined_2_prev_r[0 +: WI];


    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            pipelined_1_data1_r <= {(2*WI){1'd0}}; 
            pipelined_1_data2_r <= {(2*WI){1'd0}};
            pipelined_1_data3_r <= {(2*WI){1'd0}};
            pipelined_1_data4_r <= {(2*WI){1'd0}};
            pipelined_1_prev_r  <= {(WI+12){1'd0}};
            pipelined_1_PSK_r   <= 3'd0;

            pipelined_2_data1_r <= {(2*WI){1'd0}};
            pipelined_2_prev_r  <= {(WI+12){1'd0}};
            pipelined_2_PSK_r   <= 3'd0;

            pipelined_3_data1_r <= {(2*WI){1'd0}};
            pipelined_3_index_r <= 12'd0;
            pipelined_3_PSK_r   <= 3'd0;
        end
        else begin
            pipelined_1_data1_r <= result_1;
            pipelined_1_data2_r <= result_2;
            pipelined_1_data3_r <= result_3;
            pipelined_1_data4_r <= result_4;
            pipelined_1_prev_r  <= k_prev_r[count_best_r[2:0]];
            pipelined_1_PSK_r   <= count_PSK_r;

            pipelined_2_data1_r <= {final_dat_real, final_dat_imag};
            pipelined_2_prev_r  <= pipelined_1_prev_r;
            pipelined_2_PSK_r   <= pipelined_1_PSK_r;

            pipelined_3_data1_r <= final_dat_pipeleine3;
            pipelined_3_index_r <= pipelined_2_prev_r[WI +: 12];
            pipelined_3_PSK_r   <= pipelined_2_PSK_r;
        end
    end

    /*--------------------------------------------------*/
    // choose_for_kbest k-best result (k=8)
    assign pos = comp1 + comp2 + comp3 + comp4 + comp5 + comp6 + comp7 + comp8;
    assign big1 = pipelined_3_data1_r >= k_best_r[0][0 +: WI];
    assign big2 = pipelined_3_data1_r >= k_best_r[1][0 +: WI];
    assign big3 = pipelined_3_data1_r >= k_best_r[2][0 +: WI];
    assign big4 = pipelined_3_data1_r >= k_best_r[3][0 +: WI];
    assign big5 = pipelined_3_data1_r >= k_best_r[4][0 +: WI];
    assign big6 = pipelined_3_data1_r >= k_best_r[5][0 +: WI];
    assign big7 = pipelined_3_data1_r >= k_best_r[6][0 +: WI];
    assign big8 = pipelined_3_data1_r >= k_best_r[7][0 +: WI];


    // the time point that already all trash data is cleaned out from the pipeline
    assign can_start_compare = ( {count_best_r, count_PSK_r} > 7'b0001010 ) && (state_r >= 3'd3 && state_r <= 3'd5);

    always@ (*) begin
        if(can_start_compare) begin
            comp1 = big1;
            comp2 = big2;
            comp3 = big3;
            comp4 = big4;
            comp5 = big5;
            comp6 = big6;
            comp7 = big7;
            comp8 = big8;
        end
        else begin
            case(pipelined_3_PSK_r)
                3'd0: begin
                    comp1 = 1'd0;
                    comp2 = 1'd0;
                    comp3 = 1'd0;
                    comp4 = 1'd0;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd1: begin
                    comp1 = big1;
                    comp2 = 1'd0;
                    comp3 = 1'd0;
                    comp4 = 1'd0;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd2: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = 1'd0;
                    comp4 = 1'd0;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd3: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = big3;
                    comp4 = 1'd0;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd4: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = big3;
                    comp4 = big4;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd5: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = big3;
                    comp4 = big4;
                    comp5 = big5;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd6: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = big3;
                    comp4 = big4;
                    comp5 = big5;
                    comp6 = big6;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end

                3'd7: begin
                    comp1 = big1;
                    comp2 = big2;
                    comp3 = big3;
                    comp4 = big4;
                    comp5 = big5;
                    comp6 = big6;
                    comp7 = big7;
                    comp8 = 1'd0;
                end

                default: begin
                    comp1 = 1'd0;
                    comp2 = 1'd0;
                    comp3 = 1'd0;
                    comp4 = 1'd0;
                    comp5 = 1'd0;
                    comp6 = 1'd0;
                    comp7 = 1'd0;
                    comp8 = 1'd0;
                end
            endcase
        end
    end


    always@ (*) begin
        case(state_r)
            S_LAYER4: next_best_data = {9'd0, pipelined_3_PSK_r, pipelined_3_data1_r};
            S_LAYER3: next_best_data = {6'd0, pipelined_3_PSK_r, pipelined_3_index_r[0 +: 3], pipelined_3_data1_r};
            S_LAYER2: next_best_data = {3'd0, pipelined_3_PSK_r, pipelined_3_index_r[0 +: 6], pipelined_3_data1_r};
            S_LAYER1: next_best_data = {pipelined_3_PSK_r, pipelined_3_index_r[0 +: 9], pipelined_3_data1_r};
            default: next_best_data = {(WI+12){1'd0}};
        endcase
    end


    always@ (*) begin
        case(pos)
            4'd0: begin
                choose_for_kbest[0] = next_best_data;
                choose_for_kbest[1] = k_best_r[0];
                choose_for_kbest[2] = k_best_r[1];
                choose_for_kbest[3] = k_best_r[2];
                choose_for_kbest[4] = k_best_r[3];
                choose_for_kbest[5] = k_best_r[4];
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];          
            end

            4'd1: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = next_best_data;
                choose_for_kbest[2] = k_best_r[1];
                choose_for_kbest[3] = k_best_r[2];
                choose_for_kbest[4] = k_best_r[3];
                choose_for_kbest[5] = k_best_r[4];
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];      
            end

            4'd2: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = next_best_data;
                choose_for_kbest[3] = k_best_r[2];
                choose_for_kbest[4] = k_best_r[3];
                choose_for_kbest[5] = k_best_r[4];
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];       
            end

            4'd3: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = next_best_data;
                choose_for_kbest[4] = k_best_r[3];
                choose_for_kbest[5] = k_best_r[4];
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];       
            end

            4'd4: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = k_best_r[3];
                choose_for_kbest[4] = next_best_data;
                choose_for_kbest[5] = k_best_r[4];
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];      
            end

            4'd5: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = k_best_r[3];
                choose_for_kbest[4] = k_best_r[4];
                choose_for_kbest[5] = next_best_data;
                choose_for_kbest[6] = k_best_r[5];
                choose_for_kbest[7] = k_best_r[6];       
            end

            4'd6: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = k_best_r[3];
                choose_for_kbest[4] = k_best_r[4];
                choose_for_kbest[5] = k_best_r[5];
                choose_for_kbest[6] = next_best_data;
                choose_for_kbest[7] = k_best_r[6];       
            end

            4'd7: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = k_best_r[3];
                choose_for_kbest[4] = k_best_r[4];
                choose_for_kbest[5] = k_best_r[5];
                choose_for_kbest[6] = k_best_r[6];
                choose_for_kbest[7] = next_best_data;      
            end

            4'd8: begin
                choose_for_kbest[0] = k_best_r[0];
                choose_for_kbest[1] = k_best_r[1];
                choose_for_kbest[2] = k_best_r[2];
                choose_for_kbest[3] = k_best_r[3];
                choose_for_kbest[4] = k_best_r[4];
                choose_for_kbest[5] = k_best_r[5];
                choose_for_kbest[6] = k_best_r[6];
                choose_for_kbest[7] = k_best_r[7];
            end

            default: begin
                for(i=0 ;i<=7; i=i+1) begin
                    choose_for_kbest[i] = {(WI+12){1'd0}};
                end
            end
        endcase
    end

    /*--------------------------------------------------*/

    always@ (*) begin
        for(i=0 ;i<=7 ;i=i+1) begin
            k_best_w[i] = choose_for_kbest[i];
        end
    end

    always@ (*) begin
        for(i=0 ;i<=7 ;i=i+1) begin
            k_prev_w[i] = k_prev_r[i];
        end

        if ((state_r == S_LAYER4 && (count_best_r==4'b0100)) ||  (state_r == S_LAYER3 && (count_best_r==4'b1011)) ||
            (state_r == S_LAYER2 && (count_best_r==4'b1011)) ||  (state_r == S_LAYER1 && (count_best_r==4'b1011))) begin
            // only changes the prev of k-best in the last moment
            for(i=0 ;i<=7 ;i=i+1) begin
                k_prev_w[i] = k_best_r[i];
            end
        end
        else if(state_r == S_PREP) begin
            for(i=0 ;i<=7 ;i=i+1) begin
                k_prev_w[i] = {(WI+12){1'd0}};
            end
        end
    end

    always@ (posedge Clk or posedge Reset) begin
        if(Reset) begin
            for(i=0 ;i<=7 ;i=i+1) begin
                k_best_r[i] <= {(WI+12){1'd0}};
                k_prev_r[i] <= {(WI+12){1'd0}};
            end
        end
        else begin
            for(i=0 ;i<=7 ;i=i+1) begin
                k_best_r[i] <= k_best_w[i];
                k_prev_r[i] <= k_prev_w[i];
            end
        end
    end


endmodule


module sqrt2_reciprocal_approx_unit #(
    parameter WI = 10
)(
    input  [ (WI-1) : 0] x,
    output [ (WI-1) : 0] y
);
    // 1/(sqrt(2)) is about 0.707
    // 0.707 is about 0.1011010011 in binary 
    // choose to implement 0.707 in fract=6 bits

    wire [WI+5 : 0] temp1, temp2, temp3;
    wire [WI+5 : 0] shift1, shift2, shift3, shift4;
    
    assign shift1 = { {6{x[WI-1]}} , x[WI-1 : 6], x[5:0]};
    assign shift2 = { {4{x[WI-1]}} , x[WI-1 : 4], x[3:0], 2'd0};
    assign shift3 = { {3{x[WI-1]}} , x[WI-1 : 3], x[2:0], 3'd0};
    assign shift4 = { {1{x[WI-1]}} , x[WI-1 : 1], x[0], 5'd0};

    assign temp1 = shift1 + shift2;
    assign temp2 = shift3 + shift4;
    assign temp3 = temp1  + temp2;
    assign y = temp3[6 +: WI];

endmodule

module complex_mult_approx_unit #(
    parameter WI = 10
)(
    input [2*WI-1 : 0] Rij, //a + jb
    input [2:0] PSK,
    output [2*WI-1 : 0] result
);
    wire [WI-1:0] real_R, imag_R;
    wire [WI-1:0] res1, res2;
    wire [WI-1:0] temp1, temp2, temp3, temp4;

    reg [WI-1:0] data1, data2, data3, data4;
    wire [WI-1:0] real_res, imag_res;
    
    assign real_R = Rij[WI +: WI]; //a
    assign imag_R = Rij[0 +: WI];  //b

    sqrt2_reciprocal_approx_unit #(.WI(WI)) u0 (.x(real_R), .y(res1)); //0.707a
    sqrt2_reciprocal_approx_unit #(.WI(WI)) u1 (.x(imag_R), .y(res2)); //0.707b

    assign temp1 = ~real_R + { {(WI-1){1'd0}}, 1'd1}; //-a
    assign temp2 = ~imag_R + { {(WI-1){1'd0}}, 1'd1}; //-b
    assign temp3 = ~res1 + { {(WI-1){1'd0}}, 1'd1};   // -0.707a
    assign temp4 = ~res2 + { {(WI-1){1'd0}}, 1'd1};   // -0.707b

    assign real_res = data1 + data2;
    assign imag_res = data3 + data4;
    assign result = {real_res, imag_res};

    always@ (*) begin
        case(PSK)
            3'd0: begin
                data1 = temp3;
                data2 = res2;
                data3 = temp3;
                data4 = temp4;
            end

            3'd1: begin
                data1 = temp1;
                data2 = {WI{1'd0}};
                data3 = temp2;
                data4 = {WI{1'd0}};
            end

            3'd2: begin
                data1 = temp2;
                data2 = {WI{1'd0}};
                data3 = real_R;
                data4 = {WI{1'd0}};
            end

            3'd3: begin
                data1 = temp3;
                data2 = temp4;
                data3 = res1;
                data4 = temp4;
            end
            
            3'd4: begin
                data1 = imag_R;
                data2 = {WI{1'd0}};
                data3 = temp1;
                data4 = {WI{1'd0}};
            end
            
            3'd5: begin
                data1 = res1;
                data2 = res2;
                data3 = temp3;
                data4 = res2;
            end
            
            3'd6: begin
                data1 = res1;
                data2 = temp4;
                data3 = res1;
                data4 = res2;
            end
            
            3'd7: begin
                data1 = real_R;
                data2 = {WI{1'd0}};
                data3 = imag_R;
                data4 = {WI{1'd0}};
            end
            
            default: begin
                data1 = {WI{1'd0}};
                data2 = {WI{1'd0}};
                data3 = {WI{1'd0}};
                data4 = {WI{1'd0}};
            end
        endcase
    end

endmodule



