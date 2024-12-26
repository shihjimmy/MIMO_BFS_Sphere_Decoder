`timescale 1ns / 1ps
`define CLK_period 13.0 
`define MAX_CYCLE 1000000     
`define R_MATRIX "./pattern/channel.dat"  
`define Y_VECTOR "./pattern/z.dat"        
`define GOLDEN "./pattern/golden.dat"     
`define DISTANCE "./pattern/distance.dat" 

module tb;
    parameter WI = 10;  
    parameter num_of_symbols = 11;

    reg Clk;
    reg Reset;
    reg flagChannelorData;
    reg in_valid;
    reg [(4*2*WI)-1:0] InData;
    wire [11:0] OutData;
    wire out_valid;

    sphere_decoder #(
        .WI(10)
    ) dut (
        .Clk(Clk),
        .Reset(Reset),
        .in_valid(in_valid),
        .flagChannelorData(flagChannelorData),
        .InData(InData),
        .OutData(OutData),
        .out_valid(out_valid)
    );

    reg [(2*WI)-1:0] R_MATRIX [0:11];
    reg [(2*WI)-1:0] Y_VECTOR [0 : (num_of_symbols*4)-1];
    reg [11:0] golden [0 : (num_of_symbols-1)];
    reg [9:0] golden_distance [0 : (num_of_symbols-1)];


    initial begin
        Clk = 1;
        forever #(`CLK_period/2) Clk = ~Clk; 
    end

    initial begin
        $readmemb(`R_MATRIX, R_MATRIX);
        $readmemb(`Y_VECTOR, Y_VECTOR);
        $readmemb(`GOLDEN, golden);
        $readmemb(`DISTANCE, golden_distance);

        // $fsdbDumpfile("sphere_decoder.fsdb");
        // $fsdbDumpvars;
        // $fsdbDumpMDA;
    end

    reg start;
    integer count;
    integer error;
    integer cycle;
    integer R_finish;
    reg temp;

    initial begin
        Reset = 0;
        start = 0;
        count = 0;
        error = 0;
        temp = 0;
        in_valid = 0;
        cycle = 0;
        R_finish = 0;
        flagChannelorData = 0;

        #(`CLK_period*9.5) 
        Reset = 1;
        
        #(`CLK_period*1)
        Reset = 0;
        start = 1; 
    end

    always @(posedge Clk) begin
        cycle <= cycle + 1;

        if(cycle > `MAX_CYCLE) begin
            $display("time is over");
            $display("\n");
            $finish;
        end
    end

    always @(negedge Clk) begin
        if(start) begin
            in_valid <= 1;

            if(R_finish) begin
                flagChannelorData <= 0;
                InData <= { Y_VECTOR[4*count + 3], Y_VECTOR[4*count + 2], Y_VECTOR[4*count + 1], Y_VECTOR[4*count] };
                start <= 0;
            end
            else if(count == 0 || count == 1) begin
                flagChannelorData <= 1;
                InData <= { R_MATRIX[4*count+3], R_MATRIX[4*count+2], R_MATRIX[4*count+1], R_MATRIX[4*count] };   
                count <= count + 1;       
            end
            else if(count == 2) begin
                flagChannelorData = 1;
                InData <= { { (4*WI){1'd0} }, R_MATRIX[9], R_MATRIX[8] };
                count <= 0;
                R_finish <= 1;
            end
        end
        else begin
            in_valid <= 0;
        end
    end


    always@ (negedge Clk) begin
        if(count == num_of_symbols) begin
            if(error==0) $display("All test patterns are correct, congratulations");
            else $display("There are %d errors!", error);
            $display("\n");
            $finish;
        end
    end 

    always @(negedge Clk) begin 
        if(out_valid) begin

            in_valid <= 1;
            flagChannelorData <= 0;
            InData <= { Y_VECTOR[4*(count+1) + 3], Y_VECTOR[4*(count+1) + 2], Y_VECTOR[4*(count+1) + 1], Y_VECTOR[4*(count+1)] };

            if(golden[count] !== OutData) begin
                $display("You got a wrong answer");
                $display("Golden is: %b", golden[count]);
                $display("Your answer is: %b", OutData);
                $display("\n");
                error <= error + 1;
                count <= count + 1;
            end
            else begin
                $display("Your answer is correct: %b", OutData);
                $display("Golden is: %b", golden[count]);
                $display("Your answer is: %b", OutData);

                // 拿掉下面兩行註解測試distance的運算結果，但只能在behavior simulation執行。
                // $display("Here is the Euclidean distance from the golden detected symbol: %b", golden_distance[count]);
                // $display("Here is your Euclidean distance: %b", dut.k_prev_r[0][0 +: WI]);

                $display("Here is your detected symbol: ");
                count <= count + 1;

                case(OutData[9+:3])
                    3'd0: $display("The detected symbol of x1 is: -0.707 -j0.707");
                    3'd1: $display("The detected symbol of x1 is: -1 + j0");
                    3'd2: $display("The detected symbol of x1 is: 0 + j1");
                    3'd3: $display("The detected symbol of x1 is: -0.707 + j0.707");
                    3'd4: $display("The detected symbol of x1 is: 0 - j1");
                    3'd5: $display("The detected symbol of x1 is: 0.707 - j0.707");
                    3'd6: $display("The detected symbol of x1 is: 0.707 + j0.707");
                    3'd7: $display("The detected symbol of x1 is: 1 + j0");
                endcase


                case(OutData[6+:3])
                    3'd0: $display("The detected symbol of x2 is: -0.707 -j0.707");
                    3'd1: $display("The detected symbol of x2 is: -1 + j0");
                    3'd2: $display("The detected symbol of x2 is: 0 + j1");
                    3'd3: $display("The detected symbol of x2 is: -0.707 + j0.707");
                    3'd4: $display("The detected symbol of x2 is: 0 - j1");
                    3'd5: $display("The detected symbol of x2 is: 0.707 - j0.707");
                    3'd6: $display("The detected symbol of x2 is: 0.707 + j0.707");
                    3'd7: $display("The detected symbol of x2 is: 1 + j0");
                endcase

                case(OutData[3+:3])
                    3'd0: $display("The detected symbol of x3 is: -0.707 -j0.707");
                    3'd1: $display("The detected symbol of x3 is: -1 + j0");
                    3'd2: $display("The detected symbol of x3 is: 0 + j1");
                    3'd3: $display("The detected symbol of x3 is: -0.707 + j0.707");
                    3'd4: $display("The detected symbol of x3 is: 0 - j1");
                    3'd5: $display("The detected symbol of x3 is: 0.707 - j0.707");
                    3'd6: $display("The detected symbol of x3 is: 0.707 + j0.707");
                    3'd7: $display("The detected symbol of x3 is: 1 + j0");
                endcase

                case(OutData[0+:3])
                    3'd0: $display("The detected symbol of x4 is: -0.707 -j0.707");
                    3'd1: $display("The detected symbol of x4 is: -1 + j0");
                    3'd2: $display("The detected symbol of x4 is: 0 + j1");
                    3'd3: $display("The detected symbol of x4 is: -0.707 + j0.707");
                    3'd4: $display("The detected symbol of x4 is: 0 - j1");
                    3'd5: $display("The detected symbol of x4 is: 0.707 - j0.707");
                    3'd6: $display("The detected symbol of x4 is: 0.707 + j0.707");
                    3'd7: $display("The detected symbol of x4 is: 1 + j0");
                endcase

                $display("\n");
            end
        end
    end

endmodule
