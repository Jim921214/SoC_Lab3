`timescale 1ns / 1ps

module fir
    #(  parameter pADDR_WIDTH = 12,
        parameter pDATA_WIDTH = 32,
        parameter tap_num     = 32 
     )
     (
         // AXI4-Lite for configuration
         // write channel
         output  wire                     awready,  // write address ready
         output  wire                     wready,   // write data    ready
         input   wire                     awvalid,  // write address valid
         input   wire [(pADDR_WIDTH-1):0] awaddr,   // write address
         input   wire                     wvalid,   // write data    valid
         input   wire [(pDATA_WIDTH-1):0] wdata,    // write data
         // read channel
         output  wire                     arready,  // read  address ready
         input   wire                     rready,   // read  data    ready
         input   wire                     arvalid,  // read  address valid
         input   wire [(pADDR_WIDTH-1):0] araddr,   // read  address
         output  wire                     rvalid,   // read  data    valid
         output  reg  [(pDATA_WIDTH-1):0] rdata,    // read  data

         // AXI4-Stream for x input
         input   wire                     ss_tvalid,
         input   wire [(pDATA_WIDTH-1):0] ss_tdata,
         input   wire                     ss_tlast,
         output  wire                     ss_tready,

         // AXI4-Stream for y output
         input   wire                     sm_tready,
         output  reg                      sm_tvalid,
         output  wire [(pDATA_WIDTH-1):0] sm_tdata,
         output  wire                     sm_tlast,

         // bram for tap RAM
         output  reg  [3:0]               tap_WE,
         output  reg                      tap_EN,
         output  reg  [(pDATA_WIDTH-1):0] tap_Di,
         output  reg  [(pADDR_WIDTH-1):0] tap_A,
         input   wire [(pDATA_WIDTH-1):0] tap_Do,

         // bram for data RAM
         output  reg  [3:0]               data_WE,
         output  reg                      data_EN,
         output  reg  [(pDATA_WIDTH-1):0] data_Di,
         output  reg  [(pADDR_WIDTH-1):0] data_A,
         input   wire [(pDATA_WIDTH-1):0] data_Do,
        
         input   wire                     axis_clk,
         input   wire                     axis_rst_n
     );

    wire [2:0] ap_ctrl;

    // AXI-Lite Write and Read
    reg awready_reg;
    reg wready_reg;
    reg arready_reg;
    reg rvalid_reg;
    reg [(pADDR_WIDTH-1):0] araddr_reg; // hold araddr until rvalid

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            arready_reg <= 0;
            rvalid_reg <= 0;
            awready_reg <= 0;
            wready_reg <= 0;
        end else begin
            arready_reg <= (arvalid & ~arready);
            rvalid_reg <= (arready | rvalid & ~rready);
            awready_reg <= (awvalid & wvalid);
            wready_reg <= (awvalid & wvalid);
        end
    end

    assign arready = arready_reg;
    assign rvalid = rvalid_reg;
    assign awready = awready_reg;
    assign wready = wready_reg;

    // ap_ctrl logic
    reg  [2:0] ap_state;
    reg  [2:0] ap_state_next;

    localparam  AP_PROC = 2'b00,
                AP_IDLE = 2'b01,
                AP_DONE = 2'b10;

    assign ap_ctrl[0] = (awaddr == 12'h00 && wdata[0] == 1'b1);
    assign ap_ctrl[1] = (ap_state == AP_DONE);
    assign ap_ctrl[2] = (ap_state == AP_IDLE);

    always @* begin
        case (ap_state)
            AP_IDLE:
            begin
                if(ap_ctrl[0]) begin
                    ap_state_next = AP_PROC;
                end else begin
                    ap_state_next = AP_IDLE;
                end
            end
            AP_PROC:
            begin
                if(sm_tlast) begin
                    ap_state_next = AP_DONE;
                end else begin
                    ap_state_next = AP_PROC;
                end
            end
            AP_DONE:
            begin
                if(araddr_reg == 12'd0 && rvalid) begin
                    ap_state_next = AP_IDLE;
                end else begin
                    ap_state_next = AP_DONE;
                end
            end
            default:
            begin
                if(ap_ctrl[0]) begin
                    ap_state_next = AP_PROC;
                end else begin
                    ap_state_next = AP_IDLE;
                end
            end
        endcase
    end

    always @(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n)
            ap_state <= AP_IDLE;
        else
            ap_state <= ap_state_next;
    end
// hold araddr until rvalid 
    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            araddr_reg <= 0;
        end else if(arready) begin
            araddr_reg <= araddr;
        end else begin
            araddr_reg <= araddr_reg;
        end
    end
    // write data length and tap_num
    // 0x10-14: data_length, 0x14-18: tap_num

    reg  [31:0] data_length_reg;
    reg  [31:0] tap_num_reg;

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            data_length_reg <= 0;
        end else if(ap_state == AP_IDLE && awaddr == 12'h10) begin
            data_length_reg <= wdata;
        end else begin
            data_length_reg <= data_length_reg;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            tap_num_reg <= 0;
        end else if(ap_state == AP_IDLE && awaddr == 12'h14) begin
            tap_num_reg <= wdata;
        end else begin
            tap_num_reg <= tap_num_reg;
        end
    end

    always@* begin
        if(araddr_reg == 12'h00) begin
            rdata = ap_ctrl;
        end else if(araddr_reg == 12'h10) begin
            rdata = data_length_reg;
        end else if(araddr_reg == 12'h14) begin
            rdata =  tap_num_reg;
        end else begin
            rdata = tap_Do;
        end
    end

    // Tap-RAM
    reg [5:0] tap_cnt;
    reg [4:0] x_w_cnt;
    reg [4:0] x_r_cnt;
    reg [8:0] y_cnt;

    always@* begin
        tap_EN = 1;
        tap_WE = (wready) && (awaddr[11:0] >= 12'h80 && awaddr[11:0] <= 12'hFF)? 4'b1111 : 4'b0000;

        if(ap_state == AP_PROC) begin
            tap_A = 4 * tap_cnt;
        end else if(awready) begin
            tap_A = awaddr[6:0];
        end else if(arready) begin
            tap_A = araddr[6:0];
        end else begin
            tap_A = tap_A;
        end

        tap_Di = wdata;
    end
    // FIR state
    reg  [2:0] fir_state;
    reg  [2:0] fir_state_next;

    localparam  FIR_CAL  = 3'b000,
                FIR_IDLE = 3'b001,
                FIR_LOAD = 3'b010,
                FIR_DONE = 3'b011,
                FIR_WAIT = 3'b100;

    always@* begin
        case (fir_state)
            FIR_IDLE:
            begin
                if(ap_state == AP_PROC && ss_tvalid) begin
                    fir_state_next = FIR_LOAD;
                end else begin
                    fir_state_next = FIR_IDLE;
                end
            end
            FIR_CAL:
            begin
                if ((y_cnt < tap_num_reg)? tap_cnt - 1 == y_cnt : tap_cnt == tap_num_reg + 1)begin
                    fir_state_next = FIR_WAIT;
                end else begin
                    fir_state_next = FIR_CAL;
                end
            end
            FIR_LOAD:
            begin
                if (ss_tvalid)begin
                    fir_state_next = FIR_CAL;
                end else begin
                    fir_state_next = FIR_LOAD;
                end
            end
            FIR_DONE:
            begin
                if(ap_state == AP_IDLE) begin
                    fir_state_next = FIR_IDLE;
                end else begin
                    fir_state_next = FIR_DONE;
                end
            end
            FIR_WAIT:
            begin
                if(y_cnt == data_length_reg && (sm_tready & sm_tvalid)) begin
                    fir_state_next = FIR_DONE;
                end else if(sm_tready & sm_tvalid) begin
                    fir_state_next = FIR_LOAD;
                end else begin
                    fir_state_next = FIR_WAIT;
                end
            end
        endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            fir_state <= FIR_IDLE;
        end else begin
            fir_state <= fir_state_next;
        end
    end


    // tap_cnt address generator

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            tap_cnt <= 0;
        end else if(fir_state != FIR_CAL) begin
            tap_cnt <= 0;
        end else begin
            tap_cnt <= (tap_cnt < tap_num_reg + 1) ? tap_cnt + 1 : 0;
        end
    end
/////////////////////////////////////////////////////////////////////////////////////////

    // x_w_cnt address generator

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            x_w_cnt <= 0;
        end else if(fir_state == FIR_IDLE) begin
            x_w_cnt <= 0;
        end else if(x_w_cnt == tap_num_reg) begin
            x_w_cnt <= 0;
        end else if(ss_tvalid & ss_tready)begin
            x_w_cnt <= x_w_cnt + 1;
        end else begin
            x_w_cnt <= x_w_cnt;
        end
    end

    // x_r_cnt address generator

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            x_r_cnt <= tap_num_reg - 1;
        end else if(fir_state == FIR_LOAD) begin
            x_r_cnt <= x_w_cnt;
        end else begin
            x_r_cnt <= (x_r_cnt == 0)? tap_num_reg - 1: (fir_state == FIR_CAL)? x_r_cnt - 1 : x_r_cnt;
        end
    end

    // y_cnt

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            y_cnt <= 9'd0;
        end else if(fir_state == FIR_DONE) begin
            y_cnt <= 9'd0;
        end else if(ss_tvalid & ss_tready) begin
            y_cnt <= y_cnt + 1;
        end else begin
            y_cnt <= y_cnt;
        end
    end

    
    // AXI-Stream for input X
    assign ss_tready = (fir_state == FIR_LOAD)? 1 : 0;

    // Data-RAM
    always@* begin
        data_EN = 1'b1;
        data_WE = (fir_state == FIR_LOAD) ? 4'b1111 : 4'b0000; 
        data_Di = ss_tdata;
        data_A =  (fir_state == FIR_LOAD) ? 4 * x_w_cnt : 4 * x_r_cnt;
    end

    wire [pDATA_WIDTH-1:0] x;
    wire [pDATA_WIDTH-1:0] h;
    reg  [pDATA_WIDTH-1:0] y;

    assign x = data_Do;
    assign h = tap_Do;

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            y <= 0;
        end else if(fir_state == FIR_IDLE) begin
            y <= 0;
        end else if(tap_cnt == 1) begin
            y <= h * x;
        end else if(tap_cnt <= 32) begin
            y <= h * x + y;
        end else begin
            y <= y;
        end
    end

    reg  [pDATA_WIDTH-1:0] y_ans;
    
    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            y_ans <= 0;
        end else if(fir_state == FIR_IDLE) begin
            y_ans <= 0;
        end else if((y_cnt < tap_num_reg)? tap_cnt - 1 == y_cnt : tap_cnt == tap_num_reg + 1) begin
            y_ans <= y;
        end else begin
            y_ans <= y_ans;
        end
    end

    // AXI-Stream for output y
    assign sm_tlast = (fir_state == FIR_DONE);
    assign sm_tdata = y_ans;

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n) begin
            sm_tvalid <= 1'b0;
        end else if(fir_state != FIR_CAL && fir_state != FIR_WAIT) begin
            sm_tvalid <= 1'b0;
        end else if((y_cnt < tap_num_reg)? tap_cnt - 1 == y_cnt : tap_cnt == tap_num_reg + 1) begin
            sm_tvalid <= 1'b1;
        end else if(sm_tready) begin       
            sm_tvalid <= 1'b0;
        end else begin
            sm_tvalid <= sm_tvalid;
        end
    end  

endmodule