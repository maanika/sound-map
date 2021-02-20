/* ============================================================================
 * File Name: DDS24_Core_v0_0.v
 * Version 0.0 (build 17.01)
 * 
 * Description:
 *   DDS24: 24-bit DDS frequency generator component (v 0.0)
 *   Produces two syncronous fractional frequency outputs (out1 and out2)
 *   Outputs out1 and out2 separated by variable 8-bit phase shift [0..255]. 
 *
 * Credits:
 *   based on original IQ_DDS code by <PSoC73>:
 *   http://www.cypress.com/?app=forum&id=2492&rID=88149
 * 
 *   Special thanks to <pavloven>, <vdvorak>, <kabron>
 *
 * Note:
 *   frequency resolution = clock_frequency / 2^24
 *   frequency maximum    = clock_frequency / 2
 *   frequency minimum    = clock_frequency / 2^24
 *
 * ============================================================================
 * PROVIDED AS-IS, NO WARRANTY OF ANY KIND, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * FREE TO SHARE, USE AND MODIFY UNDER TERMS: CREATIVE COMMONS - SHARE ALIKE
 * ============================================================================
*/

//`#start header` -- edit after this line, do not edit this line
`include "cypress.v"
`ifdef DDS24_Core_v0_0_V_ALREADY_INCLUDED
`else
`define DDS24_Core_v0_0_V_ALREADY_INCLUDED
//`#end` -- edit above this line, do not edit this line
// Generated on 10/27/2015 at 22:13
// Component: DDS24_Core_v0_0

module DDS24_Core_v0_0( load, en, ctrl, phase, clock, out1, out2, rdy1, rdy2, rfd );//

  // out1, out2  parameters //
  
  parameter   out1_width = 8'd8;    // out1 bus width (variable)
  parameter   out2_width = 8'd8;    // out2 bus width (variable)
  localparam  Hi1 = out1_width-1;   // out1 high bit index
  localparam  Hi2 = out2_width-1;   // out2 high bit index

  // Inputs & outputs //

  input  wire           load;	    // load data
  input  wire           en;         // DDS enable
  input  wire [22:0]    ctrl;       // DDS hw control bus  
  input  wire [7:0]     phase;      // DDS hw phase bus  
  input  wire           clock;      // input clock 
  output wire [Hi1:0]   out1;       // out1 bus
  output wire [Hi2:0]   out2;       // out2 bus
  output wire           rdy1;       // out1 data ready
  output wire           rdy2;       // out2 data ready
  output wire           rfd;        // ready for new data (phase or shift) from API
  
  
  
  // Parameters // 
  
  parameter out2_enable   = 1'b0;   // out2 output enable 
  parameter hw_load       = 1'b0;   // strobe on load (1) API update (0)
  parameter tune_word_h   = 23'd1;  // hardcoded tune word 
  parameter out2_phase_h  = 8'd0;   // hardcoded phase shift 
  
  localparam CTRL_API     = 2'b00;  // API call
  localparam CTRL_HARDC   = 2'b01;  // Hardcoded
  localparam CTRL_BUS     = 2'd10;  // Digital bus
  parameter [1:0] ControlFreq  = CTRL_API; // control frequency by: API csll | Hardcoded | Digital bus
  parameter [1:0] ControlPhase = CTRL_API; // control phase by:     API call | Hardcoded | Digital bus   
  
  localparam ENMODE_AUTO   = 2'b00;
  localparam ENMODE_CRONLY = 2'b01;
  localparam ENMODE_HWONLY = 2'b10;
  localparam ENMODE_CR_HW  = 2'b11;
  parameter [1:0] EnableMode = ENMODE_CRONLY; // auto enable | software only | hardware only | HW and SW 
  
  
  
  // no connect wires //
  
  wire [22:0] tune_word;            // Control Register output phase increment
  wire [7:0]  controlD;             // Control Register Output Phase shift 
  wire        ClockOutFromEnBlock;  // main clock for DDS core - 


//============================================================================== 
//                             DDS Accumulator
//
//          produce single or dual outputs with phase shift
//============================================================================== 

 
    reg  [23:0]   Acc;              // accumulator
    reg  [23:16]  Ac1;              // accumulator1 helper 
    reg  [23:16]  Ac2;	            // 8-bit helper phase shifted from Acc 0-360
   
    generate                                        // select single or dual output
    if(out2_enable==1'b1)                           // dual phase output
    begin 
        always @(posedge ClockOutFromEnBlock) begin	
            Acc <= Acc+tune_word;		            // accumulator
            Ac1 <= Acc[23:16];                      // add 1-clock delay to match Ac2 register
            Ac2 <= Acc[23:16]+controlD;	            // 0-360 deg, (approx.?)
        end 

        assign out1 = Ac1[23:(23-Hi1)];		        // get out I-bus as top core_out_width wires
        assign out2 = Ac2[23:(23-Hi2)];	            // out2 output is wire - phase shifted        
    end
    else begin                                      //single phase output
        always @(posedge ClockOutFromEnBlock) begin	
            Acc <= Acc+tune_word;		            // accumulator
        end  
        assign out1 = Acc[23:(23-Hi1)];		        // DDS output 1 (wire)        
    end
    endgenerate

     
     
//============================================================================== 
//              Data ready: sync outputs for bus out1 and out2  
//
//  produce a single pulse of high polarity 1/2 clock length and 1/2 clock delay
//============================================================================== 

    reg [Hi1:0] out1_old;   //out1 previous value
    reg [Hi2:0] out2_old;   //out2 previous value
    
    generate
    if(out2_enable==1'b1)                           // dual phase output
    begin 
        always @(posedge ClockOutFromEnBlock) 	   
        begin
           out1_old <= out1;
           out2_old <= out2;
        end
        assign rdy1 = |(out1 ^ out1_old) & ~clock;  //pos edge:  delay 1/2 clock, width 1/2 clock
        assign rdy2 = |(out2 ^ out2_old) & ~clock;  //pos edge:  delay 1/2 clock, width 1/2 clock
    end
    else begin                                      //single phase output
        always @(posedge ClockOutFromEnBlock) 	    
        begin
           out1_old <= out1;
        end
        assign rdy1 = |(out1 ^ out1_old) & ~clock;  //pos edge:  delay 1/2 clock, width 1/2 clock        
    end
    endgenerate
    

//==============================================================================
//    Enable control (Auto, Software, Hardware, Software & Hardware)  
//
//  Uses UDB_clock_enable to enable DDS output (ClockOutFromEnBlock = clock & en)
//  Uses control register (CR) for API control when Enable Mode = SW or SW_HW
//==============================================================================
    
    
    wire clock_en;                                          // temp 
    
    //cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(`TRUE))
    cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(`FALSE))    //
    clock_enable_block (
                        .clock_out(ClockOutFromEnBlock),    // output clock
                        .clock_in(clock),                   // input clock
                        .enable(clock_en)                   // input enable     
    );
    
                                                     
    localparam CLOCK_ENABLE_BIT   = 1'd0;                   // reserve bit 0 for clock enable                                                
    wire [7:0] ctrl8;                                       // Control Register bit location (bits 7-1 are unused)
    
    generate
    if( (EnableMode == ENMODE_CRONLY) || (EnableMode == ENMODE_CR_HW) ) begin : sCTRLReg 
        cy_psoc3_control #(.cy_force_order(`TRUE))          //Default mode
            ctrlreg( .control(ctrl8) );
    end
    endgenerate
    
    wire control_enable; assign control_enable = ctrl8[CLOCK_ENABLE_BIT];

    assign clock_en = (EnableMode == ENMODE_AUTO)?   1'b1:                  // auto enable
                      (EnableMode == ENMODE_CRONLY)? control_enable:        // software_only  
                      (EnableMode == ENMODE_CR_HW)? (control_enable & en):  // software_and_hardware
                                                     en;                    // hardware_only
    
    
    
//==============================================================================
//                              Ready for data   
//  
//  goes high when acomponent is ready to update control phase or delay registers
//==============================================================================
   
    
    //this realization consumes 1 status cell and makes 1.5-2 clock delay
    generate                                    // resource does not removed automatically
    if (hw_load==1'b1) begin                    // hardware load = true, rfd is visible
        wire s_load;                            // synched load
        cy_psoc3_sync HWLoadSync (              // consumes 1 status cell to make DFF
                .clock	(ClockOutFromEnBlock),
    			.sc_in	(load),                 //  
	    		.sc_out	(s_load));              //  
                         
        assign rfd = s_load;                    // delay 1 full clock
    end
    endgenerate

    
    //this relaizes instant alternative way with no delays, but async output
    //assign rfd = clock_en && load;            // instant async update 
                                    
                                   
                                    
//==============================================================================
//              DDS control source: API / Hardcoded / HW bus
//
// use API, Hardcoded value or Digital bus to control DDS frequency and phase
//==============================================================================

    
    wire [23:0]  control_cr;                    // Control Register output phase increment
    wire [7:0]   controlD_cr;                   // Control Register Output Phase shift 


    // 3:1 frequency mux: API | Hardcoded | Digital bus
    generate  
    if (ControlFreq==CTRL_API)   assign tune_word = control_cr[22:0];  else // API 
    if (ControlFreq==CTRL_HARDC) assign tune_word = tune_word_h;       else // Hardcoded 
    //if (ControlFreq==CTRL_BUS)                                                 
        if (hw_load==1'd0)
            assign tune_word = ctrl[22:0];                                  // bus direct
        else begin
            reg[22:0] r_ctrl; always @(posedge load) r_ctrl <= ctrl[22:0];  // DFF emulation for bus load
            assign tune_word = r_ctrl;                                      // bus registered on load 
        end
    endgenerate 
    

    // 3:1 phase mux: API | Hardcoded | Digital bus
    generate  
    if (ControlPhase==CTRL_API)   assign controlD = controlD_cr;       else // API 
    if (ControlPhase==CTRL_HARDC) assign controlD = out2_phase_h;      else // Hardcocded
    //if (ControlPhase==CTRL_BUS)                                                    
        if (hw_load==1'd0)
            assign controlD = phase;                                        // bus direct
        else begin
            reg[7:0] r_phase; always @(posedge load) r_phase <= phase;      // DFF emulation for bus load
            assign controlD = r_phase;                                      // bus registered on load 
        end
    endgenerate 
    
    

//==============================================================================
//          API control registers to set phase increment and shift
//
//  tune_word = {ControlPhaseC, ControlPhaseB, ControlPhaseA}
//  where ControlPhaseA, ControlPhaseB, ControlPhaseC - are Lo, Me, Hi bits
//
//  options: (i) direct API update or (ii) buffered with hardware strobe
//==============================================================================


    generate                                
    if (hw_load==1'b0) begin: sControl                                          // direct API update
 
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE))  // default mode
            ControlPhaseA(
                .control(control_cr[7:0]));                                     // output Lo bits [07:00]

            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE))  // default mode
            ControlPhaseB(
                .control(control_cr[15:8]));                                    // output Md bits [15:08]
    
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE))  // default mode
            ControlPhaseC(
                .control(control_cr[23:16]));                                   // output Hi bits [23:16]
               
            // ControlPhaseD is 8-bit PhaseShift
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE))  // default mode
            ControlPhaseD(
                .control(controlD_cr));                                         // output PhaseShift [07:00] 
        
    end
    else begin: sControl                                                        // buffered update on hardware load 

            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE), .cy_ctrl_mode_1(8'h00), .cy_ctrl_mode_0(8'hFF)) //Sync mode
            ControlPhaseA(
                .control(control_cr[7:0]),                                      // output Lo bits [07:00]
                .clock(load)            );                                      // hw load 
            
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE), .cy_ctrl_mode_1(8'h00), .cy_ctrl_mode_0(8'hFF)) //Sync mode
            ControlPhaseB(
                .control(control_cr[15:8]),                                     // output Md bits [15:08]
                .clock(load)             );                                     // hw load 
    
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE), .cy_ctrl_mode_1(8'h00), .cy_ctrl_mode_0(8'hFF)) //Sync mode
            ControlPhaseC(
                .control(control_cr[23:16]),                                    // output Hi bits [23:16]
                .clock(load)              );                                    // hw load
               
            // ControlPhaseD is 8-bit PhaseShift
            cy_psoc3_control #(.cy_init_value (8'h00), .cy_force_order(`TRUE), .cy_ctrl_mode_1(8'h00), .cy_ctrl_mode_0(8'hFF)) //Sync mode
            ControlPhaseD(
                .control(controlD_cr),                                          // output PhaseShift [07:00] 
                .clock(load)        );                                          // hw load

    end
    endgenerate
   

//==============================================================================




endmodule
`endif /* DDS24_Core_v0_0_V_ALREADY_INCLUDED */

////////////////////////////////////////////////////////////////////////////////
//`#end` -- edit above this line, do not edit this line
//`#start footer` -- edit after this line, do not edit this line
//`#end` -- edit above this line, do not edit this line
