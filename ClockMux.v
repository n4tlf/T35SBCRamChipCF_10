/********************************************************************************
*       ClockMux.v.     TFOX        Updated version 0.5     Jan 20, 2023        *
*           TFOX, N4TLF January 31, 2023   You are free to use it               *
*           however you like.  No warranty expressed or implied                 *
*   Clock speed INPUT Multiplexer.  Uses a much higher clock to drive posedge   *
*       selection.  Switches 7 & 6 select cpu clock speed (which is then        *
*       divided by six in microcontroller.vhdl.                                 *
*       7 & 6 = 11 (DN DN) = 2MHz       7 & 6 = 01 (UP DN) = 25MHz              *
*       7 & 6 = 10 (DN UP) = 31kHz      7 & 6 = 00 (UP UP) = 250Hz              *
********************************************************************************/

module  ClockMux(
    input           MHz2,
    input           MHz25,
    input           KHz31,
    input           Hz250,
    input           pll0_250MHz,
    input   [1:0]   sw, 
    output  reg    cpuclk
    );

always @(posedge pll0_250MHz) begin
    case(sw)
        2'b00:      cpuclk = Hz250;
        2'b01:      cpuclk = MHz25;
        2'b10:      cpuclk = KHz31;
        default:    cpuclk = MHz2;   
     endcase   

     end
    
endmodule
    
