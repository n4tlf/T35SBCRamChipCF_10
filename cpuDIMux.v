/********************************************************************
*   FILE:   cpuDIMux.v      Ver 0.4         Oct. 12, 2022           *
*                                                                   *
*   This function selects which device's DATA OUT goes into the Z80 *
*   CPU DATA INPUT bus at any given time. It uses the device's      *
*   select line to enable that device's DATA OUT signals            *
*   The Efinix FPGAs do NOT have internal tri-state buffering       *
*   capabilities, so this is especially important to prevent        *
*   clashing of signals of the Z80 DATA INPUT bus.                  *
*   TFOX, N4TLF January 20, 2023   You are free to use it           *
*       however you like.  No warranty expressed or implied         *
*       Ver 0.3 1/30/23:  Added pll0_250MHz clocking                *
*       Ver 0.4 2/14/23:  Added/fixed IDE selection (project 10)    *
*                           and fixed = vs <= in else if            *
********************************************************************/

module cpuDIMux
    (
    input [7:0] romData,
    input [7:0] ramaData,
    input [7:0] s100DataIn,
    input [7:0] ledread,
    input [7:0] iobyte,
    input [7:0] usbRxD,
    input [7:0] usbStatus,
    input   reset_cs,
    input   rom_cs,
    input   ram_cs,
    input   inPortcon_cs,
    input   inLED_cs,
    input   iobyteIn_cs,
    input   usbStat_cs,
    input   usbRxD_cs,
    input   ide_cs,
    input   pll0_250MHz,
    output wire [7:0] outData
    );
    
reg [7:0] selectedData;

always @(posedge pll0_250MHz) begin
    if (rom_cs)
        selectedData <= romData;
   else if (ide_cs)
        selectedData <= s100DataIn;
    else if(inPortcon_cs)
        selectedData <= s100DataIn;
    else if (ram_cs)
        selectedData <= ramaData;
    else if (inLED_cs)
        selectedData <= ledread;
    else if (iobyteIn_cs)
        selectedData <= iobyte;
     else if (usbRxD_cs)
        selectedData <= usbRxD;
     else if(usbStat_cs)
        selectedData <= usbStatus;
   else if (reset_cs)
        selectedData <= 8'h00;   // execute a NOP for now
    end
    
    assign outData = selectedData;
    
endmodule
