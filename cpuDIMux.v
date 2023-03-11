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
*       Ver 0.45 3/10/23    Fixed to allow other S100 I/O boards    *
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
    input   z80Read,
    input   pll0_250MHz,
    output reg [7:0] outData
    );

always @(posedge pll0_250MHz) begin
    if (rom_cs)
        outData <= romData;
    else if (reset_cs)
        outData <= 8'h00;

        else if (ide_cs)
        outData <= s100DataIn;
    else if(inPortcon_cs)
        outData <= s100DataIn;
    else if (ram_cs)
        outData <= ramaData;
    else if (inLED_cs)
        outData <= ledread;
    else if (iobyteIn_cs)
        outData <= iobyte;
    else if (usbRxD_cs)
        outData <= usbRxD;
    else if(usbStat_cs)
        outData <= usbStatus;
    else if (z80Read)
        outData <= s100DataIn;
    end
    
endmodule
