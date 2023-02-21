/************************************************************************
*   File:  portDecoder.v    TFOX    Ver 0.3     Oct.12, 2022            *
*       This module creates I/O Port _cs (chip selects) (active HIGH)   *
*       based on addresses A7-A0 only.                                  *
*   TFOX, N4TLF January 20, 2023   You are free to use it               *
*       however you like.  No warranty expressed or implied             *
*   Feb 12, 2023: Ver 0.5, added and fixed IDE Ports for 8255           *
************************************************************************/

module portDecoder
    (
    input [7:0]     address,
    input           iowrite,            //sOUT signal
    input           ioread,             // sINP signal
    //
    output          outPortFF_cs,
    output          inPortCon_cs,
    output          outFbarLEDs_cs,
    output          inFbarLEDs_cs,
    output          outMiscCtl_cs,
    output          inIOBYTE_cs,
    output          outRAMA16_cs, 
    output          inUSBst_cs,
    output          inusbRxD_cs,
    output          outusbTxD_cs,
    output          idePorts8255_cs
   );

    assign outPortFF_cs = (address[7:0] == 8'hff) && iowrite;
    assign inPortCon_cs = (address[7:1] == 7'h00) && ioread;
    assign outFbarLEDs_cs = (address[7:0] == 8'h06) && iowrite; // Port 06 write
    assign inFbarLEDs_cs = (address[7:0] == 8'h06) && ioread;  // port 06 read
    assign outMiscCtl_cs = (address[7:0] == 8'h07) && iowrite; // Port 07 out
    assign inIOBYTE_cs = (address[7:0] == 8'h36) && ioread;    // IN = IOBYTE(switches)
    assign outRAMA16_cs = (address[7:0] == 8'h36) && iowrite;  // Out 36 D0 = RAM A16
    assign inUSBst_cs   = (address[7:0] == 8'h34) && ioread;  // input USB status port
    assign inusbRxD_cs  = (address[7:0] == 8'h35) && ioread;  // USB UART Rx Data input
    assign outusbTxD_cs = (address[7:0] == 8'h35) && iowrite; // USB UART Tx Data output
    assign idePorts8255_cs = (address[7:2] == 6'b001100 & (ioread | iowrite));
    
    endmodule
