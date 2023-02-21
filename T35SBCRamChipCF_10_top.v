/************************************************************************
*   FILE:  T35SBCRamChipCF_10_top.v                                   	*
*                                                                     	*
*	This project adds IDE and Compact Flash support.					*
*		Work on several "fixes" were also created here first, before	*
*		being applied to several earlier projects						*
*   TFOX, N4TLF January 20, 2023   You are free to use it             	*
*       however you like.  No warranty expressed or implied           	*
***********************************************************************/

module  T35SBCRamChipCF_10_top(
    clockIn,            // 50MHz input from onboard oscillator
    pll0_LOCKED,
    pll0_2MHz,
    pll0_25MHz,         // NOTE:  This also drives SBC_LEDs mux to 
                        // eliminate a build error
    pll0_250MHz,
                        // Next comes all the S100 bus signals
    s100_n_RESET,       // on SBC board reset button (GPIOT_RXP12)
    s100_DI,            // S100 Data In bus
    s100_xrdy,          // xrdy is S100 pin 3, on Mini Front Panel
                        // and Monahan Bus DIsplay Board (BDB)
    s100_rdy,           // second Ready signal, S100 pin
    s100_HOLD,
    //
    S100adr0_15,
    S100adr16_19,
    s100_DO,            // S100 SBC Data Out bus
   
    s100_pDBIN,
    s100_pSYNC,   
    s100_pSTVAL,
    s100_n_pWR,
    s100_sMWRT,
    s100_pHLDA,
    s100_PHI,
    s100_CLOCK,         // 2MHz Clock signal to S100 bus    
    s100_sHLTA,
    s100_sINTA,
    s100_n_sWO,
    s100_sMEMR,
    s100_sINP,
    s100_sOUT,
    s100_sM1,
    s100_PHANTOM,       // turn OFF Phantom LED on Front panels
    s100_ADSB,          // turn OFF these (ADSB & SDSB) LEDs on BDB
    s100_CDSB,          // turn OFF these LEDs on BDB
    
                    // Some of the SBC non-S100 output signals
    SBC_LEDs,           // The SBC LEDs for testing
    sw_IOBYTE,          // I/O Byte Switches  NOT USED AS Z80 IOBYTE HERE!
    outPrn,             // Printer Port output
    outPrnStrobe,
    out8255_n_cs,
    idePorts_n_rd,
    idePorts_n_wr,
    outIDE_n_rd,
    outIDE_n_wr,
    cpuClkOut_P19,
    spare_P1,
    spare_P17,
    spare_P32,
    spare_P33,

    ram_A16,
    ram_A17,
    ram_A18,
    ram_n_cs,
    ram_n_oe,
    ram_n_wr,
    biData_IN,
    biData_OUT,
    biData_OE,
    
    seg7,
    seg7_dp,
    boardActive,
    buzzer,
    diagLED,
    highRomLED,
    lowRomLED,
    highRamLED,

    s100PhantomLED,
    prnAckLED,
    usbTXbusyLED,
    usbRXbusyLED,
    usbRXData,
    usbTXData,
    usbCTS,
    usbDTR, 

    F_in_cdsb,
    F_in_sdsb,    
    F_add_oe,
    F_bus_stat_oe,
    F_out_DO_oe,
    F_out_DI_oe,
    F_bus_ctl_oe);
        
    input   clockIn;
    input   pll0_LOCKED;
    input   pll0_2MHz;
    input   pll0_25MHz;
    input   pll0_250MHz;
    input   usbRXData;
    input   usbDTR;
    input   [7:0] sw_IOBYTE;
    input   s100_n_RESET;
    input   s100_xrdy;
    input   s100_rdy;
    input   s100_HOLD;
    input   [7:0] s100_DI;
    input   F_in_cdsb;
    input   F_in_sdsb;    
    output  [15:0]S100adr0_15;
    output  [3:0] S100adr16_19;
    output  s100_pDBIN;
    output  s100_pSYNC; 
    output  s100_pSTVAL;
    output  s100_n_pWR;
    output  s100_sMWRT;
    output  s100_pHLDA;
    output  [7:0] s100_DO;
    output  s100_PHI;
    output  s100_CLOCK;
    output  s100_sHLTA;
    output  s100_sINTA;
    output  s100_n_sWO;
    output  s100_sMEMR;
    output  s100_sINP;
    output  s100_sOUT;
    output  s100_sM1;
    //
    output  [7:0] SBC_LEDs;
    output  [7:0] outPrn;
    output  outPrnStrobe;
    output  out8255_n_cs;       // output to 8255 chip select (pin 6)
    output  idePorts_n_rd;      // U27 pins 1&19 buffer 8255->DI bus
    output  idePorts_n_wr;      // U24 pins 1&19 buffer DO bus->8255
    output  outIDE_n_rd;        // to 8255 pin 5 (rd*)
    output  outIDE_n_wr;        // to 8255 pin 36 (wr*)
    output  cpuClkOut_P19;
    output  spare_P1;
    output  spare_P17;
    output  spare_P32;
    output  spare_P33;
    
    output  [6:0] seg7;
    output  seg7_dp;
    output  s100_PHANTOM;       // turn OFF phantom light
    output  s100_ADSB;          // turn OFF these LEDs (ADSB & SDSB) on BDB
    output  s100_CDSB;          // turn OFF these LEDs on BDB
    output  ram_A16;
    output  ram_A17;
    output  ram_A18;
    output  ram_n_cs;
    output  ram_n_oe;
    output  ram_n_wr;
    
    input   [7:0] biData_IN;
    output  [7:0] biData_OUT;
    output  [7:0] biData_OE;
    
    output  boardActive;
    output  buzzer;
    output  diagLED;
    output  highRomLED;
    output  lowRomLED;
    output  highRamLED;
    output  s100PhantomLED;
    output  prnAckLED;
    output  usbTXbusyLED;
    output  usbRXbusyLED;
    output  usbTXData;
    output  usbCTS;
    
    output  F_add_oe;
    output  F_bus_stat_oe;
    output  F_out_DO_oe;
    output  F_out_DI_oe;
    output  F_bus_ctl_oe;
    
///////////////////////////////////////////////////////////////////
    parameter   romWaitvalue = 8'hFF;
    parameter   ioWaitvalue = 8'hFF;

    wire    cpuClock;
    wire    Clkcpu;
    wire    z80_n_m1;
    wire    z80_n_mreq;
    wire    z80_n_iorq;
    wire    z80_n_rd;
    wire    z80_n_wr;
    wire    z80_n_rfsh;
    wire    z80_n_halt;
    wire    z80_n_busak;
    wire    z80_n_wait;
    
    wire    [15:0]  cpuAddress;
    wire    [7:0]   cpuDataOut;
    wire    [7:0]   cpuDataIn;
    wire    [7:0]   romOut;
    wire    [7:0]   out255;
    wire    [7:0]   sw_IOBYTE;
    wire    [7:0]   debugReg;
    wire    [7:0]   fbarSbcLeds;    // out/in port 06, BAR LEDs
    wire    [7:0]   miscControl;    // Out port 07, Misc control
    wire    [7:0]   outramA16;       // IN=IOBYTE switch, Out (d0)=RAM A16
    wire    [7:0]   usbStat;        // USB UART Input Status port
    wire    [7:0]   usbRxData;      // USB UART received data
    wire    [7:0]   usbTxData;      // USB UART Transmit Data
    wire    [7:0]   usbTxDelay;
    wire    n_reset;
    wire    n_resetLatch;
    wire    biOutEN;            // Bidirectional Bus OUTPUT ENABLE
    wire    romWait;
    wire    ioWait;
    wire    boardWait;
    wire    z80mreq;
    wire    memRD;        // Memory READ signal
    wire    memWR;
    wire    outFF;
    wire    inPortCON_cs;
    wire    rom_cs;
    wire    ram_cs;
    wire    nop_cs;
    wire    miscCtl_cs;
    wire    inBarLED_cs;
    wire    outBarLED_cs;
    wire    iniobyte_cs;
    wire    inusbStat_cs;       // USB UART Status input select
    wire    inusbRxData_cs;     // USB UART Rx Data input select
    wire    outusbTxData_cs;    // USB UART Tx Data output select
    
    wire    outrama16_cs;
    wire    n_inta;             // internal INTA signal
    wire    inta;               // TEMPorary POSITIVE INTA 
    wire    mrq_norfsh;         // memory request, NOT refesh
    wire    sWO;                // combined write out
    wire    sOUT;
    wire    sINP;
    wire    n_mwr;
    wire    liorq;              // latched iorq
    wire    psyncstrt;          // start of psync signal
    wire    psyncend;           // end of psync
    wire    endsync;
    wire    busin;              // active high bus in for S100
    wire    pstval;             // ps trobe value
//    wire    pdbin;              // pDBIN FF output
    wire    psync;              // S100 pSync
    wire    io_output;          // IO OUTPUT signal
    wire    n_pWR;
    wire    z80_n_HoldIn;
    wire    pHLDA;
    wire    pDBIN;
    wire    romDisable;
    wire    romHigh;
    wire    usbDataReady;
    wire    usbUARTbusy;
    wire    usbByteRcvd;
    wire    usbBusyRcvg;
    wire    usbUARTerror;
    wire    ide8255_cs;
    wire    ide8255_RD;
    
    
    wire    statDisable;
    wire    ctlDisable;
    
    reg [20:0]  counter;            // 26-bit counter
    wire [6:0]  z80_stat;           // z80 CPU status register
    wire [6:0]  statusout;          // z80 S100 status outputs
    wire [4:0]  controlBus;         // S100 Control signals mux in
    wire [4:0]  controlOut;         // S100 Control Signals Output to bus
    wire [15:0] buildAddress;         // S100 address build location
    wire [13:0] romAddress;
    wire [16:0] ramAddress;
    
////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////  MISC TESTING/DEBUG STUFF
assign boardActive = !pll0_LOCKED;   // LED is LOW to turn ON

assign n_reset = s100_n_RESET;
assign seg7 = 7'b0001000;               // The letter "A", Top segment is LSB
assign seg7_dp = !(n_resetLatch & counter[20]); // Tick to show activity
assign  cpuClkOut_P19 = Clkcpu;

assign  diagLED = boardWait;

assign spare_P1 = !((!z80_n_iorq) & psync);
assign spare_P17 = cpuClock;
assign spare_P32 = z80_n_wait;
assign spare_P33 = ioWait;

////////////////////////////    TURN ON SBC BUFFERS FOR NOW
assign F_add_oe = !F_in_sdsb;            // Address Bus enable  GPIOB_TXN17
assign F_bus_stat_oe = !F_in_sdsb;       // Status bus enable   GPIOB_TXN19
assign F_out_DO_oe = !F_in_sdsb;         // S100 Data OUT bus   GPIOL_114
assign F_out_DI_oe = !F_in_sdsb;         // S100 Data IN bus    GPIOL_116
assign F_bus_ctl_oe = !F_in_cdsb;        // Control bus enable  GPIO_R120

///////////////////////////     Create various Z80 and S100 signals

assign  n_inta = !(!z80_n_m1 & !z80_n_iorq);        // create Interrupt ACK signal 
assign  io_output = (!z80_n_wr & !z80_n_iorq);      // create OUTPUT signal
assign  mrq_norfsh = ((!z80_n_mreq) & z80_n_rfsh);  // memory rqst, NOT during refresh
assign  n_mwr = !(mrq_norfsh & z80_n_rd);           // memory WRITE, NOT during refresh
assign  sWO = (n_mwr & z80_n_wr);                    // combined write out
assign  psyncstrt = !(inta | liorq | mrq_norfsh);   // START_SYNC NOR gate on Waveshare
assign  endsync = !(psyncend);

assign  z80mreq = !z80_n_mreq;
assign  psync = !(psyncstrt | endsync | !z80_n_rfsh);   // PSYNCRAW NOR gate on Waveshare
assign  busin = (n_inta & z80_n_rd);       // create the BUS IN signal
assign  pstval = !(psync & !cpuClock);   // create the pSTVAL signal
assign  n_pWR = !(endsync & (!z80_n_wr));
assign  sOUT = (!z80_n_wr & !z80_n_iorq);    // create the basic OUT status bit
assign  sINP = (!z80_n_rd & !z80_n_iorq);    // crfeate the basic INP status bit
assign  memRD = (!z80_n_rd & !z80_n_mreq);   // create the basic memory READ status bit
assign  memWR = (!z80_n_wr & !z80_n_mreq);   // create the basic memory WRITE status bit
assign  romDisable = miscControl[1];
assign  romHigh = (miscControl[0] | miscControl[2]);

assign outPrn = counter[19:12];
assign outPrnStrobe = counter[20];
assign boardWait = (romWait & ioWait);
assign buzzer = 1'b1;


/************************************************************************************
*       HM628512 512k STATIC RAM CHIP INTERACE                                      *
************************************************************************************/
assign ram_A16 = ramAddress[16];        //outramA16[0];      // A16 to RAM chip
assign ram_A17 = 1'b0;          //S100adr16_19[1];           // A17 to RAM chip
assign ram_A18 = 1'b0;          //S100adr16_19[2];           // A18 to RAM chip
assign ram_n_cs = !ram_cs;      // Static RAM chip select signal (active low)
assign ram_n_oe = !memRD;       // Static RAM memory READ signal (active low)
assign ram_n_wr = !memWR;       // Static RAM memory WRITE signal (active low)

assign biData_OUT[7:0] = cpuDataOut[7:0];   // set bidirectional data to CPU data

assign biOutEN =  (memWR & ram_cs); // Create the biDir. Out Enable for RAM Writes
assign biData_OE[0] = biOutEN;      // LOW enables Bidirectional Data OUT 0
assign biData_OE[1] = biOutEN;      // LOW enables Bidirectional Data OUT 1
assign biData_OE[2] = biOutEN;      // LOW enables Bidirectional Data OUT 2
assign biData_OE[3] = biOutEN;      // LOW enables Bidirectional Data OUT 3
assign biData_OE[4] = biOutEN;      // LOW enables Bidirectional Data OUT 4
assign biData_OE[5] = biOutEN;      // LOW enables Bidirectional Data OUT 5
assign biData_OE[6] = biOutEN;      // LOW enables Bidirectional Data OUT 6
assign biData_OE[7] = biOutEN;      // LOW enables Bidirectional Data OUT 7

/*************************************************************************************
*           IDE Interface       For Compact Flash Card                               *
*************************************************************************************/
assign out8255_n_cs = !(ide8255_cs);                 // to 8255 chip select (pin 6)
assign idePorts_n_rd = (outIDE_n_rd | out8255_n_cs); // from 8255 to CPU (IN) bfr U27
assign idePorts_n_wr = (outIDE_n_wr | out8255_n_cs); // to 8255 from CPU (OUT) bfr U24
assign outIDE_n_rd = !(s100_sINP & !pDBIN);          // to 8255 RD* (pin 5)
assign outIDE_n_wr = !(sOUT & !n_pWR);               // to 8255 WR* (pin 36)

/************************************************************************************
*   Z80 Status register.  Controls the seven Z80 Status out (s) bits.               *
************************************************************************************/
assign z80_stat[0] = !z80_n_halt;   // inverted z80 !HLTA
assign z80_stat[1] = sOUT;          // sOUT signal
assign z80_stat[2] = sINP;          // sIN signal
assign z80_stat[3] = memRD;         // sMEMR signal
assign z80_stat[4] = sWO;           // create sWO- signal
assign z80_stat[5] = !z80_n_m1;     // create the sM1 signal
assign z80_stat[6] = !n_inta;       // create sINTA signal

/************************************************************************************
*   Z80 Control Register.  Control bits out (p bits).                               *
************************************************************************************/
assign  controlBus[0] = psync;      // Active High, start of new bus cycle
assign  controlBus[1] = pstval;     // Active LOW, Indicates stable address & status
assign  controlBus[2] = pDBIN;      // Active HIGH, read strobe, slave can input data
assign  controlBus[3] = n_pWR;      // Active LOW, generalized write strobe to slaves
assign  controlBus[4] = pHLDA;      // Active HIGH, Perm Master relinquishing control

/************************************************************************************
*   Various Address bus creation or modification                                                          *
************************************************************************************/
assign buildAddress[7:0] = cpuAddress [7:0];
assign ramAddress[15:0] = buildAddress[15:0];
assign ramAddress[16] = (!buildAddress[15] & outramA16[0]);
assign romAddress[11:0] = buildAddress[11:0];
assign romAddress[12] = miscControl[0];
assign romAddress[13] = miscControl[2];

assign  highRomLED = (~romHigh);
assign  lowRomLED = (romHigh);
assign  highRamLED = ~outramA16[0];
assign  prnAckLED = 1'b1;

/********************************************************************************
*       USB Status Register                                                     *
********************************************************************************/
assign usbStat[0] = usbDataReady;   // USB UART Data Ready
assign usbStat[1] = usbUARTbusy;    // USB UART Tx busy
assign usbStat[2] = usbByteRcvd;    // USB UART byte received
assign usbStat[3] = usbBusyRcvg;    // USB UART busy receiving
assign usbStat[4] = 1'b0;
assign usbStat[5] = 1'b0;
assign usbStat[6] = 1'b0;
assign usbStat[7] = usbUARTerror;   // USB UART receive error
assign usbRXbusyLED = !usbByteRcvd;
assign usbTXbusyLED = !usbUARTbusy;

//////////////////////////////////////  FIXED S100 SIGNALS HERE /////////////////////
// s100_pDBIN created by read strobe FF below
assign s100_pDBIN = !pDBIN;
assign z80_n_wait = s100_xrdy & s100_rdy & boardWait;      // Z80 Wait = low to wait
assign s100_pSYNC = psync;
assign s100_pSTVAL = pstval;
assign s100_n_pWR = n_pWR;
assign s100_PHI = cpuClock;
assign s100_CLOCK = pll0_2MHz;             
assign s100_sMWRT = !(n_pWR | io_output);
assign s100_pHLDA = !z80_n_busak;
assign s100_DO = cpuDataOut;                  // S100 Data OUT bus signals
 
//////////////////////////////////////////////////////////////////////////////////////
//      Status Output signals, per IEEE-696.  These signals are latched by pSTVAL   //
//          based on John Monahan's Waveshare design.                               //
//          Status signals are prefixed with an "s"                                 //
//////////////////////////////////////////////////////////////////////////////////////
assign  s100_sHLTA = statusout[0];      //!z80_n_halt;           //statusout[0];
assign  s100_sOUT =  statusout[1];      //sOUT;              //statusout[1];
assign  s100_sINP =  statusout[2];      //sINP;              //statusout[2];
assign  s100_sMEMR = statusout[3];      //memRD;           //statusout[3];
assign  s100_n_sWO = statusout[4];      //sWO;               //statusout[4];
assign  s100_sM1 =   statusout[5];      //!z80_n_m1;             //statusout[5];
assign  s100_sINTA = statusout[6];      //!n_inta;           //statusout[6];

//////////////////////////////////////////////////////////////////////////////////
//  Control Output signals, per IEEE-696.  These signals can be tristated based //
//  on John Monahan's Waveshare design.  Efinix does not have tri-state         //
//  outputs internally, so these outputs are set high instead.... for now       //
//  These signals are prefixed with a "p"                                       //
//////////////////////////////////////////////////////////////////////////////////
//assign  s100_pSYNC = controlOut[0];
//assign  s100_pSTVAL = cntrolOut[1];
//assigns100_pDBIN = controlOut[2];
//assign  s100_n_pWR = controlOut[3];
//assign  s100_pHLDA = controlOut[4];

assign s100_PHANTOM = 0;
assign s100PhantomLED = ~s100_PHANTOM;

assign s100_ADSB = !statDisable;                // Address and Status Disable
assign s100_CDSB = !ctlDisable;                 // Control Signals Disabe



//////////////////////////////////////////////////////////////////////////////
//      Debug Register.  These are displayed when IOBYTE switches 5 & 4     //
//          are set to 10 (OFF ON)                                          //
//////////////////////////////////////////////////////////////////////////////
assign debugReg[0] = !ram_cs;
assign debugReg[1] = !n_reset;          //z80_n_rd;
assign debugReg[2] = !nop_cs;           //z80_n_mreq;
assign debugReg[3] = !rom_cs;
assign debugReg[4] = s100_pDBIN;
assign debugReg[5] = s100_sMEMR;
assign debugReg[6] = s100_sINP;
assign debugReg[7] = s100_sOUT;

//////////////////////////////////////////////////////////////////////////
always @(posedge pll0_2MHz)
    begin
        if(n_reset == 0) begin      // if reset set low...
            counter <= 21'b0;       // reset counter to 0
        end                         // end of reset counter
        else
            counter <= counter + 1; // increment counter
    end
    

////////////////////////////////////////////////////////////////////////////
///////////     Z80 microcomputer module       (Z80 top module)         ////
////////////////////////////////////////////////////////////////////////////
    
microcomputer(
		.n_reset    (n_reset),              // INPUT  LOW to reset
		.clk        (cpuClock),
		
		.n_wr       (z80_n_wr),
		.n_rd       (z80_n_rd),
		.n_mreq     (z80_n_mreq),
		.n_iorq     (z80_n_iorq),
		.n_wait		(z80_n_wait),
        .n_int      (1'b1),
		.n_nmi      (1'b1),
        .n_busrq    (z80_n_HoldIn),
        .n_m1       (z80_n_m1),
        .n_rfsh     (z80_n_rfsh),
        .n_halt     (z80_n_halt),
		.n_busak    (z80_n_busak),
        .clkcpu     (Clkcpu),           // microcomputer.vhd divider output    
		.address    (cpuAddress),
		.dataOut    (cpuDataOut),
		.dataIn     (cpuDataIn)	
		);

/************************************************************************************
*   S100 High Address MUX.  Z80 sends I/O Data OUT on A8-A15.  This disables that   *
************************************************************************************/
cpuHAdrMux  HighAdrMux(
    .cpuHighAdr     (cpuAddress[15:8]),     // Address for all but INPUT or OUTPUT
    .sOUT           (sOUT),                 // Zero out A15-08 if an OUTPUT
    .sINP           (sINP),                 // or an INPUT  
    .pll0_250MHz    (pll0_250MHz),
    .HighAdr        (buildAddress[15:8]));  // Send modified A15-08 to BUILD Address
        
 /************************************************************************************
*   CPU Data INPUT Multiplexer      Note: Efinix FPGAs do NOT have tristate ability  *
*************************************************************************************/
cpuDIMux    cpuInMux (
    .romData        (romOut[7:0]),
    .s100DataIn     (s100_DI[7:0]),
    .ramaData       (biData_IN[7:0]),
    .ledread        (fbarSbcLeds[7:0]),
    .iobyte         (sw_IOBYTE[7:0]),    
    .usbStatus      (usbStat[7:0]),
    .usbRxD         (usbRxData[7:0]),
    .reset_cs       (nop_cs),
    .rom_cs         (rom_cs),
    .inPortcon_cs   (inPortCON_cs),
    .ram_cs         (ram_cs),
    .inLED_cs       (inBarLED_cs),
    .iobyteIn_cs    (iniobyte_cs),  
    .usbStat_cs     (inusbStat_cs),
    .usbRxD_cs      (inusbRxData_cs),
    .ide_cs         (ide8255_cs),
    .pll0_250MHz    (pll0_250MHz),
    .outData        (cpuDataIn[7:0])
    );
 
/************************************************************************************
*   Memory decoder                                                                  *
************************************************************************************/     
memAdrDecoder  mem_cs(
    .address        (buildAddress[15:12]),  // change to [15:9] for small ROM (prj 6)
    .memwrite       (memWR),
    .memread        (memRD),
    .reset_cs       (nop_cs),
    .rom_cs         (rom_cs),
    .ram_cs         (ram_cs)
     );

/************************************************************************************
*   Boot ROM for Z80 CPU                                                            *
************************************************************************************/     
rom   #(.ADDR_WIDTH(14),                    // set address width for larger ROMs
	.RAM_INIT_FILE("SBC-MON2_4+4K+4K.inithex"))
            //("SBC-MON2_4+4K+4K.inithex"))     //("RAM_TEST.inithex"))
            //("SBC-MON2_4+4K.inithex"))        //("rom.inithex"))    
            //("00RAM_TEST.inithex"))  //("00RAM_TEST_FPGA_ROM.inithex")) 
            //("RAM_TEST.inithex"))  //("RAM_TEST_FPGA_ROM.inithex"))     
    test_rom (
    .address    (romAddress[13:0]),     //(cpuAddress[10:0]),
	.clock      (cpuClock),
	.data       (romOut[7:0])
);

/************************************************************************************
    IO Ports Decoder.                                                               *
************************************************************************************/
portDecoder ports_cs(
    .address        (cpuAddress[7:0]),
    .iowrite        (sOUT),    
    .ioread         (sINP),
    .outPortFF_cs   (outFF),
    .inPortCon_cs   (inPortCON_cs),
    .outFbarLEDs_cs (outBarLED_cs),
    .inFbarLEDs_cs  (inBarLED_cs),
    .outMiscCtl_cs  (miscCtl_cs),
    .inIOBYTE_cs    (iniobyte_cs),
    .inUSBst_cs     (inusbStat_cs),
    .inusbRxD_cs    (inusbRxData_cs),
    .outusbTxD_cs   (outusbTxData_cs),
    .outRAMA16_cs   (outrama16_cs),
    .idePorts8255_cs  (ide8255_cs));
  
/************************************************************************************
*   Z80 CPU status bits latch.  Output feeds the S100 status bit (sXXXX)            *
************************************************************************************/
n_bitLatch      #(7)
      s100stat(
     .load      (pstval),
     .clock     (!cpuClock),
//     .clr       (1'b0),
     .inData    (z80_stat),
     .regOut    (statusout)
     );

 /************************************************************************************
*   S100 Address 0-15 Latch.     Latches address bus for S100 timing                *
************************************************************************************/
n_bitReg        #(16)
      s100adr(
     .load      (pstval),
     .clock     (!cpuClock),
     .clr       (1'b0),
     .inData    (buildAddress),
     //
     .regOut    (S100adr0_15)
     );

/************************************************************************************
*   S100 Address 16-19 Latch.     Latches address bus A16-A19 for S100 timing       *
************************************************************************************/
n_bitReg        #(4)
      s100adr16_19(
     .load      (pstval),
     .clock     (!cpuClock),
     .clr       (1'b0),
     .inData    (4'b0),
     .regOut    (S100adr16_19)
     );

/************************************************************************************
*   S100 Control Bus Signals MUX.  Sets Control bus to Z80 signals or all high      *
************************************************************************************/
ctlBusMux  CtlBusMux(
    .controlin      (controlBus),
    .select         (!n_resetLatch),
    .pll0_250MHz    (pll0_250MHz),
    .controlout     (controlOut)        
    );

/************************************************************************************
*   FBAR Diagnostic LEDs on Port 06 (also readback capability)                      *
************************************************************************************/
n_bitReg    outPort06(
 //    #(parameter N = 8)(
     .load      (outBarLED_cs),
     .clock     (cpuClock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (fbarSbcLeds));


/************************************************************************************
*   Output Port 07      Misc Control bits                                           *
************************************************************************************/
n_bitReg    outPort07(
 //    #(parameter N = 8)(
     .load      (miscCtl_cs),
     .clock     (cpuClock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (miscControl));

/************************************************************************************
*   Port 35  Out = USB Tx Data Out                                                  *
************************************************************************************/
n_bitReg    outUSBport35(
 //    #(parameter N = 8)(
     .load      (outusbTxData_cs),
     .clock     (cpuClock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (usbTxData));
     
     
/************************************************************************************
*   Port 36  IN = IOBYTE switches,     Out = RAM A16 Page Low/High (using D0)       *
************************************************************************************/
n_bitReg    outPort36(
 //    #(parameter N = 8)(
     .load      (outrama16_cs),
     .clock     (cpuClock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (outramA16));
   
/************************************************************************************
*   S100 output Port 255 (0xFF) to Front Panel LEDs                                 *
************************************************************************************/
n_bitReg    outPortFF(
 //    #(parameter N = 8)(
     .load      (outFF),
     .clock     (cpuClock),
     .clr       (!n_reset),
     .inData    (cpuDataOut),
     .regOut    (out255)
    );
    
/*************************************************************************************
*   onboard LEDs INPUT Multiplexer.  This allows quick troubleshooting               *
*       NOTE NOTE   This module INVERTS the output for driving the LEDs              *
*************************************************************************************/
LedBarMux       lmux(
    .cpuDO          (cpuDataOut [7:0]),     // if both switches UP (00)
    .cpuDI          (cpuDataIn [7:0]),      // if Switches UP DN (01)
    .fbarSbcLeds    (fbarSbcLeds),          // if both switches DOWN (11)
    .portFFDO       (out255[7:0]),              // if switches DN UP (10)   
//    .debugreg   (debugReg[7:0]),        
    .sw         	(sw_IOBYTE[5:4]),
    .pll0_25MHz    (pll0_25MHz),
     //    .ram_cs,   
     .LEDoutData   (SBC_LEDs)   // INVERTED DATA OUT TO DRIVE LEDS!!!!!!
    );

/*********************************************************************************
*   IORQ Latch FF                                                                *
*********************************************************************************/
dff3     iorqlatch(
        .clk        (cpuClock),
        .pst_n      (1'b1),
        .clr_n      (!z80_n_iorq),     
        .din        (!z80_n_iorq),
        .q          (liorq)
        );
        
/********************************************************************************
*   pSYNC End latch FF                                                          *
********************************************************************************/        
dff3     endpsync(          // was dff
        .clk        (cpuClock),
        .pst_n      (!psyncstrt),
        .clr_n      (1'b1),     
        .din        (psyncstrt),
        .q          (psyncend)
        );

/********************************************************************************
*   Read Strobe latch FF    Output creates/latches pDBIN signal                 *
********************************************************************************/
dff2     readstrobe(                // was dff2
        .clk        (!pstval),
//        .pst_n      (1'b1),
        .clr_n      (busin),     
        .din        (busin),
        .q          (pDBIN)
        );

/********************************************************************************
*   RESET Latch FF      Output drived Active LED and Control Out mux            *
********************************************************************************/        
dff3     resetLatch(                // was dff
        .clk        (cpuClock),
        .pst_n      (n_reset),          // was(1'b1),
        .clr_n      (1'b1),     
        .din        (1'b1),
        .q          (n_resetLatch)
        );

/********************************************************************************
*   S100 HOLD IN (busreq) Latch FF  Driven from S100 HOLD pin                   *
********************************************************************************/        
dff3     holdInLatch(               // was dff
        .clk        (cpuClock),
        .pst_n      (1'b1),
        .clr_n      (1'b1),     
        .din        (s100_HOLD),
        .q          (z80_n_HoldIn)
        );

/********************************************************************************
*   S100 HLDAout (busak) Latch FF  Outputs HLDA to disable Address and Status   *
********************************************************************************/        
dff3     HLDAoutLatch(              // was dff
        .clk        (!cpuClock),
        .pst_n      (1'b1),
        .clr_n      (!z80_n_busak),     
        .din        (!z80_n_busak),
        .q          (statDisable)
        );

/********************************************************************************
*   S100 HOLD IN (busreq) Latch FF  Driven from S100 HOLD pin                   *
********************************************************************************/        
dff3     ctlDisableLatch(       //was dff
        .clk        (cpuClock),
        .pst_n      (1'b1),
        .clr_n      (1'b1),     
        .din        (!(s100_HOLD | !statDisable)),
        .q          (ctlDisable)
        );

/********************************************************************************
*   S100 HOLD IN (busreq) Latch FF  Driven from S100 HOLD pin                   *
********************************************************************************/        
dff3    jumptoRom(
        .clk        (n_reset),
        .pst_n      (!rom_cs),
        .clr_n      (1'b1),     
        .din        (1'b0),
        .q          (nop_cs));

/********************************************************************************
*   CPU Clock input Mux.  Selects one of four clock frequencies                 *
********************************************************************************/
//assign cpuClock = pll0_25MHz;

ClockMux    ClkMux(
    .MHz2       (pll0_2MHz),
    .MHz25       (pll0_25MHz),
    .KHz31      (counter[5]),
    .Hz250      (counter[12]),
    .pll0_250MHz (pll0_250MHz),
    .sw         (sw_IOBYTE[7:6]), 
    .cpuclk     (cpuClock)
    );
    
/********************************************************************************
*   IO wait generation shift register (similar to hardware 74LS165)             *
*       Each zero in SerIn or ParIn causes one wait state output                *
*       ioWaitValue: FF = no wait states, 00 = 312ns (max wait)                 *
*       To add wait states, start with MOST significant bit first(ie: 7F=1 wait)*
*       3F = 2 wait states, etc.  DO NOT START WITH LSB first,as wait state     *
*           timing will be off.  For now, leave SerIn as a high for after shift *
********************************************************************************/
ParShiftReg     iowait(
    .clk        (cpuClock),                  // shift on Positive clock edge
//    .clr        (1'b1),                        // active low to clear
    .SerIn      (1'b1),                      // shift register serial input
    .ParIn      (ioWaitvalue),               // Shift register parallel input
    .load       ((!z80_n_iorq) & psync),     // Shift (high)/Load (low) input
    .qout       (ioWait)                     // single bit WAIT output
    );

/********************************************************************************
*   ROM wait generation shift register (similar to hardware 74LS165)            *
*       Each zero in SerIn or ParIn causes one wait state output                *
*       Timing is the same as IO wait states described above                    *
********************************************************************************/
ParShiftReg     romwait(
    .clk        (cpuClock),                     // shift on Positive clock edge
//    .clr        (1'b1),                         // active low to clear
    .SerIn      (1'b1),                         // shift register serial input
    .ParIn      (romWaitvalue),                 // Shift register parallel input
    .load       (!(z80mreq & rom_cs & psync)),  // Shift (high)/Load (low) input
    .qout       (romWait)                       // single bit output
    );

 
/****************************************************************************
*   Eight bit shift register
****************************************************************************/
ShiftReg    usbTXdelay(
    .clk    (cpuClock),
    .clr    (!usbUARTbusy),
    .SerIn  (outusbTxData_cs),
    .qout   (usbTxDelay));

/****************************************************************************
*       USB UART serial data transfer                                       *
*           From opencores                                                  *
****************************************************************************/
uart  usbuart(
    .clk                (clockIn),		// The master clock for this module 50MHz
    .rst                (!n_reset),     // Synchronous reset.
    .rx 				(usbRXData),		// UART Input - Incoming serial line
    .tx 				(usbTXData),	    // UART output - Outgoing serial line
    .transmit 			(usbTxDelay[2]), // Input to UART - Signal to transmit a byte = 1
    .tx_byte 		    (usbTxData),    // UART input - Byte to transmit
    .received 			(usbByteRcvd),   // UART output - Indicates a byte has been rcvd
    .rx_byte 		    (usbRxData),     // UART OUTPUT - Byte received
    .is_receiving 		(usbBusyRcvg),   // UART output Low when receive line is idle.
    .is_transmitting 	(usbUARTbusy),   // UART output - Low when transmit line is idle.
    .recv_error         (usbUARTerror),  // output - Indicates error in receiving data.
	.data_ready         (usbDataReady),  // UART Output - has Rx data
	.data_read          (inusbRxData_cs)); // UART input - read the received data
    
endmodule   
    
