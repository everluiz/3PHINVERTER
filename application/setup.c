/*
 * setup.c
 *
 *  Created on: 1 de jul de 2022
 *      Author: Ever_
 */
#include "setup.h"

void setup_GPIO(void){
    EALLOW;

    /*GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;    // configura como GPIO o pino 31

    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // configura como GPIO o pino 34

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;     // Led 31 (Blue) output
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;     // Led 34 (Red) output

    GpioCtrlRegs.GPAPUD.bit.GPIO31 = 1;  // Disable pull-up on GPIO31
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;  // Disable pull-up on GPIO34

    GpioCtrlRegs.GPACSEL4.bit.GPIO31 = GPIO_MUX_CPU1; // seleciona qual CPU/CLA manipula o pino
    GpioCtrlRegs.GPBCSEL1.bit.GPIO34 = GPIO_MUX_CPU1; // seleciona qual CPU/CLA manipula o pino*/

    // GPIO 12 como controle do disjuntor de conexao com a rede (Disj_PCC)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;                    // Configura MUX e GMUX para usar a GPIO 12 como saida
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;                      // Desabilita os resistores de pull-up
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;                      // Ajusta como saida digital
    GpioDataRegs.GPADAT.bit.GPIO12 = 0;                      // Inicializa a saida em estado logico baixo
    GpioCtrlRegs.GPACSEL2.bit.GPIO12 = GPIO_MUX_CPU1;        // Designa a CPU1 para controlar a GPIO

    // GPIO 13 como controle do gate IGBT do SC e BAT
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;                    // Configura MUX e GMUX para usar a GPIO 13 como saida
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;                      // Desabilita os resistores de pull-up
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;                      // Ajusta como saida digital
    GpioDataRegs.GPADAT.bit.GPIO13 = 0;                      // Inicializa a saida em estado logico baixo
    GpioCtrlRegs.GPACSEL2.bit.GPIO13 = GPIO_MUX_CPU1;        // Designa a CPU1 para controlar a GPIO

//    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0; //sinal trigger do ADC para ler no typhoon
//    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
//    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
//    GpioCtrlRegs.GPACSEL2.bit.GPIO10 = 0;

    //PWM-1A ( DUTY CICLE ADC TRIGGER)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;    // configura como PWM o pino 0
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;     // Disable pull-up on GPIO0

    //PWM-1B ( DUTY CICLE BOOST PV)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;   // configura o grupo 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;    // configura como PWM o pino 1
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;     // Disable pull-up on GPIO0

    //PWM-2A ( DUTY CICLE INVERSOR A:S1)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;   // configura o grupo 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;    // configura como PWM o pino 2
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;     // Disable pull-up on GPIO2

    //PWM-2B complementar ( DUTY CICLE INVERSOR A:S2)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;   // configura o grupo 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;    // configura como PWM o pino 3
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;     // Disable pull-up on GPIO2

    //PWM-3A ( DUTY CICLE INVERSOR B:S1)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0;   // configura o grupo 2
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;    // configura como PWM o pino 4
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;     // Disable pull-up on GPIO4

    //PWM-3B complementar ( DUTY CICLE INVERSOR B:S2)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 0;   // configura o grupo 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;    // configura como PWM o pino 5
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;     // Disable pull-up on GPIO2

    //PWM-4A ( DUTY CICLE INVERSOR C:S1)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO6 = 0;   // configura o grupo 3
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;    // configura como PWM o pino 6
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;     // Disable pull-up on GPIO6

    //PWM-4B complementar ( DUTY CICLE INVERSOR C:S2)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO7 = 0;   // configura o grupo 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;    // configura como PWM o pino 7
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;     // Disable pull-up on GPIO2


    //PWM-5A ( DUTY CICLE Bidirecional SC:S1)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;    // configura como PWM o pino 8
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;     // Disable pull-up on GPIO8

    //PWM-5B complementar ( DUTY CICLE Bidirecional SC:S2)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;    // configura como PWM o pino 9
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;     // Disable pull-up on GPIO9

    //PWM-6A ( DUTY CICLE Bidirecional BAT:S1)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;    // configura como PWM o pino 10
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;     // Disable pull-up on GPIO8

    //PWM-6B complementar ( DUTY CICLE Bidirecional BAT:S2)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 0;   // configura o grupo 0
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;    // configura como PWM o pino 11
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;     // Disable pull-up on GPIO9

    //GPIO 28 e 29 USCI-A

    //GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO28 = GPIO_MUX_CPU1;

    //GpioCtrlRegs.GPAGMUX2.bit.GPIO29 = 3;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
    GpioCtrlRegs.GPACSEL4.bit.GPIO29 = GPIO_MUX_CPU1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;           // Asynch input

//    GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 3;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;
//    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 1;
//    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;
//    GpioCtrlRegs.GPBCSEL2.bit.GPIO42 = GPIO_MUX_CPU1;
//
//    GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 3;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;
//    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;
//    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 0;
//    GpioCtrlRegs.GPBCSEL2.bit.GPIO43 = GPIO_MUX_CPU1;
//    GpioCtrlRegs.GPBQSEL1.bit.GPIO43 = 3;           // Asynch input

    EDIS;
}

void setup_ePWM(void){
    EALLOW;
    //======================================================================================================================================================
    //    EPWM-1A ( ADC TRIGGER - SOC "start of conversion") && EPWM-1B ( boost PV)
    //
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;        // habilita clock do periferico epwm1
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;     // desabilita o timer base (pulsos do cristal) durante configuracao


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up-down"
     *
     *      PRD = 200e6 / (4 * 20e3) = 2500
     */
    // configuracao do contador
    EPwm1Regs.TBPRD = 2500;//5000;                        // Timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm1Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;              // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle
    EPwm1Regs.CMPA.bit.CMPA = (EPwm1Regs.TBPRD >> 1);       // D= 0.5

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    // acao realizada pelo PWM B
    EPwm1Regs.AQCTLB.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;                    // set when reaches CMP counting down

    //duty cycle PWM B
    EPwm1Regs.CMPB.bit.CMPB = (uint16_t)(0 *((float)EPwm1Regs.TBPRD));       // start at D = 0

    //PWM complementar
    //EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    //EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    //EPwm1Regs.DBFED.bit.DBFED = 100;                      // FED = 1us TBCLKs (faling edge delay)
    //EPwm1Regs.DBRED.bit.DBRED = 100;                      // RED = 1us TBCLKs (rising edge delay)

    //Trigger ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                     // enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;//ET_CTR_PRDZERO;       // dispara ADC no PRD e zero;
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;                // trigger on every event

    //======================================================================================================================================================
    //                                            EPWM-2A ( Inversor A-S1)
    //
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;        // habilita clock do periferico epwm2


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up-down"
     *
     *      PRD = 200e6 / (4 * 20e3) = 2500
     */
    // configuracao do contador
    EPwm2Regs.TBPRD = 2500;//2500;                        // Timer period
    EPwm2Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm2Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)(0 *((float)EPwm2Regs.TBPRD));       // start at D = 0
    //EPwm2Regs.CMPA.bit.CMPA = (EPwm2Regs.TBPRD >> 1)+(EPwm2Regs.TBPRD >> 2);       // half of TBPRD + a quarter of TBPRD, D= 0.5+0.25 = 0.75

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM
    EPwm2Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    //PWM complementar
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    EPwm2Regs.DBFED.bit.DBFED = 10;                      // FED = 20 TBCLKs (faling edge delay)
    EPwm2Regs.DBRED.bit.DBRED = 10;                      // RED = 20 TBCLKs (rising edge delay)

    //Trigger ADC
    //EPwm2Regs.ETSEL.bit.SOCAEN = 1;                     // enable SOC on A group
    //EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // dispara ADC no PRD e zero
    //EPwm2Regs.ETPS.bit.SOCAPRD = ET_1ST;                // trigger on every event

    //======================================================================================================================================================
    //                                            EPWM-3A ( Inversor B-S1)
    //
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;        // habilita clock do periferico epwm3


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up-down"
     *
     *      PRD = 200e6 / (4 * 20e3) = 2500
     */
    // configuracao do contador
    EPwm3Regs.TBPRD = 2500;//2500;                        // Timer period
    EPwm3Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm3Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;              // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)(0 *((float)EPwm3Regs.TBPRD));       // start at D = 0

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM
    EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    //PWM complementar
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    EPwm3Regs.DBFED.bit.DBFED = 10;                      // FED = 20 TBCLKs (faling edge delay)
    EPwm3Regs.DBRED.bit.DBRED = 10;                      // RED = 20 TBCLKs (rising edge delay)

    //Trigger ADC
    //EPwm3Regs.ETSEL.bit.SOCAEN = 1;                     // enable SOC on A group
    //EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // dispara ADC no PRD e zero
    //EPwm3Regs.ETPS.bit.SOCAPRD = ET_1ST;                // trigger on every event

    //======================================================================================================================================================
    //                                            EPWM-4A ( Inversor C-S1)
    //
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;        // habilita clock do periferico epwm4


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up ou down"
     *
     *      PRD = 200e6 / (2 * 20e3) = 5000
     */
    // configuracao do contador
    EPwm4Regs.TBPRD = 2500;//2500;                        // Timer period
    EPwm4Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm4Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;              // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle
    EPwm4Regs.CMPA.bit.CMPA = (uint16_t)(0 *((float)EPwm3Regs.TBPRD));       // start at D = 0

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM
    EPwm4Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    //PWM complementar
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    EPwm4Regs.DBFED.bit.DBFED = 10;                      // FED = 20 TBCLKs (faling edge delay)
    EPwm4Regs.DBRED.bit.DBRED = 10;                      // RED = 20 TBCLKs (rising edge delay)

    //Trigger ADC
    //EPwm4Regs.ETSEL.bit.SOCAEN = 1;                     // enable SOC on A group
    //EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // dispara ADC no PRD e zero
    //EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;                // trigger on every event

    //======================================================================================================================================================
    //                                            EPWM-5A ( Bidirecional SC )
    //
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;        // habilita clock do periferico epwm2


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up-down"
     *
     *      PRD = 200e6 / (4 * 20e3) = 2500
     */
    // configuracao do contador
    EPwm5Regs.TBPRD = 2500;//2500;                        // Timer period
    EPwm5Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm5Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm5Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle PWM A
    EPwm5Regs.CMPA.bit.CMPA = (uint16_t) (uint16_t)(0 *((float)EPwm5Regs.TBPRD)); // start at D = 0

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM A
    EPwm5Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    // configura trip zone
    EPwm5Regs.TZCTL.bit.TZA = 0b10;  // Force EPWMxA low
    EPwm5Regs.TZCTL.bit.TZB = 0b10;  // Force EPWMxB low

    //PWM complementar
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    EPwm5Regs.DBFED.bit.DBFED = 10;                      // FED = 20 TBCLKs (faling edge delay)
    EPwm5Regs.DBRED.bit.DBRED = 10;                      // RED = 20 TBCLKs (rising edge delay)

    //Trigger ADC
    //EPwm5Regs.ETSEL.bit.SOCAEN = 1;                     // enable SOC on A group
    //EPwm5Regs.ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;       // dispara ADC no PRD e zero
    //EPwm5Regs.ETPS.bit.SOCAPRD = ET_1ST;                // trigger on every event

    //======================================================================================================================================================
    //                                            EPWM-6A ( Bidirecional BAT )
    //
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;        // habilita clock do periferico epwm6


    /*
     *      PRD = Fclock / (2*PWMfreq)     "para modos up e down"
     *
     *      PRD = Fclock / (4*PWMfreq)     "para modo up-down"
     *
     *      PRD = 200e6 / (4 * 20e3) = 2500
     */
    // configuracao do contador
    EPwm6Regs.TBPRD = 2500;//2500;                        // Timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0;                        // Phase 0
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;       // sincronismo de PWMs desabilitado
    EPwm6Regs.TBCTR = 0x0000;                             // Clear counter
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;        // Count up/down
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;               // Disable phase loading
    EPwm6Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;                // Clock ratio to SYSCLKOUT, prescaler high speed clock
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;                 // prescaler clock

    //duty cycle PWM A
    EPwm6Regs.CMPA.bit.CMPA = (uint16_t) (uint16_t)(0 *((float)EPwm6Regs.TBPRD)); // start at D = 0

    //shadow (atualizacao de PRD e CMP apenas quanco contador estiver em ZERO ou PRD)
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;           // Load registers every ZERO or PRD
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;

    // acao realizada pelo PWM A
    EPwm6Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;              // no action when counter reaches PRD
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;              // no action when counter reaches ZERO
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;                  // clear when reaches CMP counting up
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;                    // set when reaches CMP counting down

    // configura trip zone
    EPwm6Regs.TZCTL.bit.TZA = 0b10;  // Force EPWMxA low
    EPwm6Regs.TZCTL.bit.TZB = 0b10;  // Force EPWMxB low

    //PWM complementar
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;            // Active Hi complementary
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;       // enable Dead-band module
    EPwm6Regs.DBFED.bit.DBFED = 10;                      // FED = 20 TBCLKs (faling edge delay)
    EPwm6Regs.DBRED.bit.DBRED = 10;                      // RED = 20 TBCLKs (rising edge delay)


    // ----------------- FINAL SETTING ----------------------
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;     // habilita o timer base (pulsos do cristal)
    EDIS;
}

void setup_ADC_A(void){
    uint16_t acqps;
    // determine minimum acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        acqps = 14;//40;                                 // 75 ns (resolution is 12-bit)
    else
        acqps = 63;                                 // 320 ns (resolution is 16-bit)

    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;              // ADCCLK enable
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;              // set ADCCLK divider

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;           // set pulse um ciclo antes do resultado
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;              // power up the ADC
    DELAY_US(1000);                                 // delay for 1ms to allow ADC time to power up

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;              // SOC0 channel select to ADCINA0 == [ Va_pcc ]
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)                                                    //ADCSOC1CTL table (pag1640 referenceManual)

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;              // SOC1 channel select to ADCINA1 == [ Vb_pcc ]
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;            // set SC1 start trigger on EPWM1A

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;              // SOC2 channel select to ADCINA2 == [ Vc_pcc ]
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;            // set SC2 start trigger on EPWM1A

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;              // SOC3 channel select to ADCINA3 == [ V_PV ]
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;            // set SC3 start trigger on EPWM1A

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;              // SOC4 channel select to ADCINA4 == [ V_SC ]
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;            // set SC4 start trigger on EPWM1A

    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;              // SOC5 channel select to ADCINA5 == [ V_CC ]
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 5;            // set SC2 start trigger on EPWM1A


    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;       // end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;            // enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // INT1 flag cleared


    EDIS;
}

void setup_ADC_B(void){
    uint16_t acqps;
    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;              // ADCCLK enable
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;              // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;           // set pulse um ciclo antes do resultado
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;              // power up the ADC
    DELAY_US(1000);                                 // delay for 1ms to allow ADC time to power up
    if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
            acqps = 40;                                 // 75 ns (resolution is 12-bit)
        else
            acqps = 63;                                 // 320 ns (resolution is 16-bit)

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;              // SOC0 channel select to ADCINB0 == [ iL_a ]
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;              // SOC1 channel select to ADCINB1 == [ iL_b ]
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;              // SOC1 channel select to ADCINB2 == [ iL_c ]
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;              // SOC1 channel select to ADCINB2 == [ iL_PV ]
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 4;              // SOC1 channel select to ADCINB2 == [ iL_SC ]
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;              // SOC1 channel select to ADCINB2 == [ iL_Bat ]
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = acqps;          // sample window in SYSCLK cycles
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 5;            // set SC0 start trigger on EPWM1A (TRIG_SEL_ePWM1_SOCA)

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 5;       // end of SOC2 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;            // enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // INT1 flag cleared
    EDIS;
}

void Setup_UART(void){
    // pg 2279 spruhm8i.pdf - Technical reference
    SciaRegs.SCICCR.all = 0x0007;           // 1 stop bit,  No loopback, no parity,8 char bits,
    //SciaRegs.SCICCR.bit.SCICHAR = 0x07;     // SCI character length from one to eight           bits

    // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;          // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.bit.TXINTENA = 0;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;    // enable RX interrupt
    SciaRegs.SCIHBAUD.all = 0x0000;         // SCI_PRD LSB
    SciaRegs.SCILBAUD.all = SCI_PRD;        // SCI_PRD MSB

    // buffer msg registers
    SciaRegs.SCIFFTX.all = 0xC022; // every word put in buffer is transmitted
    SciaRegs.SCIFFRX.all = 0x0028; // receive and store eight words in buffer to then call int.

    /*
    SciaRegs.SCIFFRX.bit.RXFFOVF = 0;
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 0;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    SciaRegs.SCIFFRX.bit.RXFFST = 0;
    SciaRegs.SCIFFRX.bit.RXFFINT = 0;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 0;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
    SciaRegs.SCIFFRX.bit.RXFFIL = 1;*/

    SciaRegs.SCIFFCT.all = 0x00;
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}


