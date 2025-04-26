#include "setup.h"
#include "filter.h"
#include "PI_REG1.h"
#include <math.h>
#include <stdio.h>
#include "Controles.h"
#include "fis_EMS3inputs_fis.h"
#include "Solar_F.h"


// PLL STRUCTURE
#if GRIDPHASE == GRID_1PH
    SPLL_1ph_SOGI_F spll1;
#elif GRIDPHASE == GRID_3PH
    SPLL_3ph_SRF_F spll1;
    ABC_DQ0_POS_F abc_dq0_pos1;     // grid voltage ABC_DQ0
    //ABC_DQ0_NEG_F abc_dq0_neg1;
    ABC_DQ0_POS_F abc_dq0_pos2;     // grid current ABC_DQ0
    //ABC_DQ0_NEG_F abc_dq0_neg2;
    DQ0_ABC_F dq0_abc1;             // grid voltage DQ0_ABC
#endif

#define GRID_FREQ           60
#define ISR_FREQUENCY       20000
#define ISR_PERIOD          0.00005
#define PI                  3.14159
#define tempo_atingido1          CpuTimer1Regs.TCR.bit.TIF

// NOTCH FILTER STRUCTURE
//NOTCH_COEFF_F notch_TwiceGridFreq;
//NOTCH_VARS_F Bus_Volt_notch;
//float notch_c1 = 0.1;
//float notch_c2 = 0.00001;

int comando_ligar = 0;

#define DEBUG 1 // Define DEBUG macro

// supercapacitors constants
#define SOC_SC_MIN          59.38
#define SOC_SC_MAX          95.625
#define SC_NOM_VOLTAGE      320

//UART constants
#define BUFFER_SIZE 8
unsigned char buffer_tx[8];

//------- DECLARACAO DE VARIAVEIS ----------
volatile uint16_t adcinA0;
volatile uint16_t adcinA1;
volatile uint16_t adcinA2;
volatile uint16_t adcinA3;
volatile uint16_t adcinA4;
volatile uint16_t adcinA5;
// Nao ha conexao na controlCARD de adcinA6, A7, B6, B7.
volatile uint16_t adcinB0;
volatile uint16_t adcinB1;
volatile uint16_t adcinB2;
volatile uint16_t adcinB3;
volatile uint16_t adcinB4;
volatile uint16_t adcinB5;

//float32 GridMeas = 0.0; // Grid measure in p.u (reading/peak_voltage_value)
float v_pcc[3];         // Tensao no ponto de conexao [Va, Vb, Vc]
float i_pcc[3];         // Corrente no ponto de conexao [Ia, Ib, Ic]

float iL_pv = 0;         // current boost
float iLdc_new = 0;     // filtered current boost
//float duty_boost = 0;   // duty cycle boost
//float duty_bid_sc = 0;   // duty cycle bidirectional buck-boost SUPERCAPACITOR (with complementary pwm)
//float duty_bid_bat = 0;  // duty cycle bidirectional buck-boost BATTERY (with complementary pwm)

//// mppt variables
//float p_pv_old = 0;     // PV power [n-1]
//float p_pv_new = 0;     // PV power [n]
float p_pv_new = 0;     // PV power [n]
float v_pv = 0;         // PV voltage [n]
float Vcc[2] = {0,0};   // Link CC voltage [n]
float v_pv_new = 0;     // PV filtered voltage [n]
//float v_pv_old = 0;     // PV filtered voltage [n-1]
//float delta_V = 2;      // voltage delta
//float v_pv_Max = 400;
//float v_pv_Min = 0;
//unsigned int count_mppt = 0;        // counter to enter mppt
//unsigned int mppt_period = 20000;   // runs mppt at each 500ms (0.5)/(2.5e-5)
//unsigned int boolStartMPPT = 0;     // boolean

// bidirectional buck-boost converter
float v_sc = 0;                         // SC voltage [n]
float iLsc = 0;                         // buckBoost inductor current [n]
float v_sc_new = 0;                     // SC filtered voltage [n]
float iLsc_new = 0;                     // SC filtered inductor current [n]
float iLbat_new = 0;                    // BAT filtered inductor current [n]
float SoC_est = 50;                     // State of Charge of the battery
float ESS_power_ref = 0.0;              // power reference for one ESS unit
float HESS_power_ref = 0.0;             // power reference for the HESS
uint32_t fis_counter = 0;
uint16_t MAsumVecPointer = 0;           // pointer to the current MAsumVec position

// Filter variables
filter lpf_iL_pv = filter_DEFAULTS;
filter lpf_v_pv = filter_DEFAULTS;
filter lpf_v_sc = filter_DEFAULTS;
filter lpf_iL_sc = filter_DEFAULTS;
filter lpf_iL_bat = filter_DEFAULTS;
filter lpf_v_cc = filter_DEFAULTS;

filter lpf_va = filter_DEFAULTS;
filter lpf_vb = filter_DEFAULTS;
filter lpf_vc = filter_DEFAULTS;
filter lpf_iLCL_a = filter_DEFAULTS;
filter lpf_iLCL_b = filter_DEFAULTS;
filter lpf_iLCL_c = filter_DEFAULTS;

// control variables
PIREG1 C_PV_iL = PIREG1_DEFAULTS;
PIREG1 C_PV_voltage = PIREG1_DEFAULTS;
PIREG1 C_SC = PIREG1_DEFAULTS;
PIREG1 C_BAT = PIREG1_DEFAULTS;
PIREG1 C_Vd_inv= PIREG1_DEFAULTS;
PIREG1 C_Vq_inv= PIREG1_DEFAULTS;
PIREG1 C_Id_inv= PIREG1_DEFAULTS;
PIREG1 C_Iq_inv= PIREG1_DEFAULTS;

//PID_GRANDO_F_CONTROLLER pid_grando_controller1;
//PID_GRANDO_F_CONTROLLER pid_grando_controller2;

// FIS variables
float inputs[3] = {3000.0, 50.0, 240.0};
float FIS_output;

// M.A. variable
float AvgPower = 0.0;               // variable that store the current Moving Average
float AvgPowerVAR = 0.0;            // variable that store the Moving Average to variability calc.
float std_dev = 0.0;                // standard deviation (variability) variable

#define FIS_PLOT_SIZE       700
#if DEBUG

int FIS_plot_counter = 0;
float FIS_plot[FIS_PLOT_SIZE];

#endif

//------------------------------------------

__interrupt void isr_cpu_timer0(void);
__interrupt void isr_adc(void);
void send_buffer_tx(void);


int main(void)
{


    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();
    EALLOW;
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    InitPieVectTable();

    setup_GPIO();
    setup_ePWM();
    setup_ADC_A();
    setup_ADC_B();
    Setup_UART();


    EALLOW;
    PieVectTable.TIMER0_INT = &isr_cpu_timer0;   // Atribui a funcao de interrupcao a ser executada
    PieVectTable.ADCA1_INT = &isr_adc;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;           // habilita o Timer 0 na tabela de interrupcoes
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;           // habilita o ADC na tabela de interrupcoes
    EDIS;
    IER |= M_INT1;// habilita a linha 1 da tabela de interrupcao (timer e adc)

    InitCpuTimers();
    CpuTimer0Regs.TCR.bit.TIF = 0;                      // Inicializa o Timer0
    CpuTimer0Regs.TCR.bit.TIE = 0;
    CpuTimer0Regs.TCR.bit.TSS = 1;
    //ConfigCpuTimer(&CpuTimer0, 200, 1000000); // configura o timer 0 para o clock de 200Mhz com periodo de 1 segundo.
    //CpuTimer0Regs.TCR.all = 0x4001;


    // const. low pass filter 2 khz
    lpf_iL_pv.a1 = -1.1429805;
    lpf_iL_pv.a2 = 0.4128016;
    lpf_iL_pv.a3 = 0;
    lpf_iL_pv.b0 = 0.06745527;
    lpf_iL_pv.b1 = 0.13491055;
    lpf_iL_pv.b2 = 0.06745527;
    lpf_iL_pv.b3 = 0;
    lpf_iL_pv.calc = (void (*)(unsigned int))filter_calc;
    lpf_iL_pv.reset = (void (*)(unsigned int))filter_reset;
    lpf_iL_pv.reset(&lpf_iL_pv);


    // const. low pass filter 2 khz
    lpf_v_pv.a1 = -1.1429805;
    lpf_v_pv.a2 = 0.4128016;
    lpf_v_pv.a3 = 0;
    lpf_v_pv.b0 = 0.06745527;
    lpf_v_pv.b1 = 0.13491055;
    lpf_v_pv.b2 = 0.06745527;
    lpf_v_pv.b3 = 0;
    lpf_v_pv.calc = (void (*)(unsigned int))filter_calc;
    lpf_v_pv.reset = (void (*)(unsigned int))filter_reset;
    lpf_v_pv.reset(&lpf_v_pv);


    // const. low pass filter 2 khz
    lpf_v_sc.a1 = -1.1429805;
    lpf_v_sc.a2 = 0.4128016;
    lpf_v_sc.a3 = 0;
    lpf_v_sc.b0 = 0.06745527;
    lpf_v_sc.b1 = 0.13491055;
    lpf_v_sc.b2 = 0.06745527;
    lpf_v_sc.b3 = 0;
    lpf_v_sc.calc = (void (*)(unsigned int))filter_calc;
    lpf_v_sc.reset = (void (*)(unsigned int))filter_reset;
    lpf_v_sc.reset(&lpf_v_sc);

    // const. low pass filter 2 khz
    lpf_iL_sc.a1 = -1.1429805;
    lpf_iL_sc.a2 = 0.4128016;
    lpf_iL_sc.a3 = 0;
    lpf_iL_sc.b0 = 0.06745527;
    lpf_iL_sc.b1 = 0.13491055;
    lpf_iL_sc.b2 = 0.06745527;
    lpf_iL_sc.b3 = 0;
    lpf_iL_sc.calc = (void (*)(unsigned int))filter_calc;
    lpf_iL_sc.reset = (void (*)(unsigned int))filter_reset;
    lpf_iL_sc.reset(&lpf_iL_sc);

    // const. low pass filter 2 khz
    lpf_iL_bat.a1 = -1.1429805;
    lpf_iL_bat.a2 = 0.4128016;
    lpf_iL_bat.a3 = 0;
    lpf_iL_bat.b0 = 0.06745527;
    lpf_iL_bat.b1 = 0.13491055;
    lpf_iL_bat.b2 = 0.06745527;
    lpf_iL_bat.b3 = 0;
    lpf_iL_bat.calc = (void (*)(unsigned int))filter_calc;
    lpf_iL_bat.reset = (void (*)(unsigned int))filter_reset;
    lpf_iL_bat.reset(&lpf_iL_bat);

    // const. low pass filter 2 khz
    lpf_v_cc.a1 = -1.1429805;
    lpf_v_cc.a2 = 0.4128016;
    lpf_v_cc.a3 = 0;
    lpf_v_cc.b0 = 0.06745527;
    lpf_v_cc.b1 = 0.13491055;
    lpf_v_cc.b2 = 0.06745527;
    lpf_v_cc.b3 = 0;
    lpf_v_cc.calc = (void (*)(unsigned int))filter_calc;
    lpf_v_cc.reset = (void (*)(unsigned int))filter_reset;
    lpf_v_cc.reset(&lpf_v_cc);

    // const. low pass filter 2 khz
    lpf_va.a1 = -1.1429805;
    lpf_va.a2 = 0.4128016;
    lpf_va.a3 = 0;
    lpf_va.b0 = 0.06745527;
    lpf_va.b1 = 0.13491055;
    lpf_va.b2 = 0.06745527;
    lpf_va.b3 = 0;
    lpf_va.calc = (void (*)(unsigned int))filter_calc;
    lpf_va.reset = (void (*)(unsigned int))filter_reset;
    lpf_va.reset(&lpf_va);

    // const. low pass filter 2 khz
    lpf_vb.a1 = -1.1429805;
    lpf_vb.a2 = 0.4128016;
    lpf_vb.a3 = 0;
    lpf_vb.b0 = 0.06745527;
    lpf_vb.b1 = 0.13491055;
    lpf_vb.b2 = 0.06745527;
    lpf_vb.b3 = 0;
    lpf_vb.calc = (void (*)(unsigned int))filter_calc;
    lpf_vb.reset = (void (*)(unsigned int))filter_reset;
    lpf_vb.reset(&lpf_vb);

    // const. low pass filter 2 khz
    lpf_vc.a1 = -1.1429805;
    lpf_vc.a2 = 0.4128016;
    lpf_vc.a3 = 0;
    lpf_vc.b0 = 0.06745527;
    lpf_vc.b1 = 0.13491055;
    lpf_vc.b2 = 0.06745527;
    lpf_vc.b3 = 0;
    lpf_vc.calc = (void (*)(unsigned int))filter_calc;
    lpf_vc.reset = (void (*)(unsigned int))filter_reset;
    lpf_vc.reset(&lpf_vc);


    // const. low pass filter 2 khz
    lpf_iLCL_a.a1 = -1.1429805;
    lpf_iLCL_a.a2 = 0.4128016;
    lpf_iLCL_a.a3 = 0;
    lpf_iLCL_a.b0 = 0.06745527;
    lpf_iLCL_a.b1 = 0.13491055;
    lpf_iLCL_a.b2 = 0.06745527;
    lpf_iLCL_a.b3 = 0;
    lpf_iLCL_a.calc = (void (*)(unsigned int))filter_calc;
    lpf_iLCL_a.reset = (void (*)(unsigned int))filter_reset;
    lpf_iLCL_a.reset(&lpf_iLCL_a);

    // const. low pass filter 2 khz
    lpf_iLCL_b.a1 = -1.1429805;
    lpf_iLCL_b.a2 = 0.4128016;
    lpf_iLCL_b.a3 = 0;
    lpf_iLCL_b.b0 = 0.06745527;
    lpf_iLCL_b.b1 = 0.13491055;
    lpf_iLCL_b.b2 = 0.06745527;
    lpf_iLCL_b.b3 = 0;
    lpf_iLCL_b.calc = (void (*)(unsigned int))filter_calc;
    lpf_iLCL_b.reset = (void (*)(unsigned int))filter_reset;
    lpf_iLCL_b.reset(&lpf_iLCL_b);

    // const. low pass filter 2 khz
    lpf_iLCL_c.a1 = -1.1429805;
    lpf_iLCL_c.a2 = 0.4128016;
    lpf_iLCL_c.a3 = 0;
    lpf_iLCL_c.b0 = 0.06745527;
    lpf_iLCL_c.b1 = 0.13491055;
    lpf_iLCL_c.b2 = 0.06745527;
    lpf_iLCL_c.b3 = 0;
    lpf_iLCL_c.calc = (void (*)(unsigned int))filter_calc;
    lpf_iLCL_c.reset = (void (*)(unsigned int))filter_reset;
    lpf_iLCL_c.reset(&lpf_iLCL_c);

    C_PV_iL.Kp = -0.055292;//-0.55292;
    C_PV_iL.Ki = -19.63121;//-196.3121;
    C_PV_iL.OutMax = 30; // max current reference
    C_PV_iL.OutMin = 2.4;  // min current reference
    C_PV_iL.UiMax = 60.0;
    C_PV_iL.UiMin = -60.0;
    C_PV_iL.Ts = ISR_PERIOD;//0.000025;//0.000025
    C_PV_iL.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_PV_iL.reset = (void (*)(unsigned int))pi_reg1_reset;

    //C_PV_voltage.Kp = 0.1131;//0.005655;
    C_PV_voltage.Kp = 0.01131;//0.005655;
    C_PV_voltage.Ki = 3.1416;//0.015708;
    C_PV_voltage.OutMax = 0.9;
    C_PV_voltage.OutMin = 0.1;
    C_PV_voltage.UiMax = 2.0;
    C_PV_voltage.UiMin = -2.0;
    C_PV_voltage.Ts = ISR_PERIOD;//0.000025;
    C_PV_voltage.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_PV_voltage.reset = (void (*)(unsigned int))pi_reg1_reset;

    //C_SC.Kp = 0.38282;//0.1413;
    C_SC.Kp = 0.038282;//0.1413;
    //C_SC.Ki = 140.3269;//31.08;
    C_SC.Ki = 14.03269;//31.08;
    C_SC.OutMax = 0.99;
    C_SC.OutMin = -0.99;
    C_SC.UiMax = 30.0;
    C_SC.UiMin = -30.0;
    C_SC.Ts = ISR_PERIOD;//0.000025;
    C_SC.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_SC.reset = (void (*)(unsigned int))pi_reg1_reset;

    //C_BAT.Kp = 0.05026;
    C_BAT.Kp = 0.005026;
    //C_BAT.Ki = 11.05;
    C_BAT.Ki = 0.1105;
    C_BAT.OutMax = 0.99;
    C_BAT.OutMin = -0.99;
    C_BAT.UiMax = 50.0;
    C_BAT.UiMin = -50.0;
    C_BAT.Ts = ISR_PERIOD;//0.000025;
    C_BAT.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_BAT.reset = (void (*)(unsigned int))pi_reg1_reset;

    // Malha interna PI (eixo direto)
    //C_Vd_inv.Kp = 0.6948;//1.3896;//2.3160;//4.1458;//6.9096;
    //C_Vd_inv.Ki = 691.1504;//1.3823e4;//2.3038e4;//1.3823e3;//3.5387e+03;
    C_Vd_inv.Kp = 4.1458;//2.7792;
    C_Vd_inv.Ki = 1.3823e3;
    C_Vd_inv.OutMax = 600;
    C_Vd_inv.OutMin = -600;
    C_Vd_inv.UiMax = 600;
    C_Vd_inv.UiMin = -600;
    C_Vd_inv.Ts = ISR_PERIOD;//0.000025; // 40kHz of sample time
    C_Vd_inv.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_Vd_inv.reset = (void (*)(unsigned int))pi_reg1_reset;

    // Malha interna PI (eixo quadratura)
    C_Vq_inv.Kp = C_Vd_inv.Kp;
    C_Vq_inv.Ki = C_Vd_inv.Ki;
    C_Vq_inv.OutMax = 400;
    C_Vq_inv.OutMin = -400;
    C_Vq_inv.UiMax = 400;
    C_Vq_inv.UiMin = -400;
    C_Vq_inv.Ts = ISR_PERIOD;//0.000025; // 40kHz of sample time
    C_Vq_inv.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_Vq_inv.reset = (void (*)(unsigned int))pi_reg1_reset;

    // [C_Id_inv] Controle do barramento cc
    C_Id_inv.Kp = -1.7772;//-2.1766;
    C_Id_inv.Ki = -203.0215;//-248.6495;
    C_Id_inv.OutMax = 70;
    C_Id_inv.OutMin = -70;
    C_Id_inv.UiMax = 100;
    C_Id_inv.UiMin = -100;
    C_Id_inv.Ts = ISR_PERIOD;//0.000025; // 40kHz of sample time
    C_Id_inv.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_Id_inv.reset = (void (*)(unsigned int))pi_reg1_reset;

    // [C_Iq_inv] Controle de potencia reativa
    C_Iq_inv.Kp = -0.3968e-03;//-5.0513e-04;
    C_Iq_inv.Ki = -0.4987;//-0.6348;
    C_Iq_inv.OutMax = 70;
    C_Iq_inv.OutMin = -70;
    C_Iq_inv.UiMax = 100;
    C_Iq_inv.UiMin = -100;
    C_Iq_inv.Ts = ISR_PERIOD;//0.000025; // 40kHz of sample time
    C_Iq_inv.calc = (void (*)(unsigned int))pi_reg1_calc;
    C_Iq_inv.reset = (void (*)(unsigned int))pi_reg1_reset;


    // SOGI PLL INITIALIZATION

    #if GRIDPHASE == GRID_1PH
        SPLL_1ph_SOGI_F_init(GRID_FREQ,((float)(1.0/ISR_FREQUENCY)),&spll1);
        SPLL_1ph_SOGI_F_coeff_update(((float)(1.0/ISR_FREQUENCY)),(float)(2*PI*GRID_FREQ),&spll1);
    #elif GRIDPHASE == GRID_3PH
        SPLL_3ph_SRF_F_init(GRID_FREQ,((float)(1.0/ISR_FREQUENCY)), &spll1);
        ABC_DQ0_POS_F_init(&abc_dq0_pos1);
        //ABC_DQ0_NEG_F_init(&abc_dq0_neg1);
        ABC_DQ0_POS_F_init(&abc_dq0_pos2);
        //ABC_DQ0_NEG_F_init(&abc_dq0_neg2);
        DQ0_ABC_F_init(&dq0_abc1);
    #endif

    // NOTCH FILTER INITIALIZATION
    //NOTCH_FLTR_F_VARS_init(&Bus_Volt_notch);
    //NOTCH_FLTR_F_COEFF_Update(((float)(1.0/ISR_FREQUENCY)),
    //(float)(2*PI*GRID_FREQ*2),(float)notch_c2,(float)notch_c1,
    //&notch_TwiceGridFreq);

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //GpioDataRegs.GPBDAT.bit.GPIO34 = 1; // set HIGH to GPIOB 34 (desliga o led, logica interna)
    //GpioDataRegs.GPADAT.bit.GPIO31 = 1; // set HIGH to GPIOA 31 (desliga o led, logica interna)

    // Step 6. IDLE loop. Just sit and loop forever (optional):

    // fuzzy logic initialization
    fis_EMS3inputs_init();

    comando_ligar = 1;
    limpa_temporizador1();
    temporizacao1(2);
    while(1)
    {
        fis_EMS3inputs_init();
        if((tempo_atingido1)){
            limpa_temporizador1();
            //fis_EMS3inputs_init();
            //fis_counter = 0;

            // divide FIS_output into MSB and LSB to send
            buffer_tx[0] = ( ( ((int)(FIS_output*10.0)) & 0xFF00 ) >> 8 );
            buffer_tx[1] = ( ((int)(FIS_output*10.0)) & 0x00FF );

            // divide MAsumVecPointer into MSB and LSB to send
            //buffer_tx[0] = ( ( ((int)(MAsumVecPointer)) & 0xFF00 ) >> 8 );
            //buffer_tx[1] = (int)(MAsumVecPointer);



            fis_EMS3inputs_run((float[]){HESS_power_ref, SoC_est, v_sc_new}, &FIS_output);  // leva entre 10ms a 30ms para calcular a FIS

            #if DEBUG
            FIS_plot[FIS_plot_counter] = FIS_output;
            FIS_plot_counter++;
            if(FIS_plot_counter >= FIS_PLOT_SIZE){
                FIS_plot_counter = 0;
            }
//            // divide FIS_output into MSB and LSB to send
//            buffer_tx[0] = ( ( ((int)(FIS_output*100.0)) & 0xFF00 ) >> 8 );
//            buffer_tx[1] = ( ((int)(FIS_output*100.0)) & 0x00FF );
            // divide AvgPower into MSB and LSB to send
//            buffer_tx[2] = ( ( ((int)AvgPower) & 0xFF00 ) >> 8 );
//            buffer_tx[3] = ( ((int)AvgPower) & 0x00FF );

            send_buffer_tx();
            #endif

            temporizacao1(0.5);
        }
    }
}

void send_buffer_tx(void){
    int i;
    for(i = 0; i < BUFFER_SIZE; i++){
        //while (SciaRegs.SCIFFTX.bit.TXFFST != 0){}
        while(!SciaRegs.SCICTL2.bit.TXRDY){
        }
        SciaRegs.SCITXBUF.all = buffer_tx[i];
    }
}

__interrupt void isr_cpu_timer0(void){
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void isr_adc(void){
    //GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1;

    // ADC CHANNEL A
    adcinA0 = AdcaResultRegs.ADCRESULT0;           // (Va_pcc)
    adcinA1 = AdcaResultRegs.ADCRESULT1;           // (Vb_pcc)
    adcinA2 = AdcaResultRegs.ADCRESULT2;           // (Vc_pcc)
    adcinA3 = AdcaResultRegs.ADCRESULT3;           // (V_PV)
    adcinA4 = AdcaResultRegs.ADCRESULT4;           // (V_SC)
    adcinA5 = AdcaResultRegs.ADCRESULT5;           // (V_cc)

    // ADC CHANNEL B
    adcinB0 = AdcbResultRegs.ADCRESULT0;           // (iL_a)
    adcinB1 = AdcbResultRegs.ADCRESULT1;           // (iL_b)
    adcinB2 = AdcbResultRegs.ADCRESULT2;           // (iL_c)
    adcinB3 = AdcbResultRegs.ADCRESULT3;           // (iL_PV)
    adcinB4 = AdcbResultRegs.ADCRESULT4;           // (iL_SC)
    adcinB5 = AdcbResultRegs.ADCRESULT5;           // (iL_Bat)

    // VOLTAGE CONVERSIONS
//    v_pcc[0] = adcinA0*0.1862 -376.71; // 75 V per 1Vdac
//    v_pcc[1] = adcinA1*0.1893 -370.6125; // 75 V per 1Vdac
//    v_pcc[2] = adcinA2*0.1888 -376.1103; // 75 V per 1Vdac

    lpf_va.x0 = adcinA0*0.1862 -376.71; // 75 V per 1Vdac
    lpf_va.calc(&lpf_va);
    v_pcc[0] = lpf_va.y0;

    lpf_vb.x0 = adcinA1*0.1893 -370.6125; // 75 V per 1Vdac
    lpf_vb.calc(&lpf_vb);
    v_pcc[1] = lpf_vb.y0;

    lpf_vc.x0 = adcinA2*0.1888 -376.1103; // 75 V per 1Vdac
    lpf_vc.calc(&lpf_vc);
    v_pcc[2] = lpf_vc.y0;


    //v_pv = adcinA3*0.2467 -500.1273; // 100 V per 1Vdac
    v_pv = adcinA3*0.2465 -498.0123; // 100 V per 1Vdac

    //Vcc[0] = adcinA5*0.2465 -500.0216; // 100 V per 1Vdac
    //Vcc[0] = adcinA5*0.2478 -501.2637; // 100 V per 1Vdac
    lpf_v_cc.x0 = adcinA5*0.2478 -501.2637; // 100 V per 1Vdac
    lpf_v_cc.calc(&lpf_v_cc);
    Vcc[0] = lpf_v_cc.y0;

//    Bus_Volt_notch.In = Vcc[0];
//    NOTCH_FLTR_F_run(&Bus_Volt_notch, &notch_TwiceGridFreq);
//    Vcc[1] = Bus_Volt_notch.Out;

    // CURRENT CONVERSIONS
    //i_pcc[0] = adcinB0*0.0368 -74.8002; // 15 A per 1Vdac
    //i_pcc[1] = adcinB1*0.0370 -74.7438; // 15 A per 1Vdac
    //i_pcc[2] = adcinB2*0.0365 -74.6188; // 15 A per 1Vdac

    lpf_iLCL_a.x0 = adcinB0*0.0368 -74.8002; // 15 A per 1Vdac
    lpf_iLCL_a.calc(&lpf_iLCL_a);
    i_pcc[0] = lpf_iLCL_a.y0;

    lpf_iLCL_b.x0 = adcinB1*0.0370 -74.7438; // 15 A per 1Vdac
    lpf_iLCL_b.calc(&lpf_iLCL_b);
    i_pcc[1] = lpf_iLCL_b.y0;

    lpf_iLCL_c.x0 = adcinB2*0.0365 -74.6188; // 15 A per 1Vdac
    lpf_iLCL_c.calc(&lpf_iLCL_c);
    i_pcc[2] = lpf_iLCL_c.y0;

    iL_pv = adcinB3*0.0367 -75.6124; // 15 A per 1Vdac

    lpf_iL_pv.x0 = iL_pv;
    lpf_iL_pv.calc(&lpf_iL_pv);
    iLdc_new = lpf_iL_pv.y0;

    lpf_v_pv.x0 = v_pv;
    lpf_v_pv.calc(&lpf_v_pv);
    v_pv_new = lpf_v_pv.y0;

    v_sc = adcinA4*0.0993 -1.0849;
    iLsc = adcinB4*0.0365 -75.2128;

    lpf_v_sc.x0 = v_sc;
    lpf_v_sc.calc(&lpf_v_sc);
    v_sc_new = lpf_v_sc.y0;

    lpf_iL_sc.x0 = iLsc;
    lpf_iL_sc.calc(&lpf_iL_sc);
    iLsc_new = lpf_iL_sc.y0;

    lpf_iL_bat.x0 = adcinB5*0.0367 -75.3759;
    lpf_iL_bat.calc(&lpf_iL_bat);
    iLbat_new = lpf_iL_bat.y0;

    maq_estados_inv();

    // divide AvgPower into MSB and LSB to send
    buffer_tx[2] = ( ( ((int)AvgPower) & 0xFF00 ) >> 8 );
    buffer_tx[3] = ( ((int)AvgPower) & 0x00FF );

    // divide std_dev into MSB and LSB to send
    buffer_tx[4] = ( ( ((int)std_dev) & 0xFF00 ) >> 8 );
    buffer_tx[5] = ( ((int)std_dev) & 0x00FF );

    buffer_tx[6] = ( ( ((int)p_pv_new) & 0xFF00 ) >> 8 );
    buffer_tx[7] = ( ((int)p_pv_new) & 0x00FF );

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear ADCINT1 flag for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Acknowledge interrupt to PIE
}
