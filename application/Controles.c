/*
 * Def_Controles.c
 *
 *  Created on: 10 de abr de 2024
 *      Author: Ever_
 */
#include "Controles.h"

//                                              VARIAVEIS, CONSTANTES E ESTRUTURAS
//#define plotsize 2000
//uint16_t plot[plotsize]; // plot vector for debug
uint32_t index = 0;
uint16_t conexao = 0; // DEBUG

#if GRIDPHASE == GRID_1PH
    extern    SPLL_1ph_SOGI_F spll1;
#elif GRIDPHASE == GRID_3PH
    extern    SPLL_3ph_SRF_F spll1;
    extern ABC_DQ0_POS_F abc_dq0_pos1;
    //extern ABC_DQ0_NEG_F abc_dq0_neg1;
    extern ABC_DQ0_POS_F abc_dq0_pos2;
    //extern ABC_DQ0_NEG_F abc_dq0_neg2;
    extern DQ0_ABC_F dq0_abc1;
#endif



//#define V_GRID_PEAK         311.124 //220.0*1.4142    // Vrms*sqrt(2)
#define V_GRID_PEAK         179.6 //127.0*1.4142    // Vrms*sqrt(2)
#define V_CC                400
#define V_SC_MIN            200
#define V_SC_MAX            320
#define V_BAT               210
#define GRID_FREQ           60
#define ISR_FREQUENCY       20000
#define ISR_PERIOD          0.00005
#define PI                  3.14159
#define T_PLL               0.155           // Tempo para assentamento do PLL durante a sincronizacao (aprox. 3 ciclos da rede)
//#define L                   0.00033         // LCL inductor value (L+Lg)
#define L                   0.00011058         // LCL inductor value (L+Lg)
#define WNL                 2*PI*L


extern float v_pcc[3];                    // Grid voltage ADC reading
extern float i_pcc[3];                    // Grid current ADC reading
float duty_a;                        // duty cycle for PWM A-S1
float duty_b;                        // duty cycle for PWM B-S1
float duty_c;                        // duty cycle for PWM C-S1
float C_Vd_inv_ref = 0;                // Vd direct reference
float C_Vq_inv_ref = 0;                // Vq quadrature reference

extern float Vcc[0];                        // Link CC voltage [n]
//extern NOTCH_COEFF_F notch_TwiceGridFreq;   // link CC notch filter structure
//extern NOTCH_VARS_F Bus_Volt_notch;         // link CC notch filter structure


//extern    PID_GRANDO_F_CONTROLLER pid_grando_controller1;
//extern    PID_GRANDO_F_CONTROLLER pid_grando_controller2;
//                                              VARIAVEIS DE ESTADOS

ESTADOS_t estado_atual = FORA_DE_OPERACAO;
ESTADOS_t estado_anterior;
extern int comando_ligar;

MPPTstates_t estado_MPPT_atual = MPPT_OFF;
MPPTstates_t estado_MPPT_anterior = MPPT_OFF;
extern float p_pv_new;
extern float iLdc_new;
extern float v_pv_new;

float v_pv_ref_FORCED = 0;
int boost_enable = 0;
int SC_enable = 0;
float sc_duty_cycle_FORCED = 0.0;
float BAT_duty_cycle_FORCED = 0.0;
//float iLdc = 0;        // current boost
extern float iLdc_new;   // filtered current boost
float duty_boost = 0;    // duty cycle boost
float duty_bid_sc = 0;   // duty cycle bidirectional buck-boost SUPERCAPACITOR (with complementary pwm)
float duty_bid_bat = 0;  // duty cycle bidirectional buck-boost BATTERY (with complementary pwm)
extern float FIS_output;


// mppt variables
float p_pv_old = 0;     // PV power [n-1]
extern float p_pv_new;     // PV power [n]
extern float v_pv;         // PV voltage [n]
extern float v_pv_new;     // PV filtered voltage [n]
float v_pv_old = 0;     // PV filtered voltage [n-1]
float delta_V = 0.5;      // voltage delta
float v_pv_Max = 400;
float v_pv_Min = 320;
unsigned int count_mppt = 0;        // counter to enter mppt
unsigned int mppt_period = 2000;   // runs mppt at each 500ms (0.1)/(5e-5)
unsigned int boolStartMPPT = 0;     // boolean

// bidirectional buck-boost converter
//float v_sc = 0;                           // SC voltage [n]
//float iLsc = 0;                           // buckBoost inductor current [n]
extern float v_sc_new;                      // SC filtered voltage [n]
extern float iLsc_new;                      // SC filtered inductor current [n]
float iLsc_ref = 0;                         // Current reference

//extern float v_bat_new;                    // BAT filtered voltage [n]
extern float iLbat_new;                     // BAT filtered inductor current [n]
float iLbat_ref = 0;                        // Current reference
extern float SoC_est;                       // Estado de carga da bateria
float C_nominal = 100.0;                    // Capacidade nominal da bateria em Ah
float carga_consumida = 0.0;
extern float ESS_power_ref;
extern float HESS_power_ref;
float SC_power_ref = 0.0;               // power reference for the SC
float BAT_power_ref = 0.0;              // power reference for the Battery
extern float FIS_output;

// control variables
extern PIREG1 C_PV_iL;                  // PV boost current controller
extern PIREG1 C_PV_voltage;             // PV boost voltage controller
extern PIREG1 C_SC;                     // SC bDir current controller
extern PIREG1 C_BAT;                    // BAT bDir current controller
extern PIREG1 C_Vd_inv;              // Inverter voltage controller (internal loop direct axis)
extern PIREG1 C_Vq_inv;              // Inverter voltage controller (internal loop quadrature axis)
extern PIREG1 C_Id_inv;              // Inverter current controller (external loop link CC)
extern PIREG1 C_Iq_inv;              // Inverter current controller (external loop Q)
unsigned int count_control = 0; // counter to enter control
unsigned int control_period = 2; // runs control at 20 khz

// Moving average variables
float sumAll = 0.0;                     // whole sum of PowerVec[]
uint16_t MpCount = 1;                   // counter of how much values have entered the vector
uint16_t avgCounter = 0;                // counter to sample P_PV at a specific period
uint16_t varCounter = 0;                // counter to sample P_PV at a specific period
uint16_t MpPointer = 0;                 // pointer to the tail of PowerVec (AvgPower)
uint16_t VARPointer = 0;                // pointer to the tail of PowerVec (AvgPowerVAR over MAX_PERIOD)
float PowerVec [MAX_PERIOD] = {0};   // vector who stores P_PV
float MeanVec [MAX_PERIOD] = {0};    // vector who stores AvgPowerVAR
extern float AvgPower;                  // current Moving Average
extern float AvgPowerVAR;               // variability Moving Average
float MA_sum = 0.0;                     // current sum of point to the Moving Average
float VAR_sum = 0.0;                    // variability sum
float AvgPowerVAR_old = 0.0;
extern float std_dev;                   // standard deviation variable
float sum_sq_diff = 0.0;                // sum for std_dev

uint16_t MAsumVec[MA_POINTS] = {0.0};               // array of sum of points to the Moving Average

//                                 VARIAVEIS CONTROLE DO INVERSOR
volatile float sin_th = 0.0;
volatile float cos_th = 1.0;


/* Function: MPPT
 * ----------------------------------------------
 * Maximum Power Point Tracking algorithm
 *
 * returns:      0 if v_pv_new is less than the minimum necessary and does not compute
 */
int MPPT(){
    switch (estado_MPPT_atual){
        case MPPT_ON:
            p_pv_new = 0.5*(p_pv_new + iLdc_new*v_pv_new);
            count_mppt++; // incrementa o contador
            if (count_mppt >= mppt_period){
                //p_pv_new = iLdc_new * v_pv_new;
                count_mppt = 0;
                if(estado_MPPT_anterior == MPPT_OFF){ // primeira iteracao do MPPT
                    C_PV_iL.Ref = v_pv_Min; //v_pv_new;
                    estado_MPPT_anterior = estado_MPPT_atual;
                }

                if(p_pv_new > p_pv_old){ // todo: '>' or '>='
                    if(v_pv_new > v_pv_old){
                        C_PV_iL.Ref += delta_V; // mppt voltage reference
                    }else{
                        C_PV_iL.Ref -= delta_V;
                    }
                }
                else if(p_pv_new < p_pv_old){
                    if(v_pv_new > v_pv_old){
                        C_PV_iL.Ref -= delta_V;
                    }else{
                        C_PV_iL.Ref += delta_V;
                    }
                }
                p_pv_old = p_pv_new;    // update the previous power value
                v_pv_old = v_pv_new;    // update the previous voltage value

            }
            break;
        case MPPT_OFF:
            count_mppt = 0;
            C_PV_iL.reset(C_PV_iL);
            C_PV_voltage.reset(C_PV_voltage);
            C_PV_iL.Out = C_PV_iL.OutMin;
            C_PV_voltage.Out = C_PV_voltage.OutMin;
            duty_boost = 0;
            break;
    }
    return 1;
}

/* Function: controle_boost_paineisPV
 * -------------------------------------------------------
 * Computes the duty cycle of the PV panel boost converter
 * v_pv_ref:    PV voltage reference for the controler
 *              0 to use the MPPT value.
 *
 * enable:      1-enable the controller PWM, 0-disable the PWM.
 */
void controle_boost_paineisPV(float v_pv_ref, int enable){
    // PWM boost
    if(enable){
        if(v_pv_ref > 0.0){
            C_PV_iL.Ref = v_pv_ref; // if there is a v_pv_ref value, use it instead of the MPPT reference
        }
        // voltage reference threshold for the PI controller.
        if(C_PV_iL.Ref > v_pv_Max) C_PV_iL.Ref = v_pv_Max;
        else if(C_PV_iL.Ref < v_pv_Min) C_PV_iL.Ref = v_pv_Min;
        // calculo da malha externa
        C_PV_iL.Fdb = v_pv_new;
        C_PV_iL.calc(&C_PV_iL);
        // calculo da malha interna
        C_PV_voltage.Fdb = iLdc_new;
        C_PV_voltage.Ref = C_PV_iL.Out;
        C_PV_voltage.calc(&C_PV_voltage);
        duty_boost = C_PV_voltage.Out;

        EPwm1Regs.CMPB.bit.CMPB = (uint16_t)(duty_boost *((float)EPwm1Regs.TBPRD));
        //EPwm5Regs.CMPA.bit.CMPA = (uint16_t)(duty_boost *((float)EPwm5Regs.TBPRD));
    }else{
        EPwm1Regs.CMPB.bit.CMPB = (uint16_t)(0 *((float)EPwm1Regs.TBPRD));
        //EPwm5Regs.CMPA.bit.CMPA = (uint16_t)(0 *((float)EPwm5Regs.TBPRD));
        //todo: reset controllers
    }
}

/* Function: update_MAsumVec
 * -----------------------------------------------------
 *
 */
void update_MAsumVec(){
    uint16_t index;
    uint16_t current;
    uint16_t var;
    for (var = 0; var < MA_POINTS; var++){
        current = ((var+1)*120);
        index = (VARPointer < current) ? (MAX_PERIOD-(current-VARPointer)) : (VARPointer-current);
        MAsumVec[var] += p_pv_new - PowerVec[index];
    }
}
/* Function: moving_average
 * -----------------------------------------------------
 * Computes the moving average of the PV power generated
 *
 * Computes the Variability (standard deviation of moving average)
 */
void moving_average(){
    //p_pv_new = iLdc_new * v_pv_new; // calculated in MPPT() func.
    if(avgCounter >= AVG_COUNTER_LEN){
        avgCounter = 0;

        if(MpCount <= MA_CURRENT){ // loading the Power array (first interaction)
            MA_sum += p_pv_new;
            AvgPower = MA_sum/MpCount;
            MpCount++;

            VAR_sum = MA_sum;
            AvgPowerVAR = AvgPower;
            sum_sq_diff += ( p_pv_new -  AvgPowerVAR ) * ( p_pv_new - AvgPowerVAR );
        }else{
            MpPointer = (VARPointer < MA_CURRENT) ? (MAX_PERIOD-(MA_CURRENT-VARPointer)) : (VARPointer-MA_CURRENT);
            MA_sum = MA_sum + p_pv_new - PowerVec[MpPointer];
            AvgPower = MA_sum/MA_CURRENT; // current M.A (over MP

            //variablility M.A. (over MAX_PERIOD)
            if(MpCount <= MAX_PERIOD){ // loading the Power array (first interaction)
                VAR_sum += p_pv_new;
                AvgPowerVAR = VAR_sum/MpCount;

                sum_sq_diff += ( p_pv_new - AvgPowerVAR ) * ( p_pv_new - AvgPowerVAR );
                std_dev = sqrt(sum_sq_diff / MpCount);

                MpCount++;
            }else{
                VAR_sum += p_pv_new - PowerVec[VARPointer];
                AvgPowerVAR = VAR_sum/MAX_PERIOD;

                sum_sq_diff -= ( PowerVec[VARPointer] - MeanVec[VARPointer]) * ( PowerVec[VARPointer] - MeanVec[VARPointer]);
                //sum_sq_diff -= ((uint16_t) PowerVec[VARPointer] - AvgPowerVAR_old) * ((uint16_t) PowerVec[VARPointer] - AvgPowerVAR_old);
                sum_sq_diff += ( p_pv_new - AvgPowerVAR ) * (p_pv_new - AvgPowerVAR );
                std_dev = sqrt(sum_sq_diff / MAX_PERIOD);
            }
        }
        if(sum_sq_diff < 0.0){
            sum_sq_diff = 0.0;
        }

        PowerVec[VARPointer] = p_pv_new;                                 // last value in PowerVec is replaced for new p_pv
        MeanVec[VARPointer] = AvgPowerVAR;
        AvgPowerVAR_old = AvgPowerVAR;
        VARPointer = (VARPointer < (MAX_PERIOD-1) ) ? VARPointer+1 : 0;    // update the pointer for the PowerVec tail (AvgPower)
    }else{
        avgCounter++;
    }
}


/* Function: controle_bidirecional_sc
 * ---------------------------------------------------------
 * Bidirectional buck-boost controller (supercapacitor bank)
 *
 *  EPWM-5A typhoon DO28
 *  complementary EPWM-5B typhoon DO22
 *
 * duty_cycle:    duty cycle value forced to the converter
 *              0.0 to use the controller calculated value.
 *
 * enable:      1-enable the controller PWM, 0-disable the PWM.
 */
void controle_bidirecional_sc(float duty_cycle, int enable){

    if(duty_cycle > 0.0){
        duty_bid_sc = duty_cycle;
    }else{
        C_SC.Fdb = iLsc_new;
        C_SC.Ref = iLsc_ref;
        C_SC.calc(&C_SC);
        duty_bid_sc = C_SC.Out + (v_sc_new/Vcc[0]);

        if(duty_bid_sc > 0.99) duty_bid_sc = 0.99;
        else if(duty_bid_sc < 0) duty_bid_sc = 0;
    }

    if(enable){
        GPIO_WritePin(13, 1);

        EPwm5Regs.CMPA.bit.CMPA = (uint16_t)(duty_bid_sc *((float)EPwm5Regs.TBPRD));
    }else{
        C_SC.reset(C_SC);
        GPIO_WritePin(13, 0);                    // Set GPIO pin low (disable IGBT)
    }
}

/* Function: controle_bidirecional_bat
 * ---------------------------------------------------------
 * Bidirectional buck-boost controller (battery bank)
 *
 *  EPWM-6A typhoon DO27
 *  complementary EPWM-6B typhoon DO21
 *
 * duty_cycle:    duty cycle value forced to the converter
 *              0.0 to use the controller calculated value.
 *
 * enable:      1-enable the controller PWM, 0-disable the PWM.
 */
void controle_bidirecional_bat(float duty_cycle, int enable){

    if(duty_cycle > 0.0){
        duty_bid_bat = duty_cycle;
    }else{
        C_BAT.Fdb = iLbat_new;
        C_BAT.Ref = iLbat_ref;
        C_BAT.calc(&C_BAT);
        duty_bid_bat = C_BAT.Out + (V_BAT/Vcc[0]);

        if(duty_bid_bat > 0.99) duty_bid_bat = 0.99;
        else if(duty_bid_bat < 0) duty_bid_bat = 0;
    }

    if(enable){
        GPIO_WritePin(13, 1);

        EPwm6Regs.CMPA.bit.CMPA = (uint16_t)(duty_bid_bat *((float)EPwm6Regs.TBPRD));
    }else{
        C_BAT.reset(C_BAT);
        GPIO_WritePin(13, 0);                    // Set GPIO pin low (disable IGBT)
    }
}
/* Function: estimador_SoC
 * ---------------------------------------------------------
 * Coulomb Counting method
 *
 */
void estimador_SoC(){
    carga_consumida += iLbat_new * 0.00005/3600.0; // convertendo corrente em A para Ah
    //SoC_est += ((float) (carga_consumida/C_nominal)*100.0);
    if ((carga_consumida > 0.01) || (carga_consumida < -0.01)){
        SoC_est += carga_consumida;
        carga_consumida = 0.0;
    }

    // Limitar SoC entre 0 e 100%
    if (SoC_est < 0.0) SoC_est = 0.0;
    if (SoC_est > 100.0) SoC_est = 100.0;
}

void gera_referencia(){
    ESS_power_ref = p_pv_new - AvgPower;                  // Compute the power reference for the ESS
    iLsc_ref = (v_sc_new > 0.0) ? ESS_power_ref/v_sc_new : 0;   // Compute the current ref. for the SC controller

    if((v_sc_new < V_SC_MIN ) && (iLsc_ref < 0.0)){
        iLsc_ref = 0.0; // dont let the SC discharge below V_SC_MIN
    }
    if((v_sc_new > V_SC_MAX ) && (iLsc_ref > 0.0)){
        iLsc_ref = 0.0; // dont let the SC charge above V_SC_MAX
    }

}

void gera_referencia_HESS(){
    HESS_power_ref = p_pv_new - AvgPower;                  // Compute the power reference for the HESS

    // calcula a referencia de potencia para os armazenadores
    if((FIS_output > 2.3) || (FIS_output < -2.3)){
        SC_power_ref = 0;
        BAT_power_ref = HESS_power_ref;
    }else if((-1.3 < FIS_output) && (FIS_output < 1.3)){
        SC_power_ref = HESS_power_ref;
        BAT_power_ref = 0;
    }else{
        SC_power_ref = (FIS_output > 0.0) ? HESS_power_ref*(1.0 -(FIS_output-1.3)) : HESS_power_ref*(1.0 -(-1.3 -FIS_output));
        BAT_power_ref = (FIS_output > 0.0) ? HESS_power_ref*(FIS_output-1.3) : HESS_power_ref*(-1.3 -FIS_output);
    }
    // calcura a referencia de corrente para o SC
    iLsc_ref = (v_sc_new > 0.0) ? SC_power_ref/v_sc_new : 0;   // Compute the current ref. for the SC controller

        if((v_sc_new < V_SC_MIN ) && (iLsc_ref < 0.0)){
            iLsc_ref = 0.0; // dont let the SC discharge below V_SC_MIN
        }
        if((v_sc_new > V_SC_MAX ) && (iLsc_ref > 0.0)){
            iLsc_ref = 0.0; // dont let the SC charge above V_SC_MAX
        }

    // calcura a referencia de corrente para a BAT
    iLbat_ref = (SoC_est > 15.0) ? BAT_power_ref/V_BAT : 0;   // Compute the current ref. for the BAT controller

}

void PLL()
{
#if GRIDPHASE == GRID_1PH
    spll1.u[0] = (float)(v_pcc[0]/V_GRID_PEAK); //v_grid in p.u
    // SPLL call
    SPLL_1ph_SOGI_F_FUNC(&spll1);
    //plot[index] = (uint16_t)(100.0*(1.0 + spll1.sin)); // plot sin(w) in debug
#elif GRIDPHASE == GRID_3PH
    //plot[index] = (uint16_t)((180 + v_pcc[0])); // plot sin(w) in debug
    abc_dq0_pos1.a = (v_pcc[0]/V_GRID_PEAK);
    abc_dq0_pos1.b = (v_pcc[1]/V_GRID_PEAK);
    abc_dq0_pos1.c = (v_pcc[2]/V_GRID_PEAK);
    sin_th = (float)__relaxed_sin(spll1.theta[1]); //stored to be used again in ISR ?theta[0]ORtheta[1]?
    cos_th = (float)__relaxed_cos(spll1.theta[1]); //stored to be used again in ISR
    abc_dq0_pos1.sin = sin_th;
    abc_dq0_pos1.cos = cos_th;
    ABC_DQ0_POS_F_MACRO(abc_dq0_pos1);
    spll1.v_q[0] = (abc_dq0_pos1.q);
    // SPLL call
    SPLL_3ph_SRF_F_FUNC(&spll1);
#endif
}

void temporizacao(float tempo){
    ConfigCpuTimer(&CpuTimer0, 200, 1000000*tempo);             // Configura o Timer0 para contar ate o tempo especificado em SEGUNDOS, considerando um clock de 200MHz
    CpuTimer0Regs.TCR.bit.TIF = 0;                              // Limpa a flag de interrupcao do Timer0
    CpuTimer0Regs.TCR.bit.TIE = 1;                              // Habilita a interrupcao do Timer0 (necessaria para setar a flag ao fim da temporizacao
    CpuTimer0Regs.TCR.bit.TSS = 0;                              // Retira o Timer0 do estado parado, iniciando a contagem
}

void temporizacao1(float tempo){
    ConfigCpuTimer(&CpuTimer1, 200, 1000000*tempo);             // Configura o Timer0 para contar ate o tempo especificado em SEGUNDOS, considerando um clock de 200MHz
    CpuTimer1Regs.TCR.bit.TIF = 0;                              // Limpa a flag de interrupcao do Timer0
    CpuTimer1Regs.TCR.bit.TIE = 1;                              // Habilita a interrupcao do Timer0 (necessaria para setar a flag ao fim da temporizacao
    CpuTimer1Regs.TCR.bit.TSS = 0;                              // Retira o Timer0 do estado parado, iniciando a contagem
}
void limpa_temporizador1(){
    EALLOW;
    CpuTimer1Regs.TCR.bit.TIF = 0;                              // Limpa a flag de interrupcao do Timer0
    CpuTimer1Regs.TCR.bit.TIE = 0;                              // Desabilita a interrupcao do Timer0 ate o proximo uso da funcao temporizacao
    CpuTimer1Regs.TCR.bit.TSS = 1;                              // Mantem o Timer0 parado
    EDIS;
}

void limpa_temporizador(){
    EALLOW;
    CpuTimer0Regs.TCR.bit.TIF = 0;                              // Limpa a flag de interrupcao do Timer0
    CpuTimer0Regs.TCR.bit.TIE = 0;                              // Desabilita a interrupcao do Timer0 ate o proximo uso da funcao temporizacao
    CpuTimer0Regs.TCR.bit.TSS = 1;                              // Mantem o Timer0 parado
    EDIS;
}

int detecta_zero(float sig_det_0)
{                                                                  // Funcao para detectar a passagem por zero de um sinal
    if((sig_det_0<=TRESHOLD_ZERO)&&(sig_det_0>=-TRESHOLD_ZERO)){   // Se o sinal encontrar-se dentro da banda de deteccao, retorna 1
        return(1);
    }
    else{                                                          // Se nao, retorna 0
        return(0);
    }
}

void modulacao_3_niveis(void)
{
    abc_dq0_pos2.a = (i_pcc[0]); // p.u: divided by 8800/127
    abc_dq0_pos2.b = (i_pcc[1]);
    abc_dq0_pos2.c = (i_pcc[2]);
    abc_dq0_pos2.sin = sin_th;
    abc_dq0_pos2.cos = cos_th;
    ABC_DQ0_POS_F_MACRO(abc_dq0_pos2);

    //[C_Vd_inv]
    C_Vd_inv.Fdb = abc_dq0_pos2.d; // grid current direct axis reference
    C_Vd_inv.Ref = C_Vd_inv_ref;
    //C_Vd_inv.calc(&C_Vd_inv);


    dq0_abc1.d =C_Vd_inv_ref;//C_Vd_inv.Out + abc_dq0_pos1.d - abc_dq0_pos2.q*WNL;

    //[C_Vq_inv]
    C_Vq_inv.Fdb = abc_dq0_pos2.q; // grid current quadrature axis reference
    C_Vq_inv.Ref = C_Vq_inv_ref;
    //C_Vq_inv.calc(&C_Vq_inv);


    dq0_abc1.q = C_Vq_inv_ref;//C_Vq_inv.Out + abc_dq0_pos1.q + abc_dq0_pos2.d*WNL;
    dq0_abc1.z =0;
    dq0_abc1.__relaxed_cos = cos_th;
    dq0_abc1.__relaxed_sin = sin_th;
    DQ0_ABC_F_FUNC(&dq0_abc1);

    duty_a = dq0_abc1.a/V_CC; // V_CC
    duty_b = dq0_abc1.b/V_CC;
    duty_c = dq0_abc1.c/V_CC;

    //plot[index] = (uint16_t)(180*(1 + duty_c)); // plot sin(w) in debug

    if(duty_a > 0.99) duty_a = 0.99;
    else if(duty_a < -0.99) duty_a = -0.99;
    if(duty_b > 0.99) duty_b = 0.99;
    else if(duty_b < -0.99) duty_b = -0.99;
    if(duty_c > 0.99) duty_c = 0.99;
    else if(duty_c < -0.99) duty_c = -0.99;

    //SVPWM(&duty_a, &duty_b, &duty_c);

    EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((duty_a+1.0)*0.5*((float)EPwm2Regs.TBPRD));
    EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((duty_b+1.0)*0.5*((float)EPwm3Regs.TBPRD));
    EPwm4Regs.CMPA.bit.CMPA =  (uint16_t)((duty_c+1.0)*0.5*((float)EPwm4Regs.TBPRD));

}

void SVPWM(float *da, float *db, float *dc){
    float dmin = 0;
    float dmax = 0;
    //Calc da seq zero para o SVPWM
    if(*da < *db && *da < *dc && *db>*dc){
      dmin = *da;
      dmax = *db;
    }else if((*da < *db) && (*da < *dc) && (*dc >*db)){
      dmin = *da;
      dmax = *dc;
    }else if((*db<*da) && (*db<*dc) && (*da>*dc)){
      dmin = *db;
      dmax = *da;
    }else if((*db<*da) && (*db<*dc) && (*dc>*da)){
      dmin = *db;
      dmax = *dc;
    }else if((*dc<*da) && (*dc<*db) && (*da>*db)){
      dmin = *dc;
      dmax = *da;
    }else if((*dc<*da) && (*dc<*db) && (*db>*da)){
      dmin = *dc;
      dmax = *db;
    }
    *da = -0.5*(dmin+dmax)+ *da;
    *db = -0.5*(dmin+dmax)+ *db;
    *dc = -0.5*(dmin+dmax)+ *dc;
}

/*  Malha interna de controle (Vd/Vq -> ABC)
**
**  Realiza a transformada ABD->DQ das correntes Ia,Ib,Ic
**  PI do eixo D
**  PI do eixo Q
**  Transformada DQ->ABC do resultado
**  Ganho sqrt(3)/Vdc
**
**  Output: Duty cycle for each 3-phase inverter leg.
*/
void controle_inv(){
    //GRID CURRENTS ABC-DQ
    abc_dq0_pos2.a = (i_pcc[0]);
    abc_dq0_pos2.b = (i_pcc[1]);
    abc_dq0_pos2.c = (i_pcc[2]);
    abc_dq0_pos2.sin = sin_th;
    abc_dq0_pos2.cos = cos_th;
    ABC_DQ0_POS_F_MACRO(abc_dq0_pos2);

    //MALHA EXTERNA
    //[C_Id_inv]
    C_Id_inv.Fdb = Vcc[0]; // Link CC voltage reference
    C_Id_inv.Ref = V_CC;
    C_Id_inv.calc(&C_Id_inv);
    C_Vd_inv_ref = C_Id_inv.Out;

    //[C_Iq_inv]
    C_Iq_inv.Fdb = -1.5*(abc_dq0_pos1.d * abc_dq0_pos2.q); // Q axis reference
    C_Iq_inv.Ref = 0;
    C_Iq_inv.calc(&C_Iq_inv);
    C_Vq_inv_ref = C_Iq_inv.Out;

    //MALHA INTERNA
    //[C_Vd_inv]
    C_Vd_inv.Fdb = abc_dq0_pos2.d; // grid current direct axis reference
    C_Vd_inv.Ref = C_Vd_inv_ref;
    C_Vd_inv.calc(&C_Vd_inv);


    dq0_abc1.d = C_Vd_inv.Out + abc_dq0_pos1.d - abc_dq0_pos2.q*WNL;

    //[C_Vq_inv]
    C_Vq_inv.Fdb = abc_dq0_pos2.q; // grid current quadrature axis reference
    C_Vq_inv.Ref = C_Vq_inv_ref;
    C_Vq_inv.calc(&C_Vq_inv);


    dq0_abc1.q = C_Vq_inv.Out + abc_dq0_pos1.q + abc_dq0_pos2.d*WNL;
    dq0_abc1.z =0;
    dq0_abc1.__relaxed_cos = cos_th;
    dq0_abc1.__relaxed_sin = sin_th;
    DQ0_ABC_F_FUNC(&dq0_abc1);

    duty_a = dq0_abc1.a/V_CC;
    duty_b = dq0_abc1.b/V_CC;
    duty_c = dq0_abc1.c/V_CC;

    if(duty_a > 0.99) duty_a = 0.99;
    else if(duty_a < -0.99) duty_a = -0.99;
    if(duty_b > 0.99) duty_b = 0.99;
    else if(duty_b < -0.99) duty_b = -0.99;
    if(duty_c > 0.99) duty_c = 0.99;
    else if(duty_c < -0.99) duty_c = -0.99;

    //SVPWM(&duty_a, &duty_b, &duty_c);

    EPwm2Regs.CMPA.bit.CMPA =  (uint16_t)((duty_a+1.0)*0.5*((float)EPwm2Regs.TBPRD));
    EPwm3Regs.CMPA.bit.CMPA =  (uint16_t)((duty_b+1.0)*0.5*((float)EPwm3Regs.TBPRD));
    EPwm4Regs.CMPA.bit.CMPA =  (uint16_t)((duty_c+1.0)*0.5*((float)EPwm4Regs.TBPRD));

}

//======================================================================================================================================================
void maq_estados_inv()
{
    //index = (index == plotsize) ? 0 : (index+1); // index counter for plot signal in debug
    switch(estado_atual){
            case FORA_DE_OPERACAO:                                              // Neste estado, o inversor esta completamente desativado e desconectado
                if(comando_ligar==LIGADO){                                      // Ao receber um comando de LIGAR, inicia a operacao
                    estado_anterior = estado_atual;
                    estado_atual = SINCRONIZANDO;                               // A primeira etapa da operacao e o sincronismo com a rede
                    temporizacao(T_PLL);                                        // Inicia a funcao de temporizacao para permitir o assentamento do PLL
                }
                break;

            case SINCRONIZANDO:                                                 // Permanece executando apenas o PLL para sincronizar (ainda desconectado)
                PLL();

                if(tempo_atingido){                                  // Ao atinigir o tempo de assentamento, indicado pela flag do Timer0, limpa o timer e passa para a PRE-CARGA do filtro LCL
                    limpa_temporizador();
                    estado_anterior = estado_atual;
                    C_Vd_inv_ref = 360;
                    estado_atual = PRE_CARGA;
                    temporizacao(T_Pre_carga);
                }
                break;

            case PRE_CARGA:
                PLL();
                modulacao_3_niveis();

                //if(detecta_zero(v_pcc[0])&&(tempo_atingido)){
                //if((conexao) && (tempo_atingido)){
                if((tempo_atingido)){
                    limpa_temporizador();
                    C_Vd_inv_ref = 0;
                    Disj_PCC = FECHADO;

                    estado_anterior = estado_atual;
                    estado_atual = CONECTADO;
                }
                break;

            case CONECTADO:
                PLL();
                controle_inv();
                estado_MPPT_atual = MPPT_ON;
                MPPT();
                boost_enable = 1;
                controle_boost_paineisPV(v_pv_ref_FORCED, boost_enable);
                moving_average();
                //gera_referencia();
                gera_referencia_HESS();
                //SC_enable = 1;
                controle_bidirecional_sc(sc_duty_cycle_FORCED, SC_enable);
                estimador_SoC();
                controle_bidirecional_bat(BAT_duty_cycle_FORCED, SC_enable);

                break;
    }
}

