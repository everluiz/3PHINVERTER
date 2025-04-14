/*
 * Controles.h
 *
 *  Created on: 10 de abr de 2024
 *      Author: Ever_
 */

#ifndef APPLICATION_CONTROLES_H_
#define APPLICATION_CONTROLES_H_
#include "F28x_Project.h"
#include "F2837xD_Device.h"
#include "F2837xD_GlobalPrototypes.h"
#include "Solar_F.h"
#include "F2837xD_epwm.h"
#include "setup.h"
#include "filter.h"
#include "PI_REG1.h"

//                              CONSTANTES GERAIS
// inverter const
#define GRID_1PH    0
#define GRID_3PH    1
#define GRIDPHASE   GRID_3PH

// avgPeriod = 500e-3 (500 milliseconds)
// isr_adcPerod = 1/20000 = 5e-5
// AVG_COUNTER_LEN = avgPeriod/isr_adcPerod = 20000
#define AVG_COUNTER_LEN         10000                                        // contador para amostragem da media movel

// Example:
// timeframe = 600 (seconds ( 10*60 -> 10 minutes))
// MP_VALUE = timeframe / avgPeriod = 1200

// MA_CURRENT = 200 is 100 seconds of MA window
#define MA_CURRENT              200                                      // tamanho do vetor de pontos da media movel
#define MAX_PERIOD              1200                                     // 600 seconds
#define MIN_PERIOD              120                                      // 60 seconds
#define MA_POINTS               11
#define THRESHOLDS              MA_POINTS-1

#define tempo_atingido          CpuTimer0Regs.TCR.bit.TIF
#define T_Pre_carga             0.1                                         // Tempo minimo para assentamento do PLL no estagio de sincronizacao
#define TRESHOLD_ZERO           5.0                                         // Largura da banda de deteccao de passagem por zero
#define Disj_PCC                GpioDataRegs.GPADAT.bit.GPIO12              // Comando do disjuntor de conexao
//

//                               ESTADOS DO CONTROLADOR BOOST(MPPT)
typedef enum{
    MPPT_OFF,               // OPERACAO DE RASTREAMENTO DO PONTO DE MAXIMA POTENCIA DESLIGADO (TODO: CHAVE FECHADA OU ABERTA?)
    MPPT_ON                // OPERACAO DE RASTREAMENTO DO PONTO DE MAXIMA POTENCIA LIGADO
//    GMPP_SCAN,              // (global maximum power point) INICIA RAMPA DE VARREDURA PARA ENCONTRAR O PONTO MPPT GLOBAL
//    LPPT                    // LIMITA A GERACAO FOTOVOLTAICA COM UM VALOR DE REFERENCIA
}MPPTstates_t;

typedef enum{
    FORA_DE_OPERACAO,       // Nao sincroniza nem despacha potencia
    SINCRONIZANDO,          // Sincroniza, mas nao despacha potencia
    PRE_CARGA,              // Sincroniza e despacha para o carregamento do filtro LCL, nao conecta na rede
    CONECTADO,              // Sincronizado, conectado e operando
}ESTADOS_t;


#define DESLIGADO 0         // INVERSOR DESLIGADO / LIGADO
#define LIGADO 1
#define ABERTO 0            // DISJUNTOR DE CONEXAO ABERTO / FECHADO
#define FECHADO 1
#define DISABLE 0
#define ENABLE 1



int MPPT(void);
void controle_paineisPV(int v_pv_ref, int enable);
void update_MAsumVec(void);
void moving_average(void);
void PLL(void);
void temporizacao(float tempo);
void temporizacao1(float tempo);
void limpa_temporizador(void);
void limpa_temporizador1(void);
int detecta_zero(float sig_det_0);
void maq_estados_inv(void);
void modulacao_3_niveis(void);
void controle_inv();
void gera_referencia();
void estimador_SoC();
void SVPWM(float *da, float *db, float *dc);
#endif /* APPLICATION_CONTROLES_H_ */
