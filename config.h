//###########################################################################
//
// config.h - Parametros do conversor buck e do controle, num unico lugar
//
//###########################################################################

#ifndef CONFIG_H
#define CONFIG_H

//---------------------------------------------------------------------------
// Controlador ao carregar o programa (troca pela variavel modo no Watch,
// com o conversor parado; vale na proxima partida):
//   MODO_PI, MODO_FCS_CORRENTE ou MODO_FCS_TENSAO
//---------------------------------------------------------------------------
#define MODO_INICIAL        MODO_FCS_TENSAO

//---------------------------------------------------------------------------
// Relogio do ePWM
//
// TBCLK = EPWMCLK = SYSCLK / 2. Sem o define _LAUNCHXL_F28379D, InitSysCtrl()
// usa IMULT_20 e PLLCLK_BY_2, ou seja, SYSCLK = 10 x cristal:
//   LaunchPad F28379D (cristal de 10 MHz):  SYSCLK = 100 MHz -> TBCLK =  50 MHz
//   controlCARD F2837xD (cristal de 20 MHz): SYSCLK = 200 MHz -> TBCLK = 100 MHz
//---------------------------------------------------------------------------
#define TBCLK_HZ            50.0e6f     // LaunchPad; 100.0e6f no controlCARD

//---------------------------------------------------------------------------
// PWM (up-down): fpwm = TBCLK / (2 * TBPRD), uma amostra por periodo
//---------------------------------------------------------------------------
#define TBPRD_PI            1250U       // 20 kHz com TBCLK = 50 MHz
#define TBPRD_FCS           625U        // 40 kHz de amostragem (chave <= 20 kHz)
                                        // (FCS de corrente e de tensao)
#define PWM_TEMPO_MORTO     50U         // em TBCLK: 1 us com TBCLK = 50 MHz

//---------------------------------------------------------------------------
// Circuito (valores nominais usados no modelo de predicao)
//---------------------------------------------------------------------------
#define E_ENTRADA           50.0f       // V
#define L_INDUTOR           1.5e-3f     // H
#define C_SAIDA             220e-6f     // F
#define R_CARGA             15.0f       // ohm

//---------------------------------------------------------------------------
// Ganhos do ADC (contagens -> grandeza fisica)
//---------------------------------------------------------------------------
#define KADC_V              (31.0f / 2080.0f)   // V por contagem (ADCINA0)
#define KADC_I              (1.3f / 1600.0f)    // A por contagem (ADCINA2)

//
// Offset do sensor de corrente: medido ao carregar o programa (e com
// recalibra = 1 no Watch), com as chaves desligadas, e descontado de iL.
// Acima de CAL_OFFSET_MAX o programa vai para falha (corrente circulando
// ou sensor com problema). CALIBRA_IL = 0 desliga a calibracao.
//
#define CALIBRA_IL          1
#define CAL_DESCARTE        64U         // amostras ignoradas no inicio
#define CAL_AMOSTRAS        1024U       // amostras na media
#define CAL_OFFSET_MAX      250U        // contagens (~0,2 A)

//---------------------------------------------------------------------------
// Referencias iniciais (alteraveis no Watch: iL_ref, vo_ref)
//   O controle segue iL_ref_atual / vo_ref_atual, que vao ate iL_ref / vo_ref
//   em rampa (alteravel no Watch: rampa_iL, rampa_vo). Na partida a rampa
//   de corrente comeca em 0 e a de tensao na tensao medida.
//---------------------------------------------------------------------------
#define IL_REF_INICIAL      1.0f        // A
#define VO_REF_INICIAL      10.0f       // V
#define IL_REF_MAX          2.0f        // A: iL_ref acima disso e limitada
#define VO_REF_MAX          25.0f       // V: vo_ref acima disso e limitada
#define RAMPA_IL_INICIAL    100.0f      // A/s (0 -> 1 A em 10 ms)
#define RAMPA_VO_INICIAL    500.0f      // V/s (0 -> 10 V em 20 ms)

//---------------------------------------------------------------------------
// Protecao por software: com o conversor ligado, qualquer medida acima do
// limite desliga as duas chaves e trava em falha (rearme pelo Watch)
//---------------------------------------------------------------------------
#define IL_MAX              2.5f        // A (sensor satura em ~3,3 A)
#define VO_MAX              30.0f       // V

//---------------------------------------------------------------------------
// Protecao de sobrecorrente por hardware: comparador CMPSS1 no mesmo pino
// da corrente (ADCINA2 = CMPIN1P) -> ePWM X-BAR TRIP4 -> trip one-shot do
// ePWM1, sem passar pela CPU. Compara a corrente instantanea (com ripple),
// por isso o limite fica acima de IL_MAX.
//
// O DAC do comparador usa VDDA (3,3 V) e o ADC usa VREFHI (3,0 V na
// LaunchPad F28379D, REF5030). Se VREFHI for 3,3 V na sua placa, o trip
// acontece ~9 % abaixo de IL_MAX_HW (lado seguro).
//---------------------------------------------------------------------------
#define IL_MAX_HW           3.0f        // A
#define ADC_VREFHI          3.0f        // V
#define CMPSS_VREF          3.3f        // V (VDDA)

//---------------------------------------------------------------------------
// PI em cascata (ki = KI * Ts)
//---------------------------------------------------------------------------
#define PI_I_KP             1.0f
#define PI_I_KI             1000.0f
#define PI_V_KP             0.1f
#define PI_V_KI             10.0f

#define PI_IL_REF_MAX       2.0f        // A: saida da malha de tensao
#define PI_D_MIN            0.0f        // limites do duty
#define PI_D_MAX            0.95f
#define PI_FEEDFORWARD      1           // 1: soma d0 = vo/E ao duty

//---------------------------------------------------------------------------
// FCS-MPC de tensao: pesos do custo e penalidade de sobrecorrente
//---------------------------------------------------------------------------
#define FCS_V_PESO_I        1.0f        // (vo_ref/R - iL)^2
#define FCS_V_PESO_V        2.0f        // (vo_ref - vo)^2
#define FCS_V_IL_LIMITE     2.0f        // A (alteravel no Watch: iL_limit)
#define FCS_V_PENALIDADE    1000.0f     // somada quando iL prevista >= limite

//---------------------------------------------------------------------------
// Pinos
//---------------------------------------------------------------------------
#define GPIO_LED            31          // LED de estado
#define GPIO_ISR            18          // depuracao: alterna a cada ISR
#define GPIO_AUX            19          // saida em 0 (sem uso no codigo)
#define GPIO_P2             2           // vai a 1 depois que o PWM liga

//
// LED: tempo entre alternancias em cada estado
//
#define LED_PARADO_S        0.5f        // lento: parado, chaves desligadas
#define LED_RODANDO_S       0.1f        // medio: controle rodando
#define LED_FALHA_S         0.025f      // rapido: falha, aguardando rearme

#endif // CONFIG_H
