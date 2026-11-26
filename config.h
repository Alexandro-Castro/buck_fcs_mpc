//###########################################################################
//
// config.h - Parametros do conversor buck e do controle, num unico lugar
//
//###########################################################################

#ifndef CONFIG_H
#define CONFIG_H

//---------------------------------------------------------------------------
// Controlador
//   Com FCS_MPC definido roda o FCS-MPC; comentado, o PI em cascata.
//---------------------------------------------------------------------------
#define FCS_MPC

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

//---------------------------------------------------------------------------
// Referencias iniciais (alteraveis no Watch: iL_ref, vo_ref)
//---------------------------------------------------------------------------
#define IL_REF_INICIAL      1.0f        // A
#define VO_REF_INICIAL      10.0f       // V

//---------------------------------------------------------------------------
// PI em cascata (ki = KI * Ts)
//---------------------------------------------------------------------------
#define PI_I_KP             1.0f
#define PI_I_KI             1000.0f
#define PI_V_KP             0.1f
#define PI_V_KI             10.0f

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
#define GPIO_LED            31          // LED, pisca a cada LED_PERIODO_S
#define GPIO_ISR            18          // depuracao: alterna a cada ISR
#define GPIO_AUX            19          // saida em 0 (sem uso no codigo)
#define GPIO_P2             2           // vai a 1 depois que o PWM liga

#define LED_PERIODO_S       0.05f       // s entre alternancias do LED

#endif // CONFIG_H
