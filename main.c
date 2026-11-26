//###########################################################################
//
// FILE:   adc_soc_epwm_cpu01.c
//
// TITLE:  Amostragem de ADCINA0 (Vo) e ADCINA2 (iL) sincronizada com o
//         ePWM1 em modo up-down (20 kHz), com EPWM1A/EPWM1B complementares.
//
// Baseado no exemplo adc_soc_epwm (F2837xD Support Library v210 - TI)
//
//###########################################################################

//
// Included Files
//
#include "F28x_Project.h"
#include "controle.h"

//
// Defines
//
#define RESULTS_BUFFER_SIZE 256

#define BLINKY_LED_GPIO     31
#define SW1                 18
#define SW2                 19
#define P2                  2

//
// ePWM1: up-down, fpwm = TBCLK / (2 * TBPRD)
//
// TBCLK = EPWMCLK = SYSCLK / 2. Sem o define _LAUNCHXL_F28379D, InitSysCtrl()
// usa IMULT_20 e PLLCLK_BY_2, ou seja, SYSCLK = 10 x cristal:
//   LaunchPad F28379D (cristal de 10 MHz):  SYSCLK = 100 MHz -> TBCLK =  50 MHz
//   controlCARD F2837xD (cristal de 20 MHz): SYSCLK = 200 MHz -> TBCLK = 100 MHz
// Confira no osciloscopio: o GPIO18 alterna a cada ISR, Ts = 1 / (2 * f_GPIO18).
//
#define TBCLK_HZ            50.0e6f             // LaunchPad; 100.0e6f no controlCARD

#ifdef FCS_MPC
#define EPWM1_PERIOD        (1250U / 2)         // TBPRD 625: 40 kHz com TBCLK = 50 MHz
#else
#define EPWM1_PERIOD        (1250U)             // TBPRD 1250: 20 kHz com TBCLK = 50 MHz
#endif

#define EPWM1_DUTY          (EPWM1_PERIOD / 2)  // 50% (duty de A = CMPA / TBPRD)
#define EPWM1_DEADBAND      50U                 // 50 TBCLK = 1 us com TBCLK = 50 MHz

//
// Periodo de amostragem = periodo do PWM up-down (uma ISR por periodo).
// O modelo de predicao do FCS-MPC precisa deste valor exato.
//
float Ts = 2.0f * EPWM1_PERIOD / TBCLK_HZ;      // FCS: 25 us; PI: 50 us
//
// LED pisca a cada LED_TOGGLE_COUNT interrupcoes do ADC
// (20 kHz / 1000 = alterna a cada 50 ms)
//
#define LED_TOGGLE_COUNT    1000U


float kADC_i =    1.3/ 1600.0;
float kADC_v =    31.0 / 2080;
float iL     = 0;
float vo     = 0;

float d = 0;
//
// Function Prototypes
//
void ConfigureADC(void);
void ConfigureEPWM(void);
void configurePWM_GPIOs(void);
void SetupADCEpwm(void);
void StartEPWM(void);
static inline void PWM1_SetA(Uint16 on);
interrupt void adca1_isr(void);


//
// Globals
//
Uint16 AdcaResults_0[RESULTS_BUFFER_SIZE];   // ADCINA0 - tensao Vo
Uint16 AdcaResults_1[RESULTS_BUFFER_SIZE];   // ADCINA2 - corrente iL
volatile Uint16 resultsIndex;
volatile Uint16 bufferFull;
volatile Uint16 ledCount;




void main(void)
{
    Uint16 i;

    //
    // Step 1. Sistema: PLL, watchdog, clocks dos perifericos
    //
    InitSysCtrl();

    //
    // Step 2. GPIOs
    //
    InitGpio();

    GPIO_SetupPinMux(BLINKY_LED_GPIO, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(BLINKY_LED_GPIO, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(SW1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SW1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(SW2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(SW2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(P2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(P2, GPIO_OUTPUT, GPIO_PUSHPULL);

    //
    // Step 3. Interrupcoes e tabela de vetores da PIE
    //
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr;
    EDIS;

    //
    // Step 4. ADC
    //
    ConfigureADC();
    SetupADCEpwm();

    controle_inicializacao(Ts);

    //
    // Step 5. ePWM (com os clocks de base de tempo travados)
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    ConfigureEPWM();
    configurePWM_GPIOs();   // pinos so vao para o ePWM depois de configurado

    //
    // Step 6. Buffers
    //
    for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
    {
        AdcaResults_0[i] = 0;
        AdcaResults_1[i] = 0;
    }
    resultsIndex = 0;
    bufferFull   = 0;
    ledCount     = 0;

    //
    // Step 7. Habilita interrupcoes
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // ADCA1
    IER |= M_INT1;
    EINT;
    ERTM;

    //
    // Step 8. Liga o PWM (e, com ele, a amostragem do ADC)
    //

#ifdef FCS_MPC
    PWM1_SetA(0);
    DELAY_US(1000);
    PWM1_SetA(1);
#endif

    StartEPWM();

    GPIO_WritePin(P2, 1);

    

    //
    // Loop principal: o PWM e o ADC rodam continuamente por hardware
    //
    for(;;)
    {
        if(bufferFull)
        {
            bufferFull = 0;

            //
            // Buffers AdcaResults_0[] e AdcaResults_1[] completaram uma volta.
            // Processamento em segundo plano, se necessario.
            //
        }
    }
}

//
// ConfigureADC - Configura e liga o ADC-A
//
void ConfigureADC(void)
{
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;   // ADCCLK = SYSCLK / 4 = 50 MHz
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;   // interrupcao no fim da conversao
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ    = 1;   // liga o ADC

    EDIS;

    DELAY_US(1000);   // 1 ms para o ADC estabilizar
}

//
// SetupADCEpwm - SOC0 = ADCINA0, SOC1 = ADCINA2, ambos disparados pelo SOCA
//                do ePWM1. ADCINT1 no fim do SOC1.
//
void SetupADCEpwm(void)
{
    Uint16 acqps;

    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14;   // 75 ns
    }
    else
    {
        acqps = 63;   // 320 ns
    }

    EALLOW;

    //
    // SOC0 -> ADCINA0 (tensao Vo)
    //
    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 0;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = acqps;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // ePWM1 SOCA

    //
    // SOC1 -> ADCINA2 (corrente iL)
    //
    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 2;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;   // ePWM1 SOCA

    //
    // ADCINT1 no fim do SOC1 (ultima conversao)
    //
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E   = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    EDIS;
}

//
// configurePWM_GPIOs - GPIO0 = EPWM1A, GPIO1 = EPWM1B
//
void configurePWM_GPIOs(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO0   = 1;   // sem pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO1   = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;   // EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 1;   // EPWM1B

    EDIS;
}

//
// ConfigureEPWM - ePWM1 up-down, 20 kHz, A/B complementares com dead-band,
//                 SOCA no pico do contador (CTR = TBPRD)
//
void ConfigureEPWM(void)
{
    EALLOW;

    //
    // Base de tempo
    //
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;     // congelado durante a configuracao
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;     // /1
    EPwm1Regs.TBCTL.bit.CLKDIV    = 0;     // /1 -> TBCLK = 100 MHz
    EPwm1Regs.TBCTL.bit.PHSEN     = 0;
    EPwm1Regs.TBPHS.bit.TBPHS     = 0;
    EPwm1Regs.TBCTR               = 0;
    EPwm1Regs.TBPRD               = EPWM1_PERIOD;

    //
    // Comparador A (shadow, carrega em CTR = 0)
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm1Regs.CMPA.bit.CMPA        = EPWM1_DUTY;

    //
    // Action qualifier (up-down simetrico)
    // A alto em torno do zero, baixo em torno do PRD. Duty de A = CMPA / TBPRD
    
#ifdef FCS_MPC
    EPwm1Regs.AQCTLA.bit.ZRO = 0;
    EPwm1Regs.AQCTLA.bit.PRD = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 0;
    EPwm1Regs.AQCTLA.bit.CAD = 0;

    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;       // 3 sem shadow: efeito imediato; 0 com shadow, efeito quando prd chega a zero
    EPwm1Regs.AQCSFRC.bit.CSFA  = 1;       // EPWM1A começa em nível baixo
#else
    EPwm1Regs.AQCTLA.bit.ZRO = 0;
    EPwm1Regs.AQCTLA.bit.PRD = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 1;          // clear em CMPA subindo
    EPwm1Regs.AQCTLA.bit.CAD = 2;          // set em CMPA descendo
#endif
    //
    // Dead-band: B complementar de A
    //
    EPwm1Regs.DBCTL.bit.IN_MODE  = 0;
    EPwm1Regs.DBCTL.bit.POLSEL   = 2;      // AHC
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBRED.bit.DBRED    = EPWM1_DEADBAND;
    EPwm1Regs.DBFED.bit.DBFED    = EPWM1_DEADBAND;

    //
    // Trigger do ADC no pico do contador (valor medio da corrente)
    //
    EPwm1Regs.ETSEL.bit.SOCAEN  = 0;       // habilitado em StartEPWM()
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;       // CTR = TBPRD
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;       // a cada periodo

    EDIS;
}


#define PWM_LOW   1U   // valores do campo CSFA
#define PWM_HIGH  2U

static inline void PWM1_SetA(Uint16 on)
{
    EPwm1Regs.AQCSFRC.bit.CSFA = on ? PWM_HIGH : PWM_LOW;
    // EPWM1B = complemento de A, com dead-band
}

//
// StartEPWM - Libera o clock de base de tempo e inicia o contador up-down
//
void StartEPWM(void)
{
    EALLOW;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   // comeca com a flag limpa
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC  = 1;   // libera os contadores
    EPwm1Regs.ETSEL.bit.SOCAEN        = 1;   // habilita SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE       = 2;   // up-down: PWM rodando
    EDIS;
}

//
// adca1_isr - Le Vo e iL a cada periodo do PWM (20 kHz)
//
int k = 0;
int min = 500;
int max = 900;
int var = 500;
int flag = 1;

interrupt void adca1_isr(void)
{
    //
    // Debug: GPIO18 alterna a cada interrupcao (onda de 10 kHz no osciloscopio)
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO18 = 1;

    //
    // Le os resultados (grava primeiro, incrementa depois)
    //
    AdcaResults_0[resultsIndex] = AdcaResultRegs.ADCRESULT0;   // Vo
    AdcaResults_1[resultsIndex] = AdcaResultRegs.ADCRESULT1;   // iL

    vo = AdcaResultRegs.ADCRESULT0 * kADC_v;
    iL = AdcaResultRegs.ADCRESULT1 * kADC_i;

    resultsIndex++;
    if(resultsIndex >= RESULTS_BUFFER_SIZE)
    {
        resultsIndex = 0;
        bufferFull   = 1;
    }

    //
    // LED
    //
    ledCount++;
    if(ledCount >= LED_TOGGLE_COUNT)
    {
        ledCount = 0;
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    }

#ifdef FCS_MPC
    //PWM1_SetA(flag);
    //flag = !flag;

    //PWM1_SetA(flag);
    // flag = controlador_fcs_mpc_corrente(vo, iL);

    flag = controlador_fcs_mpc_tensao  (vo, iL);
    PWM1_SetA(flag);
#else
    d = controle(vo, iL);
    //d = 0.5;
    EPwm1Regs.CMPA.bit.CMPA = d * EPWM1_PERIOD;
#endif





/*
    EPwm1Regs.CMPA.bit.CMPA = var;

    var = var + 50*flag - 50*(1-flag);
    if(var > max) flag = 0;
    if(var < min) flag = 1;
*/

    //
    // Limpa a flag e trata overflow
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of file
//
