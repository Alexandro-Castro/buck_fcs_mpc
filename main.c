//###########################################################################
//
// main.c - Conversor buck controlado pelo F2837xD (CPU1)
//
// Amostragem de ADCINA0 (vo) e ADCINA2 (iL) sincronizada com o ePWM1 em
// modo up-down, com EPWM1A/EPWM1B complementares. O controle roda na ISR
// do ADC, uma vez por periodo do PWM.
//
// Baseado no exemplo adc_soc_epwm (F2837xD Support Library v210 - TI)
//
//###########################################################################

#include "F28x_Project.h"
#include "config.h"
#include "controle.h"

//
// Periodo do PWM e de amostragem
//
#ifdef FCS_MPC
#define EPWM1_PERIOD        TBPRD_FCS
#else
#define EPWM1_PERIOD        TBPRD_PI
#endif

#define EPWM1_DUTY          (EPWM1_PERIOD / 2)  // 50% (duty de A = CMPA / TBPRD)

//
// Periodo de amostragem = periodo do PWM up-down (uma ISR por periodo).
// O modelo de predicao do FCS-MPC precisa deste valor exato.
//
float Ts = 2.0f * EPWM1_PERIOD / TBCLK_HZ;      // FCS: 25 us; PI: 50 us

#define RESULTS_BUFFER_SIZE 256

//
// Variaveis para o Watch
//
volatile float iL   = 0;    // A
volatile float vo   = 0;    // V
volatile float d    = 0;    // duty do PI
volatile int   flag = 0;    // estado escolhido pelo FCS-MPC

//
// Buffers das ultimas amostras do ADC (contagens)
//
Uint16 AdcaResults_0[RESULTS_BUFFER_SIZE];   // ADCINA0 - tensao vo
Uint16 AdcaResults_1[RESULTS_BUFFER_SIZE];   // ADCINA2 - corrente iL
volatile Uint16 resultsIndex;
volatile Uint16 bufferFull;

static Uint16 ledCount;
static Uint16 ledToggleCount;

//
// Prototipos
//
void ConfigureADC(void);
void SetupADCEpwm(void);
void ConfigureEPWM(void);
void configurePWM_GPIOs(void);
void StartEPWM(void);
static inline void PWM1_SetA(Uint16 on);
interrupt void adca1_isr(void);

void main(void)
{
    Uint16 i;

    //
    // Sistema: PLL, watchdog, clocks dos perifericos
    //
    InitSysCtrl();

    //
    // GPIOs
    //
    InitGpio();

    GPIO_SetupPinMux(GPIO_LED, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_LED, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(GPIO_ISR, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_ISR, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(GPIO_AUX, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_AUX, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(GPIO_P2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(GPIO_P2, GPIO_OUTPUT, GPIO_PUSHPULL);

    //
    // Interrupcoes e tabela de vetores da PIE
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
    // ADC e controle
    //
    ConfigureADC();
    SetupADCEpwm();

    controle_inicializacao(Ts);

    //
    // ePWM (com os clocks de base de tempo travados)
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    ConfigureEPWM();
    configurePWM_GPIOs();   // pinos so vao para o ePWM depois de configurado

    //
    // Buffers e LED
    //
    for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
    {
        AdcaResults_0[i] = 0;
        AdcaResults_1[i] = 0;
    }
    resultsIndex   = 0;
    bufferFull     = 0;
    ledCount       = 0;
    ledToggleCount = (Uint16)(LED_PERIODO_S / Ts);

    //
    // Habilita interrupcoes
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // ADCA1
    IER |= M_INT1;
    EINT;
    ERTM;

    //
    // Liga o PWM (e, com ele, a amostragem do ADC)
    //
#ifdef FCS_MPC
    PWM1_SetA(0);
    DELAY_US(1000);
    PWM1_SetA(1);
#endif

    StartEPWM();

    GPIO_WritePin(GPIO_P2, 1);

    //
    // Loop principal: o PWM e o ADC rodam continuamente por hardware
    //
    for(;;)
    {
        if(bufferFull)
        {
            bufferFull = 0;
        }
    }
}

//
// ConfigureADC - Configura e liga o ADC-A
//
void ConfigureADC(void)
{
    EALLOW;

    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      // ADCCLK = SYSCLK / 4
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
        acqps = 14;   // 15 SYSCLK
    }
    else
    {
        acqps = 63;   // 64 SYSCLK
    }

    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL   = 0;    // SOC0 -> ADCINA0 (tensao vo)
    AdcaRegs.ADCSOC0CTL.bit.ACQPS   = acqps;
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    // ePWM1 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL   = 2;    // SOC1 -> ADCINA2 (corrente iL)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS   = acqps;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    // ePWM1 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  // ADCINT1 no fim do SOC1
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

    GpioCtrlRegs.GPAPUD.bit.GPIO0   = 1;    // sem pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO1   = 1;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;    // EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 1;    // EPWM1B

    EDIS;
}

//
// ConfigureEPWM - ePWM1 up-down, A/B complementares com tempo morto,
//                 SOCA no pico do contador (CTR = TBPRD)
//
void ConfigureEPWM(void)
{
    EALLOW;

    //
    // Base de tempo: TBCLK = EPWMCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE   = 3;      // congelado durante a configuracao
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;      // /1
    EPwm1Regs.TBCTL.bit.CLKDIV    = 0;      // /1
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
    // Action qualifier
    //
#ifdef FCS_MPC
    //
    // FCS: sem acoes; o estado da chave e forcado por software (AQCSFRC)
    //
    EPwm1Regs.AQCTLA.bit.ZRO = 0;
    EPwm1Regs.AQCTLA.bit.PRD = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 0;
    EPwm1Regs.AQCTLA.bit.CAD = 0;

    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;        // forca carrega em CTR = 0 (3 = imediato)
    EPwm1Regs.AQCSFRC.bit.CSFA  = 1;        // EPWM1A comeca em nivel baixo
#else
    //
    // PI: up-down simetrico, A alto em torno do zero. Duty de A = CMPA / TBPRD
    //
    EPwm1Regs.AQCTLA.bit.ZRO = 0;
    EPwm1Regs.AQCTLA.bit.PRD = 0;
    EPwm1Regs.AQCTLA.bit.CAU = 1;           // clear em CMPA subindo
    EPwm1Regs.AQCTLA.bit.CAD = 2;           // set em CMPA descendo
#endif

    //
    // Tempo morto: B complementar de A
    //
    EPwm1Regs.DBCTL.bit.IN_MODE  = 0;
    EPwm1Regs.DBCTL.bit.POLSEL   = 2;       // AHC
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBRED.bit.DBRED    = PWM_TEMPO_MORTO;
    EPwm1Regs.DBFED.bit.DBFED    = PWM_TEMPO_MORTO;

    //
    // Trigger do ADC no pico do contador (valor medio da corrente no PI)
    //
    EPwm1Regs.ETSEL.bit.SOCAEN  = 0;        // habilitado em StartEPWM()
    EPwm1Regs.ETSEL.bit.SOCASEL = 2;        // CTR = TBPRD
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;        // a cada periodo

    EDIS;
}

#define PWM_LOW   1U   // valores do campo CSFA
#define PWM_HIGH  2U

//
// PWM1_SetA - Forca EPWM1A em 1 ou 0 (EPWM1B = complemento, com tempo morto)
//
static inline void PWM1_SetA(Uint16 on)
{
    EPwm1Regs.AQCSFRC.bit.CSFA = on ? PWM_HIGH : PWM_LOW;
}

//
// StartEPWM - Libera o clock de base de tempo e inicia o contador up-down
//
void StartEPWM(void)
{
    EALLOW;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // comeca com a flag limpa
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC  = 1;  // libera os contadores
    EPwm1Regs.ETSEL.bit.SOCAEN        = 1;  // habilita SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE       = 2;  // up-down: PWM rodando
    EDIS;
}

//
// adca1_isr - Le vo e iL e roda o controle, uma vez por periodo do PWM
//
interrupt void adca1_isr(void)
{
    //
    // Depuracao: GPIO18 alterna a cada interrupcao (onda de 1/(2*Ts))
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO18 = 1;

    //
    // Le os resultados
    //
    AdcaResults_0[resultsIndex] = AdcaResultRegs.ADCRESULT0;   // vo
    AdcaResults_1[resultsIndex] = AdcaResultRegs.ADCRESULT1;   // iL

    vo = AdcaResultRegs.ADCRESULT0 * KADC_V;
    iL = AdcaResultRegs.ADCRESULT1 * KADC_I;

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
    if(ledCount >= ledToggleCount)
    {
        ledCount = 0;
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    }

    //
    // Controle
    //
#ifdef FCS_MPC
    // flag = controlador_fcs_mpc_corrente(vo, iL);
    flag = controlador_fcs_mpc_tensao(vo, iL);
    PWM1_SetA(flag);
#else
    d = controle(vo, iL);
    EPwm1Regs.CMPA.bit.CMPA = d * EPWM1_PERIOD;
#endif

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
