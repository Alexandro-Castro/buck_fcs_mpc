//###########################################################################
//
// main.c - Conversor buck controlado pelo F2837xD (CPU1)
//
// Amostragem de ADCINA0 (vo) e ADCINA2 (iL) sincronizada com o ePWM1 em
// modo up-down, com EPWM1A/EPWM1B complementares. O controle roda na ISR
// do ADC, uma vez por periodo do PWM.
//
// Operacao pelo Watch (Expressions) do CCS:
//   modo    = 0 PI, 1 FCS de corrente, 2 FCS de tensao (vale ao ligar)
//   comando = 1  liga o controle; comando = 0 desliga (chaves abertas)
//   rearme  = 1  sai do estado de falha (volta a parado)
//   estado, falha, iL_falha, vo_falha mostram o que aconteceu
//
// Chaves desligadas = trip one-shot do ePWM1 (EPWM1A e EPWM1B em 0, depois
// do tempo morto). O trip tambem dispara sozinho se a CPU for parada pelo
// depurador (TZ6), se o relogio falhar (TZ5) ou se o comparador CMPSS1
// detectar iL > IL_MAX_HW (sem passar pela CPU).
//
// Baseado no exemplo adc_soc_epwm (F2837xD Support Library v210 - TI)
//
//###########################################################################

#include "F28x_Project.h"
#include "config.h"
#include "controle.h"

//
// Controlador e periodo do PWM. Ts = periodo de amostragem = periodo do
// PWM up-down (uma ISR por periodo); o modelo do FCS-MPC precisa do valor
// exato. Os tres sao atualizados por aplica_modo() a cada partida.
//
volatile modo_t modo       = MODO_INICIAL;  // escolhido no Watch
volatile modo_t modo_ativo = MODO_INICIAL;  // em uso
static Uint16 tbprd = TBPRD_FCS;
float Ts            = 2.0f * TBPRD_FCS / TBCLK_HZ;   // FCS: 25 us; PI: 50 us

#define RESULTS_BUFFER_SIZE 256

//
// Estados e falhas
//
typedef enum {
    ESTADO_PARADO  = 0,     // chaves desligadas, aguardando comando = 1
    ESTADO_RODANDO = 1,     // controle ligado
    ESTADO_FALHA   = 2      // chaves desligadas, aguardando rearme = 1
} estado_t;

typedef enum {
    FALHA_NENHUMA       = 0,
    FALHA_SOBRECORRENTE = 1,    // iL > IL_MAX (software)
    FALHA_SOBRETENSAO   = 2,    // vo > VO_MAX (software)
    FALHA_DEPURADOR     = 3,    // CPU parada pelo depurador (TZ6)
    FALHA_RELOGIO       = 4,    // falha do relogio (TZ5)
    FALHA_TRIP          = 5,    // outro trip do ePWM1
    FALHA_SOBRECORRENTE_HW = 6  // iL > IL_MAX_HW (comparador CMPSS1)
} falha_t;

//
// Comandos (escritos no Watch)
//
volatile Uint16 comando = 0;
volatile Uint16 rearme  = 0;

//
// Referencias (escritas no Watch) e rampas
//
volatile float iL_ref   = IL_REF_INICIAL;   // A
volatile float vo_ref   = VO_REF_INICIAL;   // V
volatile float rampa_iL = RAMPA_IL_INICIAL; // A/s
volatile float rampa_vo = RAMPA_VO_INICIAL; // V/s

//
// Variaveis para o Watch
//
volatile estado_t estado = ESTADO_PARADO;
volatile falha_t  falha  = FALHA_NENHUMA;
volatile float iL_falha  = 0;   // medidas no instante da falha
volatile float vo_falha  = 0;

volatile float iL   = 0;        // A
volatile float vo   = 0;        // V
volatile float d    = 0;        // duty do PI
volatile int   flag = 0;        // estado escolhido pelo FCS-MPC

volatile float iL_ref_atual = 0;    // referencias seguidas pelo controle
volatile float vo_ref_atual = 0;

//
// Buffers das ultimas amostras do ADC (contagens)
//
Uint16 AdcaResults_0[RESULTS_BUFFER_SIZE];   // ADCINA0 - tensao vo
Uint16 AdcaResults_1[RESULTS_BUFFER_SIZE];   // ADCINA2 - corrente iL
volatile Uint16 resultsIndex;
volatile Uint16 bufferFull;

static Uint16 ledCount;

//
// Prototipos
//
void ConfigureADC(void);
void SetupADCEpwm(void);
void ConfigureCMPSS(void);
void ConfigureEPWM(void);
void configurePWM_GPIOs(void);
void StartEPWM(void);
static inline void PWM1_SetA(Uint16 on);
static void aplica_modo(modo_t m);
static void liga(void);
static void desliga(void);
static void entra_falha(falha_t f);
static falha_t verifica_falhas(void);
static float rampa(float atual, float alvo, float passo);
static Uint16 cmpa_do_duty(float duty);
static float limita(float x, float min, float max);
static void pisca_led(void);
interrupt void adca1_isr(void);

void main(void)
{
    Uint16 i;

    //
    // Sistema: PLL, watchdog, clocks dos perifericos
    //
    InitSysCtrl();

    //
    // GPIOs. GPIO0/GPIO1 (EPWM1A/B) ficam como saidas em 0 ate o ePWM
    // estar rodando com o trip ativo
    //
    InitGpio();

    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(0, 0);

    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(1, 0);

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

    //
    // ePWM (com os clocks de base de tempo travados), ja com o trip ativo
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    ConfigureCMPSS();
    ConfigureEPWM();
    aplica_modo(modo);
    controle_inicializacao(Ts);

    //
    // Buffers e LED
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
    // Habilita interrupcoes
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // ADCA1
    IER |= M_INT1;
    EINT;
    ERTM;

    //
    // Liga o contador do PWM (e, com ele, a amostragem do ADC). As chaves
    // continuam desligadas pelo trip; so os pinos passam para o ePWM.
    //
    StartEPWM();
    DELAY_US(10);
    configurePWM_GPIOs();

    GPIO_WritePin(GPIO_P2, 1);

    //
    // Loop principal: o PWM, o ADC e o controle rodam por interrupcao
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
// ConfigureCMPSS - Comparador alto do CMPSS1: CMPIN1P (ADCINA2, corrente)
//                  contra o DAC interno em IL_MAX_HW. A saida filtrada vai
//                  para o TRIP4 do ePWM X-BAR (usado em ConfigureEPWM).
//
void ConfigureCMPSS(void)
{
    Uint16 dac;

    //
    // IL_MAX_HW em contagens do ADC, convertido para a escala do DAC
    //
    dac = (Uint16)((IL_MAX_HW / KADC_I) * (ADC_VREFHI / CMPSS_VREF));
    if(dac > 4095U)
    {
        dac = 4095U;
    }

    EALLOW;

    Cmpss1Regs.COMPCTL.bit.COMPDACE    = 1;     // liga comparador e DAC
    Cmpss1Regs.COMPCTL.bit.COMPHSOURCE = 0;     // entrada - = DAC interno
    Cmpss1Regs.COMPCTL.bit.COMPHINV    = 0;     // saida em 1 com iL > limite
    Cmpss1Regs.COMPHYSCTL.bit.COMPHYS  = 2;     // histerese (2x a tipica)

    Cmpss1Regs.COMPDACCTL.bit.SELREF    = 0;    // referencia do DAC = VDDA
    Cmpss1Regs.COMPDACCTL.bit.DACSOURCE = 0;    // valor de DACHVALS
    Cmpss1Regs.COMPDACCTL.bit.SWLOADSEL = 0;    // carrega a cada SYSCLK
    Cmpss1Regs.DACHVALS.bit.DACVAL      = dac;

    //
    // Filtro digital contra ruido de comutacao: 10 amostras a SYSCLK/10
    // (janela de 1 us com SYSCLK = 100 MHz), trip com 7 ou mais em 1
    //
    Cmpss1Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 9;
    Cmpss1Regs.CTRIPHFILCTL.bit.SAMPWIN        = 9;
    Cmpss1Regs.CTRIPHFILCTL.bit.THRESH         = 7;
    Cmpss1Regs.CTRIPHFILCTL.bit.FILINIT        = 1;

    Cmpss1Regs.COMPCTL.bit.CTRIPHSEL    = 2;    // CTRIPH = saida do filtro
    Cmpss1Regs.COMPCTL.bit.CTRIPOUTHSEL = 2;

    //
    // ePWM X-BAR: TRIP4 = CMPSS1.CTRIPH (mux 0, opcao 0)
    //
    EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0 = 0;
    EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0   = 1;

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
//                 SOCA no pico do contador (CTR = TBPRD), trip ativo
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
    EPwm1Regs.TBPRD               = tbprd;  // shadow: carrega em CTR = 0

    //
    // Comparador A (shadow, carrega em CTR = 0)
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm1Regs.CMPA.bit.CMPA        = 0;

    //
    // Action qualifier: as acoes dependem do modo (aplica_modo). A forca
    // por software (AQCSFRC) carrega em CTR = 0 (RLDCSF = 3 seria imediato).
    //
    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;
    EPwm1Regs.AQCSFRC.bit.CSFA  = 1;        // EPWM1A comeca em nivel baixo

    //
    // Tempo morto: B complementar de A
    //
    EPwm1Regs.DBCTL.bit.IN_MODE  = 0;
    EPwm1Regs.DBCTL.bit.POLSEL   = 2;       // AHC
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBRED.bit.DBRED    = PWM_TEMPO_MORTO;
    EPwm1Regs.DBFED.bit.DBFED    = PWM_TEMPO_MORTO;

    //
    // Digital compare: DCAH = TRIPIN4 (CMPSS1, sobrecorrente); DCAEVT1
    // quando DCAH = 1, assincrono e sem filtro do DC
    //
    EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL  = 3;   // TRIPIN4
    EPwm1Regs.TZDCSEL.bit.DCAEVT1        = 2;   // DCAH em 1
    EPwm1Regs.DCACTL.bit.EVT1SRCSEL      = 0;
    EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL  = 1;

    //
    // Trip zone: A e B em 0 (atua depois do tempo morto, entao desliga as
    // duas chaves). Fontes one-shot: TZ5 = falha do relogio, TZ6 = CPU
    // parada pelo depurador, DCAEVT1 = sobrecorrente no CMPSS1. Comeca com
    // o trip forcado (chaves desligadas).
    //
    EPwm1Regs.TZSEL.bit.DCAEVT1 = 1;
    EPwm1Regs.TZCTL.bit.TZA     = 2;        // trip: EPWM1A em 0
    EPwm1Regs.TZCTL.bit.TZB     = 2;        // trip: EPWM1B em 0
    EPwm1Regs.TZCTL.bit.DCAEVT1 = 3;        // eventos do digital compare
    EPwm1Regs.TZCTL.bit.DCAEVT2 = 3;        // sem acao direta (o padrao,
    EPwm1Regs.TZCTL.bit.DCBEVT1 = 3;        // 0, e alta impedancia)
    EPwm1Regs.TZCTL.bit.DCBEVT2 = 3;
    EPwm1Regs.TZSEL.bit.OSHT5   = 1;
    EPwm1Regs.TZSEL.bit.OSHT6   = 1;
    EPwm1Regs.TZFRC.bit.OST     = 1;

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
// aplica_modo - Periodo do PWM, Ts e action qualifier do controlador m.
//               Chamada so com as chaves desligadas (trip ativo).
//
static void aplica_modo(modo_t m)
{
    if(m > MODO_FCS_TENSAO)
    {
        m    = MODO_FCS_TENSAO;     // valor invalido no Watch
        modo = m;
    }
    modo_ativo = m;

    tbprd = (m == MODO_PI) ? TBPRD_PI : TBPRD_FCS;
    Ts    = 2.0f * tbprd / TBCLK_HZ;

    EPwm1Regs.TBPRD         = tbprd;    // shadow: carrega em CTR = 0
    EPwm1Regs.CMPA.bit.CMPA = cmpa_do_duty(0.0f);

    if(m == MODO_PI)
    {
        //
        // PI: up-down simetrico, A alto em torno do zero, duty = CMPA/TBPRD.
        // Sem forca por software: quem manda e o action qualifier.
        //
        EPwm1Regs.AQCTLA.bit.ZRO  = 0;
        EPwm1Regs.AQCTLA.bit.PRD  = 0;
        EPwm1Regs.AQCTLA.bit.CAU  = 1;      // clear em CMPA subindo
        EPwm1Regs.AQCTLA.bit.CAD  = 2;      // set em CMPA descendo
        EPwm1Regs.AQCSFRC.bit.CSFA = 0;
    }
    else
    {
        //
        // FCS: sem acoes; o estado da chave e forcado por software
        //
        EPwm1Regs.AQCTLA.bit.ZRO  = 0;
        EPwm1Regs.AQCTLA.bit.PRD  = 0;
        EPwm1Regs.AQCTLA.bit.CAU  = 0;
        EPwm1Regs.AQCTLA.bit.CAD  = 0;
        EPwm1Regs.AQCSFRC.bit.CSFA = PWM_LOW;
    }
}

//
// liga - Aplica o modo, reinicia o controle e libera as chaves
//        (PARADO -> RODANDO)
//
static void liga(void)
{
    aplica_modo(modo);
    controle_inicializacao(Ts);

    iL_ref_atual = 0;       // corrente parte de zero
    vo_ref_atual = vo;      // tensao parte do valor atual do capacitor
    d    = 0;
    flag = 0;

    EALLOW;
    EPwm1Regs.TZOSTCLR.all      = 0x00FF;   // limpa as origens do trip
    EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
    EPwm1Regs.TZCLR.bit.OST     = 1;        // libera as saidas
    EPwm1Regs.TZCLR.bit.INT     = 1;
    EDIS;

    estado = ESTADO_RODANDO;
}

//
// desliga - Abre as duas chaves pelo trip one-shot (-> PARADO)
//
static void desliga(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EDIS;

    //
    // Deixa o PWM em 0 para a proxima partida nao comecar com A em 1
    //
    if(modo_ativo != MODO_PI)
    {
        PWM1_SetA(0);
    }
    EPwm1Regs.CMPA.bit.CMPA = cmpa_do_duty(0.0f);
    d    = 0;
    flag = 0;

    estado = ESTADO_PARADO;
}

//
// entra_falha - Desliga, guarda as medidas e trava em FALHA
//
static void entra_falha(falha_t f)
{
    desliga();

    falha    = f;
    iL_falha = iL;
    vo_falha = vo;
    comando  = 0;
    estado   = ESTADO_FALHA;
}

//
// verifica_falhas - Trip do ePWM1 e limites de software (so com o controle
//                   rodando)
//
static falha_t verifica_falhas(void)
{
    if(EPwm1Regs.TZFLG.bit.OST)
    {
        if(EPwm1Regs.TZOSTFLG.bit.DCAEVT1)
        {
            return FALHA_SOBRECORRENTE_HW;
        }
        if(EPwm1Regs.TZOSTFLG.bit.OST6)
        {
            return FALHA_DEPURADOR;
        }
        if(EPwm1Regs.TZOSTFLG.bit.OST5)
        {
            return FALHA_RELOGIO;
        }
        return FALHA_TRIP;
    }

    if(iL > IL_MAX)
    {
        return FALHA_SOBRECORRENTE;
    }

    if(vo > VO_MAX)
    {
        return FALHA_SOBRETENSAO;
    }

    return FALHA_NENHUMA;
}

//
// rampa - Aproxima atual de alvo em no maximo passo por amostra
//
static float rampa(float atual, float alvo, float passo)
{
    if(alvo > atual + passo)
    {
        return atual + passo;
    }
    if(alvo < atual - passo)
    {
        return atual - passo;
    }
    return alvo;
}

//
// cmpa_do_duty - CMPA = d * TBPRD, entre 1 e TBPRD - 1. CMPA = 0 coincide
//                com o evento de zero do contador; com 1, o pulso de 2 TBCLK
//                que sobra fica dentro do tempo morto e nao chega a chave.
//
static Uint16 cmpa_do_duty(float duty)
{
    float c = duty * (float)tbprd;

    if(c < 1.0f)
    {
        return 1U;
    }
    if(c > (float)(tbprd - 1U))
    {
        return tbprd - 1U;
    }
    return (Uint16)c;
}

static float limita(float x, float min, float max)
{
    if(x > max)
    {
        return max;
    }
    if(x < min)
    {
        return min;
    }
    return x;
}

//
// pisca_led - Velocidade do LED indica o estado
//
static void pisca_led(void)
{
    float periodo;

    switch(estado)
    {
        case ESTADO_RODANDO: periodo = LED_RODANDO_S; break;
        case ESTADO_FALHA:   periodo = LED_FALHA_S;   break;
        default:             periodo = LED_PARADO_S;  break;
    }

    ledCount++;
    if((float)ledCount * Ts >= periodo)
    {
        ledCount = 0;
        GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    }
}

//
// adca1_isr - Le vo e iL e roda o controle, uma vez por periodo do PWM
//
interrupt void adca1_isr(void)
{
    falha_t f;

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

    pisca_led();

    //
    // Maquina de estados e controle
    //
    switch(estado)
    {
        case ESTADO_PARADO:
            if(comando)
            {
                liga();
            }
            break;

        case ESTADO_RODANDO:
            if(!comando)
            {
                desliga();
                break;
            }

            f = verifica_falhas();
            if(f != FALHA_NENHUMA)
            {
                entra_falha(f);
                break;
            }

            //
            // Referencias em rampa, limitadas
            //
            iL_ref_atual = rampa(iL_ref_atual,
                                 limita(iL_ref, 0.0f, IL_REF_MAX),
                                 rampa_iL * Ts);
            vo_ref_atual = rampa(vo_ref_atual,
                                 limita(vo_ref, 0.0f, VO_REF_MAX),
                                 rampa_vo * Ts);

            switch(modo_ativo)
            {
                case MODO_PI:
                    d = controle(vo, iL, vo_ref_atual);   // ja limitado
                    EPwm1Regs.CMPA.bit.CMPA = cmpa_do_duty(d);
                    break;

                case MODO_FCS_CORRENTE:
                    flag = controlador_fcs_mpc_corrente(vo, iL, iL_ref_atual);
                    PWM1_SetA(flag);
                    break;

                case MODO_FCS_TENSAO:
                default:
                    flag = controlador_fcs_mpc_tensao(vo, iL, vo_ref_atual);
                    PWM1_SetA(flag);
                    break;
            }
            break;

        case ESTADO_FALHA:
        default:
            comando = 0;
            if(rearme)
            {
                rearme = 0;
                falha  = FALHA_NENHUMA;
                estado = ESTADO_PARADO;
            }
            break;
    }

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
