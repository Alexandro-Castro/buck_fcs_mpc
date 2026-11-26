//###########################################################################
//
// controle.c - Controladores do buck: PI em cascata e FCS-MPC
//
// Modelo do buck (s = 1 chave de cima ligada, s = 0 de baixo), num
// intervalo dt com o estado s constante:
//   iL(t+dt) = iL + (dt/L) * (E*s - vo)                    (corrente linear)
//   vo(t+dt) = vo + (dt/C) * (iL + di/2 - vo/R)            (corrente media)
//
// Timing do PWM no FCS (contador up-down, periodo Ts):
//
//   zero         topo          zero          topo          zero
//    |-- s_atual --|-- s_atual --|--- s novo ---|--- s novo ---|
//                  ^ amostra e   ^ s novo entra               ^ instante
//                    decisao       (forca carrega em CTR = 0)   avaliado
//
// A amostra cai no meio do intervalo, e o estado decidido so entra no zero
// seguinte, Ts/2 depois. Por isso os FCS primeiro levam o estado medido ate
// esse zero com o estado que ja esta aplicado (compensacao do atraso) e
// depois avaliam cada candidato no fim do intervalo seguinte, no instante
// de comutacao. Os picos e vales do ripple ficam centrados na referencia,
// e a media da corrente fica na referencia em regime.
//
//###########################################################################

#include "config.h"
#include "controle.h"

//
// Limite de corrente do custo do FCS de tensao (alteravel no Watch)
//
volatile float iL_limit = FCS_V_IL_LIMITE;

//
// Referencia de corrente gerada pela malha de tensao do PI (para o Watch)
//
volatile float iL_ref_pi = 0;

//
// Parametros do modelo (alteraveis no Watch; valem na proxima partida)
//
float E = E_ENTRADA;
float L = L_INDUTOR;
float C = C_SAIDA;
float R = R_CARGA;

//
// Constantes do modelo, calculadas em controle_inicializacao() para a ISR
// so multiplicar (divisao em float e lenta no C28x)
//
static float Ts_div_L = 0;      // Ts/L
static float Ts_div_C = 0;      // Ts/C
static float Th_div_L = 0;      // (Ts/2)/L
static float Th_div_C = 0;      // (Ts/2)/C
static float G        = 0;      // 1/R
static float invE     = 0;      // 1/E (feedforward do PI)

static pi_t pi_corrente;
static pi_t pi_tensao;

int sOpt = 0;   // estado aplicado no intervalo atual (ultima decisao)

//
// controlador_pi - PI discreto: u = kp*e + ki*soma(e), com ki = KI*Ts,
//                  saturado em [umin, umax]. Anti-windup por integracao
//                  condicional: saturado, so integra o erro que tira da
//                  saturacao.
//
float controlador_pi(pi_t *pi, float erro, float umin, float umax)
{
    float u;

    u = pi->kp * erro + pi->ki * (pi->integral + erro);

    if(u > umax)
    {
        u = umax;
        if(erro < 0.0f)
        {
            pi->integral += erro;
        }
    }
    else if(u < umin)
    {
        u = umin;
        if(erro > 0.0f)
        {
            pi->integral += erro;
        }
    }
    else
    {
        pi->integral += erro;
    }

    return u;
}

//
// controle_inicializacao - Ganhos dos PI e constantes do modelo para um Ts.
//                          Chamada a cada partida, com a chave de cima
//                          desligada (sOpt = 0).
//
void controle_inicializacao(float Ts)
{
    pi_corrente.kp       = PI_I_KP;
    pi_corrente.ki       = PI_I_KI * Ts;
    pi_corrente.integral = 0;

    pi_tensao.kp       = PI_V_KP;
    pi_tensao.ki       = PI_V_KI * Ts;
    pi_tensao.integral = 0;

    iL_ref_pi = 0;
    sOpt      = 0;

    Ts_div_L = Ts / L;
    Ts_div_C = Ts / C;
    Th_div_L = 0.5f * Ts_div_L;
    Th_div_C = 0.5f * Ts_div_C;
    G        = 1.0f / R;
    invE     = 1.0f / E;
}

//
// controle - PI em cascata: tensao (externa) -> iL_ref -> corrente -> duty.
//            iL_ref limitada a [0, PI_IL_REF_MAX] (limite de corrente) e
//            duty a [PI_D_MIN, PI_D_MAX], com feedforward d0 = vo/E.
//
float controle(float v, float i, float vo_ref)
{
    float d0;

    iL_ref_pi = controlador_pi(&pi_tensao, vo_ref - v, 0.0f, PI_IL_REF_MAX);

#if PI_FEEDFORWARD
    d0 = v * invE;
    if(d0 > PI_D_MAX)
    {
        d0 = PI_D_MAX;
    }
    if(d0 < PI_D_MIN)
    {
        d0 = PI_D_MIN;
    }
#else
    d0 = 0.0f;
#endif

    return d0 + controlador_pi(&pi_corrente, iL_ref_pi - i,
                               PI_D_MIN - d0, PI_D_MAX - d0);
}

//
// controlador_fcs_mpc_corrente - Escolhe o estado que deixa a corrente no
//                                fim do proximo intervalo mais perto de
//                                iL_ref
//
int controlador_fcs_mpc_corrente(float vo, float iL, float iL_ref)
{
    int j;
    float g;
    float iz;
    float ie;
    float gOpt = 1e30f;
    int sNovo  = 0;

    const int s[2] = {1, 0};

    //
    // Compensacao do atraso: corrente no proximo zero, com o estado atual
    //
    iz = iL + (E * (float)sOpt - vo) * Th_div_L;

    for(j = 0; j < 2; j++)
    {
        ie = iz + (E * (float)s[j] - vo) * Ts_div_L;

        g = (iL_ref - ie) * (iL_ref - ie);

        if(g < gOpt)
        {
            sNovo = s[j];
            gOpt  = g;
        }
    }

    sOpt = sNovo;
    return sOpt;
}

//
// controlador_fcs_mpc_tensao - Mesma predicao, com custo na tensao e na
//                              corrente (referencia vo_ref/R) e penalidade
//                              de sobrecorrente
//
int controlador_fcs_mpc_tensao(float vo, float iL, float vo_ref)
{
    int j;
    float g;
    float di, iz, vz;
    float ie, ve;
    float iL_ref_v;
    float peso;
    float gOpt = 1e30f;
    int sNovo  = 0;

    const int s[2] = {1, 0};

    //
    // Compensacao do atraso: estado no proximo zero, com o estado atual
    //
    di = (E * (float)sOpt - vo) * Th_div_L;
    iz = iL + di;
    vz = vo + (iL + 0.5f * di - vo * G) * Th_div_C;

    iL_ref_v = vo_ref * G;

    for(j = 0; j < 2; j++)
    {
        di = (E * (float)s[j] - vz) * Ts_div_L;
        ie = iz + di;
        ve = vz + (iz + 0.5f * di - vz * G) * Ts_div_C;

        peso = (ie >= iL_limit) ? FCS_V_PENALIDADE : 0.0f;

        g = FCS_V_PESO_I * (iL_ref_v - ie) * (iL_ref_v - ie)
          + FCS_V_PESO_V * (vo_ref - ve) * (vo_ref - ve)
          + peso;

        if(g < gOpt)
        {
            sNovo = s[j];
            gOpt  = g;
        }
    }

    sOpt = sNovo;
    return sOpt;
}
