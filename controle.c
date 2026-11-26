//###########################################################################
//
// controle.c - Controladores do buck: PI em cascata e FCS-MPC
//
// Modelo discreto do buck (Euler, s = 1 chave de cima ligada, s = 0 de baixo):
//   iL[k+1] = iL[k] + (Ts/L) * (E*s - vo[k])
//   vo[k+1] = (Ts/C) * iL[k+1] + vo[k] * (1 - Ts/(R*C))
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
// Parametros do modelo
//
float E = E_ENTRADA;
float L = L_INDUTOR;
float C = C_SAIDA;
float R = R_CARGA;

//
// Constantes do modelo, calculadas em controle_inicializacao() para a ISR
// so multiplicar (divisao em float e lenta no C28x)
//
static float E_Ts_div_L = 0;
static float Ts_div_L   = 0;
static float Ts_div_C   = 0;
static float Ts_div_RC  = 0;

static pi_t pi_corrente;
static pi_t pi_tensao;

int sOpt = 0;   // ultimo estado escolhido pelo FCS-MPC

//
// controlador_pi - PI discreto: u = kp*e + ki*soma(e), com ki = KI*Ts
//
float controlador_pi(pi_t *pi, float erro)
{
    float uP;
    float uI;

    uP = pi->kp * erro;
    pi->integral = pi->integral + erro;
    uI = pi->integral * pi->ki;

    return (uP + uI);
}

//
// controle_inicializacao - Ganhos dos PI e constantes do modelo para um Ts
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

    Ts_div_L   = Ts / L;
    E_Ts_div_L = E * Ts_div_L;
    Ts_div_C   = Ts / C;
    Ts_div_RC  = Ts / (R * C);
}

//
// controle - PI em cascata: tensao (externa) -> iL_ref -> corrente -> duty
//
float controle(float v, float i, float vo_ref)
{
    float erro_i;

    iL_ref_pi = controlador_pi(&pi_tensao, vo_ref - v);
    erro_i    = iL_ref_pi - i;

    return controlador_pi(&pi_corrente, erro_i);
}

//
// controlador_fcs_mpc_corrente - Escolhe o estado que leva iL[k+1] mais
//                                perto de iL_ref
//
int controlador_fcs_mpc_corrente(float vo, float iL, float iL_ref)
{
    int j;
    float gi;
    float iL_k1;
    float giOpt = 1e5;

    int s[2] = {1, 0};

    for(j = 0; j < 2; j++)
    {
        iL_k1 = iL + E_Ts_div_L * (float)s[j] - vo * Ts_div_L;

        gi = (iL_ref - iL_k1) * (iL_ref - iL_k1);

        if(gi < giOpt)
        {
            sOpt  = s[j];
            giOpt = gi;
        }
    }

    return sOpt;
}

//
// controlador_fcs_mpc_tensao - Compensa o atraso (k+1 com o estado ja
//                              aplicado) e escolhe o estado para k+2
//
int controlador_fcs_mpc_tensao(float vo, float iL, float vo_ref)
{
    int j;
    float gv;
    float iL_k1, iL_k2;
    float vo_k1, vo_k2;
    float iL_ref_v;
    float gvOpt = 1e5;
    float peso;

    int s[2] = {1, 0};

    // com compensacao
    iL_k1 = iL + E_Ts_div_L * (float)sOpt - vo * Ts_div_L;
    vo_k1 = Ts_div_C * iL_k1 + vo * (1 - Ts_div_RC);

    // sem compensacao
    // iL_k1 = iL;
    // vo_k1 = vo;

    iL_ref_v = vo_ref / R;
    for(j = 0; j < 2; j++)
    {
        iL_k2 = iL_k1 + E_Ts_div_L * (float)s[j] - vo_k1 * Ts_div_L;
        vo_k2 = Ts_div_C * iL_k2 + vo_k1 * (1 - Ts_div_RC);

        peso = (iL_k2 >= iL_limit) ? FCS_V_PENALIDADE : 0.0f;

        gv = FCS_V_PESO_I * (iL_ref_v - iL_k2) * (iL_ref_v - iL_k2)
           + FCS_V_PESO_V * (vo_ref - vo_k2) * (vo_ref - vo_k2)
           + peso;

        if(gv < gvOpt)
        {
            sOpt  = s[j];
            gvOpt = gv;
        }
    }

    return sOpt;
}
