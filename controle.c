#include "controle.h"

pi_t pi_corrente;
pi_t pi_tensao;

float iL_ref = 1.0;
float vo_ref = 10.0;

float iL_limit = 2.0;

float Tss = 0;
float E = 50;
float L = 1.5e-3;
float R = 15;
float C = 220e-6;

float E_Ts_div_L = 0;
float Ts_div_L   = 0;
float Ts_div_C   = 0;
float Ts_div_RC  = 0;

int sOpt = 0;



float controlador_pi(pi_t *pi, float erro)
{
    float uP;
    float uI;

    uP = pi->kp * erro;
    pi->integral = pi->integral + erro;
    uI = pi->integral * pi->ki;

    return (uP + uI);
}

void  controle_inicializacao(float Ts)
{
    pi_corrente.kp = 1.0;
    pi_corrente.ki = 1000 * Ts;
    pi_corrente.integral = 0;

    pi_tensao.kp = 0.1;
    pi_tensao.ki = 10 * Ts;
    pi_tensao.integral = 0;

    Tss = Ts;

   Ts_div_L   = Ts/L;
   E_Ts_div_L = E * Ts_div_L;
   Ts_div_C   = Ts / C;
   Ts_div_RC  = Ts / (R*C);
    
}

float controle(float v, float i)
{
    float erro_i; 

    iL_ref = controlador_pi(&pi_tensao, vo_ref - v);
    erro_i = iL_ref - i;

    float out = controlador_pi(&pi_corrente, erro_i);
    
    return out;
}



int controlador_fcs_mpc_corrente(float vo, float iL)
{
    int j;
    float gi;
    float iL_k1;
    float giOpt = 1e5;

    int s[2] = {1, 0};

    for(j=0; j<2; j++)
    {
         iL_k1 = iL + E_Ts_div_L * (float)s[j] - vo * Ts_div_L;
         
         gi = (iL_ref - iL_k1) *  (iL_ref - iL_k1);
        
         if(gi < giOpt) {
            sOpt = s[j];
            giOpt = gi;
         }
    }

    return sOpt;
   
}


int controlador_fcs_mpc_tensao(float vo, float iL)
{
    int j;
    float gv;
    float iL_k1, iL_k2;
    float vo_k1, vo_k2;
    float iL_ref_v;
    float gvOpt = 1e5;
    float peso = 0.0;

    int s[2] = {1, 0};

    // com compensação
    iL_k1 = iL + E_Ts_div_L * (float)sOpt - vo * Ts_div_L;
    vo_k1 = Ts_div_C * iL_k1 + vo * (1 - Ts_div_RC);

    // sem compensação
    // iL_k1 = iL;
    // vo_k1 = vo;

    iL_ref_v = vo_ref / R;
    for(j=0; j<2; j++)
    {
        iL_k2 = iL_k1 + E_Ts_div_L * (float)s[j] - vo_k1 * Ts_div_L;
        vo_k2 = Ts_div_C * iL_k2 + vo_k1 * (1 - Ts_div_RC);

        if (iL_k2 >= iL_limit){
            peso = 1000;
        } else {
            peso = 0.0;
        }

         gv = (iL_ref_v - iL_k2)*(iL_ref_v - iL_k2) + 2*(vo_ref - vo_k2)*(vo_ref - vo_k2) + peso;
        
         if(gv < gvOpt) {
            sOpt = s[j];
            gvOpt = gv;
         }
    }
    return sOpt;
   
}
