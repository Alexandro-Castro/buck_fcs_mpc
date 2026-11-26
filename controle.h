//###########################################################################
//
// controle.h - Controladores do buck: PI em cascata e FCS-MPC
//
//###########################################################################

#ifndef CONTROLE_H
#define CONTROLE_H

typedef struct {
    float kp;
    float ki;
    float integral;
} pi_t;

//
// Limite de corrente do custo do FCS de tensao (alteravel no Watch)
//
extern volatile float iL_limit;

//
// Referencia de corrente gerada pela malha de tensao do PI (para o Watch)
//
extern volatile float iL_ref_pi;

//
// Parametros do modelo (valores iniciais em config.h; usados a cada partida)
//
extern float E;
extern float L;
extern float C;
extern float R;

float controlador_pi(pi_t *pi, float erro);
void  controle_inicializacao(float Ts);
float controle(float v, float i, float vo_ref);
int   controlador_fcs_mpc_corrente(float vo, float iL, float iL_ref);
int   controlador_fcs_mpc_tensao(float vo, float iL, float vo_ref);

#endif // CONTROLE_H
