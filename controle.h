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
// Referencias e limites (alteraveis no Watch)
//
extern volatile float iL_ref;
extern volatile float vo_ref;
extern volatile float iL_limit;

//
// Parametros do modelo (valores iniciais em config.h)
//
extern float E;
extern float L;
extern float C;
extern float R;

float controlador_pi(pi_t *pi, float erro);
void  controle_inicializacao(float Ts);
float controle(float v, float i);
int   controlador_fcs_mpc_corrente(float vo, float iL);
int   controlador_fcs_mpc_tensao(float vo, float iL);

#endif // CONTROLE_H
