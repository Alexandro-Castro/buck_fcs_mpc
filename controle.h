
#define FCS_MPC

typedef struct {
    float kp;
    float ki;
    float integral;
} pi_t;



float controlador_pi(pi_t *pi, float erro);
void  controle_inicializacao(float Ts);
float controle(float v, float i);
int controlador_fcs_mpc_corrente(float vo, float iL);
int controlador_fcs_mpc_tensao(float vo, float iL);
