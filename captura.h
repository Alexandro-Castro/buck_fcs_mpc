//###########################################################################
//
// captura.h - Grava vo, iL, a acao de controle e a referencia na ISR, com
//             pre-gatilho, para exportar pelo CCS (Memory Browser -> Save)
//
// Uso no Watch:
//   cap_gatilho   = 0 imediato, 1 na partida (comando = 1), 2 quando
//                   iL_ref ou vo_ref mudam com o conversor rodando
//   cap_decimacao = grava 1 a cada N amostras (1 = todas; janela de
//                   CAP_N * N * Ts)
//   cap_arma      = 1 comeca (arma) uma captura
//   cap_estado    = 0 parada, 1 armada, 2 gravando, 4 organizando,
//                   3 pronta. Pronta: cap_vo[0..CAP_N-1] etc. em ordem
//                   cronologica, gatilho em cap_idx_gatilho.
//
//###########################################################################

#ifndef CAPTURA_H
#define CAPTURA_H

#include "F28x_Project.h"

#define CAP_N       1000U       // amostras por canal
#define CAP_PRE     100U        // amostras antes do gatilho

//
// Eventos informados pela ISR a cada amostra (mascara de bits)
//
#define CAP_EV_PARTIDA      0x0001U
#define CAP_EV_REFERENCIA   0x0002U

typedef enum {
    CAP_PARADA     = 0,
    CAP_ARMADA     = 1,
    CAP_GRAVANDO   = 2,
    CAP_PRONTA     = 3,
    CAP_ORGANIZAR  = 4      // a ISR terminou; o laco principal reordena
} cap_estado_t;

extern volatile Uint16 cap_arma;
extern volatile Uint16 cap_gatilho;
extern volatile Uint16 cap_decimacao;
extern volatile cap_estado_t cap_estado;
extern volatile Uint16 cap_idx_gatilho;

extern float cap_vo[CAP_N];     // V
extern float cap_iL[CAP_N];     // A
extern float cap_u[CAP_N];      // estado da chave (FCS) ou duty (PI)
extern float cap_ref[CAP_N];    // referencia seguida (A ou V)

void captura_amostra(float vo, float iL, float u, float ref, Uint16 eventos);
void captura_fundo(void);

#endif // CAPTURA_H
