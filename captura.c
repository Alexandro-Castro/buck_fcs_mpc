//###########################################################################
//
// captura.c - Captura com pre-gatilho de vo, iL, acao de controle e
//             referencia (ver captura.h)
//
// Armada, a captura grava continuamente num buffer circular. No gatilho
// grava mais CAP_N - pre amostras e para; o laco principal gira o buffer
// para que o indice 0 seja a amostra mais antiga.
//
//###########################################################################

#include "captura.h"

//
// 4 canais x 1000 floats = 8000 palavras: dois por bloco RAMGS de 4K
//
#pragma DATA_SECTION(cap_vo,  "ramgs0")
#pragma DATA_SECTION(cap_iL,  "ramgs0")
#pragma DATA_SECTION(cap_u,   "ramgs1")
#pragma DATA_SECTION(cap_ref, "ramgs1")
float cap_vo[CAP_N];
float cap_iL[CAP_N];
float cap_u[CAP_N];
float cap_ref[CAP_N];

volatile Uint16 cap_arma        = 0;
volatile Uint16 cap_gatilho     = 0;
volatile Uint16 cap_decimacao   = 1;
volatile cap_estado_t cap_estado = CAP_PARADA;
volatile Uint16 cap_idx_gatilho = 0;

static Uint16 idx;          // proxima posicao de escrita
static Uint16 gravadas;     // amostras gravadas desde que armou (ate CAP_N)
static Uint16 restantes;    // amostras que faltam depois do gatilho
static Uint16 decim;        // contador da decimacao

static void grava(float vo, float iL, float u, float ref)
{
    cap_vo[idx]  = vo;
    cap_iL[idx]  = iL;
    cap_u[idx]   = u;
    cap_ref[idx] = ref;

    idx++;
    if(idx >= CAP_N)
    {
        idx = 0;
    }
    if(gravadas < CAP_N)
    {
        gravadas++;
    }
}

//
// captura_amostra - Chamada na ISR a cada amostra
//
void captura_amostra(float vo, float iL, float u, float ref, Uint16 eventos)
{
    Uint16 disparou;
    Uint16 pre;

    //
    // Novo pedido (ignorado enquanto o laco principal reordena)
    //
    if(cap_arma && (cap_estado != CAP_ORGANIZAR))
    {
        cap_arma   = 0;
        idx        = 0;
        gravadas   = 0;
        decim      = 0;
        cap_estado = CAP_ARMADA;
    }

    if((cap_estado != CAP_ARMADA) && (cap_estado != CAP_GRAVANDO))
    {
        return;
    }

    //
    // Gatilho: so armada; o gatilho e a primeira amostra depois dele
    //
    if(cap_estado == CAP_ARMADA)
    {
        switch(cap_gatilho)
        {
            case 1:  disparou = (eventos & CAP_EV_PARTIDA)    != 0U; break;
            case 2:  disparou = (eventos & CAP_EV_REFERENCIA) != 0U; break;
            default: disparou = 1U;                                  break;
        }

        if(disparou)
        {
            pre = (gravadas < CAP_PRE) ? gravadas : CAP_PRE;
            restantes       = CAP_N - pre;
            cap_idx_gatilho = pre;
            decim           = 0;
            cap_estado      = CAP_GRAVANDO;
        }
    }

    //
    // Decimacao: grava 1 a cada cap_decimacao amostras
    //
    if(decim > 0U)
    {
        decim--;
        return;
    }
    decim = (cap_decimacao > 1U) ? (cap_decimacao - 1U) : 0U;

    grava(vo, iL, u, ref);

    if(cap_estado == CAP_GRAVANDO)
    {
        restantes--;
        if(restantes == 0U)
        {
            cap_estado = CAP_ORGANIZAR;
        }
    }
}

//
// inverte - Inverte cap_x[a..b] nos quatro canais
//
static void inverte(Uint16 a, Uint16 b)
{
    float t;

    while(a < b)
    {
        t = cap_vo[a];  cap_vo[a]  = cap_vo[b];  cap_vo[b]  = t;
        t = cap_iL[a];  cap_iL[a]  = cap_iL[b];  cap_iL[b]  = t;
        t = cap_u[a];   cap_u[a]   = cap_u[b];   cap_u[b]   = t;
        t = cap_ref[a]; cap_ref[a] = cap_ref[b]; cap_ref[b] = t;
        a++;
        b--;
    }
}

//
// captura_fundo - No laco principal: com a captura terminada, gira o
//                 buffer (tres inversoes) para que idx vire o indice 0
//
void captura_fundo(void)
{
    if(cap_estado != CAP_ORGANIZAR)
    {
        return;
    }

    if(idx != 0U)
    {
        inverte(0, idx - 1U);
        inverte(idx, CAP_N - 1U);
        inverte(0, CAP_N - 1U);
    }
    idx = 0;

    cap_estado = CAP_PRONTA;
}
