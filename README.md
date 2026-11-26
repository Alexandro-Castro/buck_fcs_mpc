# Conversor buck com FCS-MPC e PI no F28379D

Firmware de controle de um conversor buck síncrono (disciplina de Conversores Estáticos). Ele parte do código de apoio do professor, que está sem alterações no primeiro commit, e foi portado do controlSUITE v210 para o C2000Ware 26.01. Tem três controladores, escolhidos em tempo de execução:

- **PI em cascata**: malha de tensão gerando a referência de corrente, malha de corrente gerando o duty. PWM fixo de 20 kHz.
- **FCS-MPC de corrente**: segue `iL_ref`.
- **FCS-MPC de tensão**: segue `vo_ref`, com termo de corrente `vo_ref/R` e penalidade de sobrecorrente.

> Nada daqui foi testado na bancada ainda. Todas as versões compilam no CCS 21; os controladores foram conferidos em simulação (pasta `sim/`).

## Hardware assumido

| Item | Detalhe |
|---|---|
| DSP | LAUNCHXL-F28379D, CPU1, depuração pela XDS100v2 da placa |
| Clock | Cristal de 10 MHz sem o define `_LAUNCHXL_F28379D`: SYSCLK = 100 MHz, TBCLK = 50 MHz. Num controlCARD (20 MHz), trocar `TBCLK_HZ` para `100.0e6f` em `config.h` |
| Circuito (modelo) | E = 50 V, L = 1,5 mH, C = 220 µF, R = 15 Ω |
| Referência do ADC | VREFHI = 3,0 V (REF5030 da LaunchPad) |

| Pino | Função |
|---|---|
| GPIO0 / GPIO1 | EPWM1A (chave de cima) / EPWM1B (chave de baixo, complementar com tempo morto de 1 µs) |
| ADCINA0 | Tensão de saída vo (`KADC_V`) |
| ADCINA2 = CMPIN1P | Corrente no indutor iL (`KADC_I`), também ligada ao comparador de sobrecorrente |
| GPIO18 | Em 1 durante a ISR: frequência = 1/Ts e largura = tempo de execução |
| GPIO31 | LED de estado |
| GPIO2 | Vai a 1 depois que o PWM liga. Vem do código original, sem função documentada; conferir na placa do laboratório |
| GPIO19 | Saída em 0, sem uso |

## Compilar e gravar

1. *File → Import Projects* no CCS, pasta `buck_fcs_mpc` (usa o produto C2000Ware 26.01).
2. Configuração **CPU1_RAM** e *Build*.
3. *Debug* com `targetConfigs/TMS320F28379D.ccxml`. O programa roda da RAM e precisa ser carregado de novo depois de desligar a placa.

Depois de carregar, o programa mede o offset do sensor de corrente (estado 3) e fica **parado, com as duas chaves desligadas**, até receber `comando = 1`.

## Operação pelo Watch (Expressions)

| Variável | Escrita | Significado |
|---|---|---|
| `modo` | sim | 0 PI, 1 FCS de corrente, 2 FCS de tensão. Vale na próxima partida |
| `comando` | sim | 1 liga o controle, 0 desliga |
| `rearme` | sim | 1 sai da falha (volta a parado) |
| `recalibra` | sim | 1, com o conversor parado: mede de novo o offset de iL |
| `iL_ref`, `vo_ref` | sim | Referências (limitadas a `IL_REF_MAX` = 2 A e `VO_REF_MAX` = 25 V) |
| `rampa_iL`, `rampa_vo` | sim | Velocidade das rampas de referência (A/s, V/s). Valores grandes dão degraus |
| `iL_limit` | sim | Limite da penalidade de corrente do FCS de tensão |
| `E`, `L`, `C`, `R` | sim | Parâmetros do modelo. Valem na próxima partida; no FCS de tensão, `R` precisa ser a carga real |
| `estado` | não | 0 parado, 1 rodando, 2 falha, 3 calibrando |
| `falha`, `iL_falha`, `vo_falha` | não | Motivo da falha e medidas no instante |
| `iL`, `vo`, `iL_ref_atual`, `vo_ref_atual` | não | Medidas e referências seguidas (em rampa) |
| `flag`, `d`, `iL_ref_pi` | não | Estado da chave (FCS), duty e saída da malha de tensão (PI) |
| `offset_iL` | não | Offset medido do sensor de corrente (contagens) |
| `isr_us`, `isr_max_us`, `isr_atrasos` | `isr_max_us` | Tempo da amostragem até o fim da ISR e ISRs que passaram do zero do contador |
| `fsw_hz` | não | Frequência média de chaveamento (janela de 0,1 s) |

O LED pisca devagar parado, mais rápido rodando e muito rápido em falha.

**Pausar a CPU no depurador desliga as chaves** (TZ6) e leva à falha 3. Por isso as variáveis devem ser lidas e escritas com o programa rodando (atualização contínua no Expressions).

## Proteções

| Código | Falha | Como atua |
|---|---|---|
| 1 | `iL > IL_MAX` (2,5 A) | Software, na ISR |
| 2 | `vo > VO_MAX` (30 V) | Software, na ISR |
| 3 | CPU parada pelo depurador | TZ6 → trip do ePWM1 |
| 4 | Falha do relógio | TZ5 → trip |
| 5 | Outro trip do ePWM1 | — |
| 6 | Corrente instantânea > `IL_MAX_HW` (3 A) | CMPSS1 → X-BAR TRIP4 → DCAEVT1 → trip, sem passar pela CPU |
| 7 | Offset de iL > 250 contagens na calibração | Corrente circulando ou sensor com problema; o rearme mede de novo |

"Desligar" é sempre o trip one-shot do ePWM1: EPWM1A e EPWM1B em 0 depois do tempo morto, ou seja, as duas chaves abertas. Toda falha trava até `rearme = 1`.

## Controladores

O contador é up-down. A amostra é feita no topo (meio do intervalo de chaveamento) e a ISR roda uma vez por período Ts.

**PI** (`modo = 0`, Ts = 50 µs): o duty é `d0 + PI(iL_ref_pi − iL)`, com feedforward `d0 = vo/E`.
- O duty fica entre 0 e 0,95 e `iL_ref_pi` entre 0 e 2 A.
- Tem anti-windup por integração condicional.
- Ganhos do professor: corrente kp = 1, Ki = 1000; tensão kp = 0,1, Ki = 10.

**FCS-MPC** (`modo = 1` ou `2`, Ts = 25 µs, chaveamento de no máximo 20 kHz):
- O estado escolhido só entra no zero seguinte do contador, Ts/2 depois da amostra.
- Por isso o controlador primeiro leva a medida até esse zero com o estado já aplicado (compensação do atraso).
- Depois avalia cada candidato no **fim do intervalo seguinte**, no instante de comutação.
- Os picos e vales do ripple ficam centrados na referência e a média fica nela, sem ação integral.

Resultado da simulação (`sim/fcs_timing.py`), comparando com o algoritmo original:

| Caso | Original | Novo |
|---|---|---|
| FCS de corrente, iL_ref 1–1,8 A, erro da média | −7,4 a +1,5 % | −0,3 a +0,3 % |
| FCS de corrente, 1 A, ripple das amostras | 0,95 A | 0,50 A |
| FCS de corrente, 1 A, chaveamento | 8,7 kHz | 11,9 kHz |
| FCS de tensão, 10 V, R real = R do modelo | +3,4 % | 0,0 % |
| FCS de tensão, R real = 7,5 Ω (modelo 15 Ω) | −17 % | −27 % |

Limitações:
- Sem ação integral, o FCS de tensão depende de `R` ser a carga real, porque a referência de corrente é `vo_ref/R`.
- Partindo de vo = 0, o FCS de corrente só liga a chave com `iL_ref` acima de cerca de (E − vo)·Ts/(2L) ≈ 0,4 A. Com a rampa isso acontece sozinho; referências menores ficam em zero.

## Captura de dados

`captura.c` grava 1000 amostras por canal (`cap_vo`, `cap_iL`, `cap_u` e `cap_ref`), com 100 antes do gatilho.

1. Ajuste `cap_gatilho`: 0 imediato, 1 na partida, 2 quando `iL_ref`/`vo_ref` mudam.
2. Ajuste `cap_decimacao`: grava 1 a cada N amostras, para uma janela de 1000·N·Ts.
3. Escreva `cap_arma = 1` e provoque o evento.
4. Quando `cap_estado = 3`, os vetores estão em ordem cronológica e o gatilho está em `cap_idx_gatilho`. Exporte pelo Memory Browser do CCS (1000 valores float a partir de cada `cap_*`).

`cap_u` é o estado da chave no FCS e o duty no PI; `cap_ref` é a referência seguida.

## Primeiro teste na bancada

1. Só a placa no USB: LED piscando devagar, `estado = 0`, `offset_iL` pequeno.
2. Osciloscópio no GPIO18: 40 kHz no FCS (Ts = 25 µs). Confira `isr_max_us` bem abaixo de 12,5 µs e `isr_atrasos = 0`.
3. Sem potência, `comando = 1`: pulsos complementares em GPIO0/GPIO1.
4. Potência com a fonte baixa e limite de corrente. Confira `iL` e `vo` contra multímetro/osciloscópio e ajuste `KADC_I`/`KADC_V`.
5. Confira o limiar do CMPSS com uma corrente conhecida, antes de subir a referência.

## Arquivos

| Arquivo | Conteúdo |
|---|---|
| `config.h` | Todos os parâmetros: clock, PWM, circuito, ADC, limites, ganhos, pinos |
| `main.c` | Periféricos, máquina de estados, proteções, ISR |
| `controle.c/h` | PI em cascata e FCS-MPC |
| `captura.c/h` | Captura com pré-gatilho |
| `sim/fcs_timing.py`, `sim/pi_cascata.py` | Simulações com o timing do DSP (só biblioteca padrão do Python) |
| `F2837xD_*.c/.asm` | Suporte do dispositivo (v210; `GlobalVariableDefs` do C2000Ware) |
