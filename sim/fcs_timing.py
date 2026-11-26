"""Compara o FCS-MPC original com o de atraso compensado, simulando o buck
com o timing do DSP: amostra no topo do contador (meio do intervalo de
chaveamento) e estado decidido aplicado so no zero seguinte (Ts/2 depois).

Uso: python fcs_timing.py   (so biblioteca padrao)
"""
E, L, C = 50.0, 1.5e-3, 220e-6
Ts = 25e-6                      # TBPRD = 625 com TBCLK = 50 MHz
h = Ts / 2
R_MODELO = 15.0                 # R usado no modelo do FCS de tensao
N_SUB = 50                      # passos de integracao por meio periodo


def planta(iL, vo, s, dt, R):
    """Integra o buck por dt com o estado s (RK4)."""
    d = dt / N_SUB
    for _ in range(N_SUB):
        def f(i, v):
            return (E * s - v) / L, (i - v / R) / C
        k1 = f(iL, vo)
        k2 = f(iL + k1[0] * d / 2, vo + k1[1] * d / 2)
        k3 = f(iL + k2[0] * d / 2, vo + k2[1] * d / 2)
        k4 = f(iL + k3[0] * d, vo + k3[1] * d)
        iL += d / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        vo += d / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    return iL, vo


def argmin(custo):
    return min((1, 0), key=custo)


# --- versao original (Euler de um passo inteiro a partir da amostra) ------
def fcs_i_original(iL, vo, s_at, ref):
    return argmin(lambda s: (ref - (iL + (E * s - vo) * Ts / L)) ** 2)


def fcs_v_original(iL, vo, s_at, vref, ilim=2.0):
    a, b, k = Ts / L, Ts / C, 1 - Ts / (R_MODELO * C)
    i1 = iL + (E * s_at - vo) * a
    v1 = b * i1 + vo * k

    def custo(s):
        i2 = i1 + (E * s - v1) * a
        v2 = b * i2 + v1 * k
        return ((vref / R_MODELO - i2) ** 2 + 2 * (vref - v2) ** 2
                + (1000 if i2 >= ilim else 0))
    return argmin(custo)


# --- versao nova (atraso compensado, custo no instante de comutacao) -------
def passo(iL, vo, s, dt):
    di = (E * s - vo) * dt / L
    return iL + di, vo + (iL + di / 2 - vo / R_MODELO) * dt / C


def fcs_i_novo(iL, vo, s_at, ref):
    iz = iL + (E * s_at - vo) * h / L
    return argmin(lambda s: (ref - (iz + (E * s - vo) * Ts / L)) ** 2)


def fcs_v_novo(iL, vo, s_at, vref, ilim=2.0):
    iz, vz = passo(iL, vo, s_at, h)

    def custo(s):
        ie, ve = passo(iz, vz, s, Ts)
        return ((vref / R_MODELO - ie) ** 2 + 2 * (vref - ve) ** 2
                + (1000 if ie >= ilim else 0))
    return argmin(custo)


def roda(ctrl, ref, R, n=16000):
    iL = vo = 0.0
    s_at, trocas, amostras = 0, 0, []
    for _ in range(n):
        iL, vo = planta(iL, vo, s_at, h, R)        # zero -> topo
        s_novo = ctrl(iL, vo, s_at, ref)           # amostra e decisao
        amostras.append((iL, vo))
        iL, vo = planta(iL, vo, s_at, h, R)        # topo -> zero
        trocas += s_novo != s_at
        s_at = s_novo
    reg = amostras[n // 2:]                        # regime
    med = [sum(a[j] for a in reg) / len(reg) for j in (0, 1)]
    pp = [max(a[j] for a in reg) - min(a[j] for a in reg) for j in (0, 1)]
    return med, pp, trocas / 2 / (n * Ts)


if __name__ == "__main__":
    print("FCS de corrente: erro da media de iL, ripple das amostras, f de chaveamento")
    for R in (15.0, 7.5):
        for ref in (0.6, 1.0, 1.4, 1.8):
            linha = f"  R = {R:4.1f}  iL_ref = {ref:.1f} A:"
            for nome, f in (("original", fcs_i_original), ("novo", fcs_i_novo)):
                (i, v), (ipp, vpp), fsw = roda(f, ref, R)
                linha += f"  {nome} {100*(i-ref)/ref:+6.1f}% {ipp:.2f} A {fsw/1e3:4.1f} kHz"
            print(linha)
    print("FCS de tensao (modelo com R = 15): erro da media de vo, ripple de vo")
    for R in (15.0, 7.5, 30.0):
        for vref in (5.0, 10.0, 20.0):
            linha = f"  R = {R:4.1f}  vo_ref = {vref:4.1f} V:"
            for nome, f in (("original", fcs_v_original), ("novo", fcs_v_novo)):
                (i, v), (ipp, vpp), fsw = roda(f, vref, R)
                linha += f"  {nome} {100*(v-vref)/vref:+6.1f}% {vpp:.3f} V"
            print(linha)
