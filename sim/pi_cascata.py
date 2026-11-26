"""Simula o PI em cascata do buck com o PWM do DSP: up-down, A alto em
torno do zero (duty = CMPA/TBPRD), amostra no topo do contador e CMPA
carregando no zero seguinte.

Compara o PI original (sem limites) com o novo (duty e iL_ref limitados,
anti-windup por integracao condicional e feedforward d0 = vo/E).

Uso: python pi_cascata.py   (so biblioteca padrao)
"""
E, L, C, R = 50.0, 1.5e-3, 220e-6, 15.0
Ts = 50e-6                      # TBPRD = 1250 com TBCLK = 50 MHz
N_SUB = 40


def planta(iL, vo, s, dt):
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


class PI:
    def __init__(self, kp, ki, umin=None, umax=None):
        self.kp, self.ki, self.s = kp, ki * Ts, 0.0
        self.umin, self.umax = umin, umax

    def __call__(self, e, umin=None, umax=None):
        umin = self.umin if umin is None else umin
        umax = self.umax if umax is None else umax
        u = self.kp * e + self.ki * (self.s + e)
        if umax is not None and u > umax:
            u = umax
            if e < 0:
                self.s += e
        elif umin is not None and u < umin:
            u = umin
            if e > 0:
                self.s += e
        else:
            self.s += e
        return u


def roda(novo, vref=10.0, n=4000, degrau=(2000, 15.0)):
    pv = PI(0.1, 10.0, 0.0, 2.0) if novo else PI(0.1, 10.0)
    pc = PI(1.0, 1000.0)
    iL = vo = 0.0
    d = 0.0
    log = []
    for k in range(n):
        if k == degrau[0]:
            vref = degrau[1]
        dap = min(max(d, 0.0), 1.0)          # o PWM satura sozinho
        t_on = dap * Ts / 2                  # A alto em [0, t_on) e (Ts-t_on, Ts]
        iL, vo = planta(iL, vo, 1, t_on)
        iL, vo = planta(iL, vo, 0, Ts / 2 - t_on)
        # amostra no topo
        iref = pv(vref - vo)
        if novo:
            d0 = min(max(vo / E, 0.0), 0.95)
            d = d0 + pc(iref - iL, 0.0 - d0, 0.95 - d0)
        else:
            d = pc(iref - iL)
        log.append((k * Ts, iL, vo, d, iref))
        iL, vo = planta(iL, vo, 0, Ts / 2 - t_on)
        iL, vo = planta(iL, vo, 1, t_on)
    return log


def resumo(log, k0, k1):
    seg = log[k0:k1]
    v = [x[2] for x in seg]
    d = [x[3] for x in seg]
    return (sum(v) / len(v), max(v) - min(v), min(d), max(d))


if __name__ == "__main__":
    for nome, novo in (("original", False), ("novo", True)):
        log = roda(novo)
        vmax = max(x[2] for x in log[:2000])
        dmin, dmax = min(x[3] for x in log), max(x[3] for x in log)
        a = resumo(log, 1500, 2000)
        b = resumo(log, 3500, 4000)
        print(f"{nome:9s} partida 0->10 V: pico de vo {vmax:5.2f} V | "
              f"10 V: media {a[0]:.2f} V pp {a[1]:.3f} V | "
              f"15 V: media {b[0]:.2f} V pp {b[1]:.3f} V | "
              f"duty calculado de {dmin:+.2f} a {dmax:+.2f}")
