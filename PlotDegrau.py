import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def carregar(csv):
    d = np.genfromtxt(csv, delimiter=",", names=True)
    return d["t_s"], d["sp"], d["ang"]
def acomodacao(t, sp, ang, t_skip=4.5):
    idx = int(np.searchsorted(t, t_skip))
    return t[idx:] - t[idx], sp[idx:], ang[idx:]
def main():
    if len(sys.argv) < 3:
      sys.exit("Uso: plot.py saida.png 'Label=arquivo.csv'")
    out = sys.argv[1]
    itens = [a.partition("=")[::2] for a in sys.argv[2:]] 
    fig, ax = plt.subplots(figsize=(11, 5))
    cores = ["tab:blue", "tab:red", "tab:green", "tab:purple"]
    sp_ref = None
    for k, (label, csv) in enumerate(itens):
        t, sp, ang = acomodaca(carregar(csv))
        if sp_ref is None:
         ax.plot(t, sp, "--", color="0.4", lw=1.8, label="Setpoint", zorder=1)
         sp_ref = True
        ax.plot(t, ang, "-", color=cores[k % len(cores)], lw=1.4, label=label, zorder=2)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Angulo (graus)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"Figura salva: {out}")
if __name__ == "__main__":
    main()
