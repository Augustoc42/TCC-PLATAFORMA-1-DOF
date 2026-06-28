import argparse
import sys
try:
    import numpy as np
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")           
    import matplotlib.pyplot as plt
except ImportError as e:
    sys.exit(f"Dependencia ausente ({e}). Use o python do Anaconda " "(matplotlib/pandas/numpy).")

def load(path, skip_s=0.0):
    df = pd.read_csv(path)
    tcol = "t_ms" if "t_ms" in df.columns else ("t" if "t" in df.columns else None)
    if tcol is None:
        df["_t"] = np.arange(len(df)) / 20.0
    else:
        t = pd.to_numeric(df[tcol], errors="coerce")
        if t.max() > 1000:
            t = t / 1000.0
        df["_t"] = t - t.iloc[0]
    if skip_s > 0:
        df = df[df["_t"] >= skip_s].copy()
        df["_t"] = df["_t"] - df["_t"].iloc[0]
    return df
def col(df, name):
    return pd.to_numeric(df[name], errors="coerce") if name in df.columns else None

def main():
    ap = argparse.ArgumentParser(description="Plota cascata (cascata.py).")
    ap.add_argument("csv", help="cascata.")
    ap.add_argument("--overlay", help="angulo (A/B).")
    ap.add_argument("--out", help="Salva PNG.")
    ap.add_argument("--title", default=None)
    cfg = ap.parse_args()
    df = load(cfg.csv, cfg.skip_s)
    t = df["_t"].to_numpy()
    has_rate = ("rsp" in df.columns) or ("gr" in df.columns)
    n_panels = 4 if has_rate else 3
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 2.4 * n_panels), sharex=True)
    fig.suptitle(cfg.title or f"Cascata — {cfg.csv}", fontsize=12)
    #SP e Ang
    ax = axes[0]
    sp, ang = col(df, "sp"), col(df, "ang")
    if sp is not None:
        ax.plot(t, sp, color="0.4", lw=1.2, label="SP (alvo)")
    if ang is not None:
        ax.plot(t, ang, color="tab:blue", lw=1.0, label="Ang (Kalman)")
    if cfg.overlay:
        try:
            dfo = load(cfg.overlay, cfg.skip_s)
            ango = col(dfo, "ang")
            if ango is not None:
                ax.plot(dfo["_t"].to_numpy(), ango, color="tab:red", lw=1.0,
                        ls="--", label=f"Ang B ({cfg.overlay})")
        except Exception as e:
            print(f"ERRO")
    ax.set_ylabel("angulo (deg)"); ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.3)
    #Erro
    ax = axes[1]
    err = col(df, "err")
    if err is not None:
        ax.plot(t, err, color="tab:orange", lw=1.0, label="Erro")
        ax.axhline(0, color="0.7", lw=0.8)
    ax.set_ylabel("erro (deg)"); ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.3)
    idx = 2
    #RSP e GR
    if has_rate:
        ax = axes[idx]; idx += 1
        rsp, gr = col(df, "rsp"), col(df, "gr")
        if rsp is not None:
            ax.plot(t, rsp, color="tab:green", lw=1.0, label="RSP (setpoint de taxa)")
        if gr is not None:
            ax.plot(t, gr, color="0.5", lw=0.8, label="GR (taxa real)")
        ax.set_ylabel("taxa (deg/s)"); ax.legend(loc="best", fontsize=8); ax.grid(alpha=0.3)
    print(f"Figura salva: {out}")
if __name__ == "__main__":
    main()
