"""Quick demo: run the EKF on recording A and print the headline metrics (~5 s)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from orientation_ekf import load, run_filter, default_config
from orientation_ekf.metrics import summarize

m = load('A_all_sensors')
r = run_filter(m, default_config())
s = summarize(m.t, r['x'], m.ref)
print(f"recording A: {len(m.t)} samples, {m.t[-1]:.0f} s | acc updates used {r['acc_ok'].mean():.0%}, mag updates used {r['mag_ok'].mean():.0%}")
for k, v in s.items():
    print(f'  {k:20s} {v:7.2f}', 'fraction' if 'within' in k else 'deg')
