import numpy as np
import matplotlib.pyplot as plt

plt.style.use('bmh')
t = np.linspace(0, 5, 500)
alpha = np.zeros_like(t)
alpha[t > 0.5] = 0.15

D = 5000; C = 1.4; B = 7.0; E = -0.2
Fy_ss = D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))

v = 20
Lrel = 0.45
tau = Lrel / v

Fy_trans = np.zeros_like(t)
for i in range(1, len(t)):
    dt = t[i] - t[i-1]
    Fy_trans[i] = Fy_trans[i-1] + (Fy_ss[i] - Fy_trans[i-1]) / tau * dt

plt.figure(figsize=(7, 3))
plt.plot(t, Fy_ss, '--k', lw=2, label='Steady-State Pacejka (Rigid)')
plt.plot(t, Fy_trans, '-r', lw=2.5, label=r'Transient (Relaxation Lag $\tau=L_{relax}/v_x$)')
plt.xlim(0.4, 1.2)
plt.ylim(-500, 5500)
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='lower right')
plt.xlabel('Time [s]', fontweight='bold')
plt.ylabel(r'Lateral Force $F_y$ [N]', fontweight='bold')
plt.title('Tire Force Response to Step Steer (Hardware Simulation)', fontweight='bold')
plt.tight_layout()
plt.savefig('d:/Projects/Bajaj ohm 2/output_step_steer.png', dpi=300)
