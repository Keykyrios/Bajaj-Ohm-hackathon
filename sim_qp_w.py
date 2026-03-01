import numpy as np
import matplotlib.pyplot as plt

plt.style.use('bmh')
e_gamma = np.linspace(-1.5, 1.5, 500)
e_0 = 0.2  # Threshold yaw error (rad/s) where intervention aggressively shifts
k = 15     # Steepness of transition

# Sigmoid transitioning W from 1 (efficiency) down to 0 (stability)
W = 1 - (1 / (1 + np.exp(-k * (np.abs(e_gamma) - e_0))))

plt.figure(figsize=(7, 3))
plt.plot(np.abs(e_gamma), W, '-g', lw=3, label=r'Sigmoid Weight $W$')
plt.axvline(e_0, color='r', linestyle='--', label=r'Threshold $|e_{\gamma,0}| = 0.2$ rad/s')
plt.xlim(0, 0.8)
plt.ylim(-0.1, 1.1)
plt.xlabel(r'Absolute Yaw Error $|e_\gamma|$ [rad/s]', fontweight='bold')
plt.ylabel('Weight Parameter W', fontweight='bold')
plt.title(r'QP Objective Weighting Transition: $W(e_\gamma)$', fontweight='bold')
plt.annotate('Pure Efficiency\n(Cruise)', xy=(0.05, 0.9), color='green', fontweight='bold')
plt.annotate('Pure Stability\n(Intervention)', xy=(0.4, 0.1), color='red', fontweight='bold')
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend()
plt.tight_layout()
plt.savefig('d:/Projects/Bajaj ohm 2/output_qp_w.png', dpi=300)
