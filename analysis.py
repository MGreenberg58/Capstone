import numpy as np
import matplotlib.pyplot as plt
from fis import CraneFIS

fis = CraneFIS()

# # Pick nominal values
# bl_nom, cgd_nom, ph_nom = 50.0, 3.0, 5.0

# # Define ranges
# bl_range = np.linspace(0, 75, 50)
# cgd_range = np.linspace(0, 10, 50)
# ph_range = np.linspace(-30, 70, 50)

# # Store outputs
# ca_bl, ofb_bl = [], []
# for bl in bl_range:
#     ca, ofb = fis.evaluate(bl, cgd_nom, ph_nom)
#     ca_bl.append(ca)
#     ofb_bl.append(ofb)

# plt.figure(figsize=(10,4))
# plt.subplot(1,3,1)
# plt.plot(bl_range, ca_bl, label='CA')
# plt.plot(bl_range, ofb_bl, label='OF')
# plt.title("BoomLength sensitivity")
# plt.xlabel("BoomLength (m)"); plt.ylabel("Output"); plt.legend()

# # Repeat for CGDistance
# ca_cgd, ofb_cgd = [], []
# for cgd in cgd_range:
#     ca, ofb = fis.evaluate(bl_nom, cgd, ph_nom)
#     ca_cgd.append(ca)
#     ofb_cgd.append(ofb)

# plt.subplot(1,3,2)
# plt.plot(cgd_range, ca_cgd, label='CA')
# plt.plot(cgd_range, ofb_cgd, label='OF')
# plt.title("CGDistance sensitivity")
# plt.xlabel("CGDistance (m)"); plt.ylabel("Output"); plt.legend()

# # Repeat for CGDistance
# ca_ph, ofb_ph = [], []
# for ph in ph_range:
#     ca, ofb = fis.evaluate(bl_nom, cgd_nom, ph)
#     ca_ph.append(ca)
#     ofb_ph.append(ofb)

# plt.subplot(1,3,3)
# plt.plot(ph_range, ca_ph, label='CA')
# plt.plot(ph_range, ofb_ph, label='OF')
# plt.title("Payload Height sensitivity")
# plt.xlabel("Height (m)"); plt.ylabel("Output"); plt.legend()
# plt.show()

N = 1000
bl_samples = np.random.uniform(0, 75, N)
cgd_samples = np.random.uniform(0, 8, N)
ph_samples = np.random.uniform(0, 60, N)

ca_samples = []
for bl, cgd, ph in zip(bl_samples, cgd_samples, ph_samples):
    ca, _ = fis.evaluate(bl, cgd, ph)
    ca_samples.append(ca)

# Compute correlation
corr_bl = np.corrcoef(bl_samples, ca_samples)[0,1]
corr_cgd = np.corrcoef(cgd_samples, ca_samples)[0,1]
corr_ph = np.corrcoef(ph_samples, ca_samples)[0,1]

print("ControlAdjustment correlations:")
print(f"BoomLength: {corr_bl:.3f}, CGDistance: {corr_cgd:.3f}, PayloadHeight: {corr_ph:.3f}")

