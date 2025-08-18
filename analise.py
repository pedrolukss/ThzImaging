import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.special import erf
tamanho = pd.read_csv('120x100_F_funcaoerro.csv')

def fit_erf(x, a, b, c, d):
    return a * erf((x-b)/c)+d

x = tamanho['x0000'] 
y = tamanho['y0000']
popt, pcov = curve_fit(fit_erf, x, y, p0=[1, 0,1, 0])
fig, ax = plt.subplots()
#ax.scatter(x,y)
#ax.plot(x, fit_erf(x, *popt), color = 'red')
#plt.show()
#print(popt)
yimg = np.tile(y, (121,1))
ygr = np.arange(122)
X, Y = np.meshgrid(x, ygr)
img = ax.imshow(yimg,
                extent=[X.min(), X.max(), Y.min(), Y.max()],
                cmap='hot',
                origin='lower',
                aspect='equal') # Use 'equal' if x and y scales are the same

# --- 3. Add Labels and a Colorbar ---
# A colorbar is essential for understanding the intensity scale.
cbar = fig.colorbar(img, ax=ax)
cbar.set_label('Intensity (Arbitrary Units)')

# Add titles and labels
ax.set_title('THz Beam Intensity Profile')
ax.set_xlabel('X Position (mm)')
ax.set_ylabel('Y Position (mm)')

# Show the final plot
plt.show()

