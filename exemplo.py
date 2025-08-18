import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

imgds = pd.read_csv('gridex.csv')
print(imgds.columns)
print(imgds)
#x_d = imgds['X']
#y_d = imgds['Y']
#z_d = imgds['Z(intensidade)']
#df = pd.DataFrame({
#    'x': x_d,
#    'y': y_d,
#    'z': z_d 
#})
#imggrid = df.pivot(index = 'y', columns = 'x', values = 'z')
fig, ax = plt.subplots()
img = ax.imshow(imggri.values,
                cmap='hot',
                origin='lower',
                # Use the actual coordinates for the axis labels
                extent=[-5, 15, -5, 15],
                aspect='auto')

cbar = fig.colorbar(img, ax=ax)
cbar.set_label('Intensity')
ax.set_title('Intensity Image from Pivoted Data')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
print(imggrid)
print(imggrid.values)
plt.show()

