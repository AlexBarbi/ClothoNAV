import pandas as pd
import matplotlib.pyplot as plt

# Carica il CSV
df = pd.read_csv('cones_abs.csv')

# Crea DataFrame per i coni sinistri
left = pd.DataFrame({
    'id': 0,
    'x': df[df['id'] == 0]['x'],
    'y': df[df['id'] == 0]['y']
})

# Crea DataFrame per i coni destri
right = pd.DataFrame({
    'id': 4,
    'x': df[df['id'] == 4]['x'],
    'y': df[df['id'] == 4]['y']
})

# Unisci e salva
df = pd.concat([left, right], ignore_index=True)

# Salva il DataFrame unito in un nuovo CSV
# df.to_csv('cones2.csv', index=False)
# Crea il grafico
plt.figure(figsize=(10, 6))
plt.scatter(df['x'], df['y'], c=df['id'], cmap='coolwarm', alpha=0.6, edgecolors='w')
plt.title('Track Cones')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.colorbar(label='Cone Type (0: Left, 1: Right)')
plt.show()
plt.close()