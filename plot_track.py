import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('racetrack_cones.csv')

left = pd.DataFrame({
    'id': 0,
    'x': df[df['id'] == 0]['x'],
    'y': df[df['id'] == 0]['y']
})

right = pd.DataFrame({
    'id': 1,
    'x': df[df['id'] == 1]['x'],
    'y': df[df['id'] == 1]['y']
})

df = pd.concat([left, right], ignore_index=True)

fig, ax = plt.subplots(figsize=(10, 6))
sc = ax.scatter(df['x'], df['y'], c=df['id'], cmap='coolwarm', alpha=0.6, edgecolors='w')
plt.title('Track Cones')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.colorbar(sc, label='Cone Type (0: Left, 1: Right)')

def on_click(event):
    if event.inaxes == ax:
        # Trova il punto più vicino al click
        distances = ((df['x'] - event.xdata)**2 + (df['y'] - event.ydata)**2)
        idx = distances.idxmin()
        x, y = df.loc[idx, ['x', 'y']]
        print(f"Selected point: x={x}, y={y}")
        ax.plot(x, y, 'o', color='black', markersize=10)
        fig.canvas.draw()

fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()