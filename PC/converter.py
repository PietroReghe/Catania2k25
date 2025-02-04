from PIL import Image
import numpy as np


image_path = "mappa2.png"  
img = Image.open(image_path).convert("L") 


img_array = np.array(img)
print(img_array.shape) 

dpi = 300  # Risoluzione in punti per pollice
pixels_per_cm = 118  # Calcola pixel per cm
cell_size = int(pixels_per_cm)  # Dimensione cella (es. 118 pixel per cm)

# Dividi l'immagine in celle
h, w = img_array.shape  # Altezza e larghezza dell'immagine
grid = img_array.reshape(h // cell_size, cell_size, -1, cell_size).mean(axis=(1, 3))  # Media dei pixel per ogni cella

print(grid.shape)  

for row in range(0,w):
    for col in range(0,h):
        x_center = (row * pixels_per_cm -1) + pixels_per_cm/2
        y_center = (col * pixels_per_cm -1) + pixels_per_cm/2

image = Image.open(image_path)
position = (x_center, y_center)
color = image.getpixel(position)
print(f"The color at position {position} is {color}")

"""for row in char_matrix:
    print("".join(row))

with open("mappa_matrice.txt", "w") as f:
    for row in char_matrix:
        f.write("".join(row) + "\n")
"""



