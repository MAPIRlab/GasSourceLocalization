import cv2
import numpy as np
import sys

def quantize(image, N):
    return np.round(image*(N/255))*(255/N);

def generateMasks(path):
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = quantize(gray, 255)
    cv2.imwrite(f"gray.png", gray)
    unique_colors = np.unique(gray)
    print(f"Found {len(unique_colors)} unique colors in the image")
    i = 0
    for val in unique_colors:
         mask = np.zeros_like(gray)
         mask[gray == val] = 255
         cv2.imwrite(f"Mask_{i}.png", mask)
         i += 1



generateMasks(sys.argv[1])