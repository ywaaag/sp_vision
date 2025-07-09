import math

w = 1280*4.8*1e-3 #像素值*像元尺寸
h = 1024*4.8*1e-3
print(f'{w=}\t{h=}')

for f in [6, 8, 12]:
    fov_h = 2 * math.atan(w / 2 / f) / math.pi * 180
    fov_v = 2 * math.atan(h / 2 / f) / math.pi * 180
    print(f'{f=}\t{fov_h=:.1f}\t{fov_v=:.1f}')