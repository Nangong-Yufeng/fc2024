import numpy as np
from utils.location import geodetic_to_enu, enu_to_geodetic

center = [38.55838809, 115.14061104, 20]
line_p = [38.557914, 115.139903, 20]
center_oth = geodetic_to_enu(*[38.55816538, 115.14085795, 20], *center)

target = geodetic_to_enu(*line_p, *center)
line = (np.array(target) - np.array([0, 0, 0])) / np.linalg.norm(np.array(target) - np.array([0, 0, 0]))
center_A = center_oth + line * 255
tmp = enu_to_geodetic(*center_A, *center)
print(tmp)

vet = np.array(center_oth) / np.linalg.norm(np.array(center_oth))
center_A = center_A - vet * 25
tmp = enu_to_geodetic(*center_A, *center)
print(tmp)

num1 = enu_to_geodetic(*(center_A - vet * 10 - line * 10), *center)
num2 = enu_to_geodetic(*(center_A - vet * 10 + line * 10), *center)
num3 = enu_to_geodetic(*(center_A + vet * 10 + line * 10), *center)
num4 = enu_to_geodetic(*(center_A + vet * 10 - line * 10), *center)
print(f"1: {num1}")
print(f"2: {num2}")
print(f"3: {num3}")
print(f"4: {num4}")

target = geodetic_to_enu(*line_p, *center)
line = (-np.array(target) - np.array([0, 0, 0])) / np.linalg.norm(np.array(target) - np.array([0, 0, 0]))
center_B = center_oth + line * 205
tmp = enu_to_geodetic(*center_B, *center)
print(tmp)

vet = np.array(center_oth) / np.linalg.norm(np.array(center_oth))
center_B = center_B - vet * 25
tmp = enu_to_geodetic(*center_B, *center)
print(tmp)

num1 = enu_to_geodetic(*(center_B - vet * 10 - line * 10), *center)
num2 = enu_to_geodetic(*(center_B - vet * 10 + line * 10), *center)
num3 = enu_to_geodetic(*(center_B + vet * 10 + line * 10), *center)
num4 = enu_to_geodetic(*(center_B + vet * 10 - line * 10), *center)
print(f"1: {num1}")
print(f"2: {num2}")
print(f"3: {num3}")
print(f"4: {num4}")
