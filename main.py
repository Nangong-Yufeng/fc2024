import numpy as np
from utils.location import geodetic_to_enu

tmp = geodetic_to_enu(22.802891, 114.2956573, 20, 22.8029739, 114.2956918, 20)
tmp = np.array(tmp)
print(tmp, np.linalg.norm(tmp))