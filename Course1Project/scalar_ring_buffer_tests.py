import numpy as np
from scalar_ring_buffer import ScalarRingBuffer

srb1 = ScalarRingBuffer(3)
srb1.insert(1.3)
if(not np.isclose(srb1.sum(), 1.3)):
    raise Exception("Huh?")
srb1.insert(4.1)
if(not np.isclose(srb1.sum(), 5.4)):
    raise Exception("Huh?")
srb1.insert(0.3)
if(not np.isclose(srb1.sum(), 5.7)):
    raise Exception("Huh?")
srb1.insert(0.4)
if(not np.isclose(srb1.sum(), 4.8)):
    raise Exception("Huh?")
srb1.insert(0.5)
if(not np.isclose(srb1.sum(), 1.2)):
    raise Exception("Huh?")
srb1.insert(0.6)
if(not np.isclose(srb1.sum(), 1.5)):
    raise Exception("Huh?")
srb1.insert(0.7)
if(not np.isclose(srb1.sum(), 1.8)):
    raise Exception("Huh?")