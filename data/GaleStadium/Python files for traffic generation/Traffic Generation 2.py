# -*- coding: utf-8 -*-
"""
Created on Thu Jul 11 16:15:04 2019

@author: zjerome
"""

import numpy as np
import math

u = 3.5
total = 0
count = 0
while total <= 600:
    number = np.random.exponential(scale=1.00, size = None)
    if number < 0.564718122007759:
        t = u * (-math.log(number))
        total += t
        print (total)
        count += 1
print (count)
print (total/count)