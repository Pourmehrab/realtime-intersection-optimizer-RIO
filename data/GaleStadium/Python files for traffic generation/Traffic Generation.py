# -*- coding: utf-8 -*-
"""
Created on Wed Jul  3 14:35:11 2019

@author: zjerome
"""

import numpy as np
import math

u = 6.0
iteration = 10000
tot = 0
while iteration != 0:
    total = 0
    count = 0
    while total <= 600:
        number = np.random.exponential(scale=100.00, size = None)
        if number < 0.716531310573789:
            t = u * (-math.log(number))
            total += t
            count += 1
    tot += count
    iteration -= 1
average = tot/10000
print(average)