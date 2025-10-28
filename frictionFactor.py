# -*- coding: utf-8 -*-
"""
Created on Tue Oct 28 08:42:05 2025

@author: frekih
"""

import numpy as np

d = 0.1 # diameter [m]
mu = 1 # dynamic viscosity [kg/m*s]
rho = 1 # density [kg/m3]
u = 1 # velocity [m/s]



r = ep / d # roughness [-]
v = mu / rho # kinematic viscosity [m2/s]
Re = v * d / v

f = 64 / Re # for laminar flow
(1 / np.sqrt(f)) == -2 * np.log10((ep / 3.7 * d) + (2.51 / Re * np.sqrt(f))) # for turbulent flow