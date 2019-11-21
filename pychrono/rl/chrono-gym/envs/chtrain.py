# -*- coding: utf-8 -*-
"""
Created on Thu Jan 10 11:01:21 2019

@author: SB
"""

import chtrain_vehicle


def Init(env_name, render):
       if env_name=='ChronoVehicle':
              return chtrain_vehicle.Model(render)
       elif env_name=='ChronoVehicle2':
              return chtrain_vehicle.Model(render)
       elif env_name=='ChronoVehicle3':
              return chtrain_vehicle.Model(render)
       else:
              print('Invalid environment name')
