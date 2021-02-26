# Code to read only the most recent piece of data
import numpy as np
import pandas as pd

import os.path
from os import path

data = pd.read_csv('distance.csv')
bottom = data.tail(1)
print('moving by:')
print(bottom)

x = bottom["dx"]
x = x.iloc[0]
print(abs(x)<1)

y = bottom["dy"]
y = y.iloc[0]

print(int(np.sign(x)))
print(round(abs(y/x))*np.sign(y))
