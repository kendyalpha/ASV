import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np


np.random.seed(42)
df = pd.DataFrame(np.random.randn(1000, 5),
                  columns=['a', 'b', 'c', 'd', 'e'])
df['a'] = df['a']+1
print(df)

print(df.cov())
