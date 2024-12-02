import sys

import matplotlib.pyplot as plt
import pandas as pd

if len(sys.argv) != 2:
    print("Usage: python3 plot_csv.py <csv file>")
    sys.exit(1)

df = pd.read_csv(sys.argv[1])

df.plot(x='timestamp', y=['present_roll','present_pitch','present_yaw'])

plt.show()
