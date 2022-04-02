import pandas as pd

df = pd.read_csv(r"ros_packages/driver_ui/driver_ui/idealvelo.csv", header = None, delim_whitespace = True)
print(df.max())
print(df.min())
print(df.max() - df.min())
#print(df.head())