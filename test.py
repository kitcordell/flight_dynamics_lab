import pandas as pd
import matplotlib.pyplot as plt


data = pd.read_csv(
    "Data.txt",
    sep="|",
    engine="python",
    skipinitialspace=True
)


print(data.columns.tolist())

print(data.head())

data = data.iloc[6000:9000]
print(data)
time = data["_totl,_time "]

print(time.iloc[1])

time = time - time.iloc[1]

print(time.iloc[1])

pitch = data["pitch,__deg "]
elevator_deflection = data["elev1,__deg .1"]


plt.plot(time,pitch, label = "pitch angle")
plt.plot(time,elevator_deflection, label = "elevator deflection")
plt.legend()
plt.show()
