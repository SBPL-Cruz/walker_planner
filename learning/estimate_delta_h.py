import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    file_name = sys.argv[1]
    data = []
    with open(file_name) as f:
        lines = f.readlines()
        for line in lines:
            data.append((line.strip()).split())
    delta_hs = []
    for line in data:
        if(float(line[1]) > 0):
            delta_hs.append(float(line[0]) / float(line[1]))
    # Filter outliers
    delta_hs = np.array(delta_hs)
    max_val = np.max(delta_hs)
    mean = np.mean(delta_hs)
    std = np.std(delta_hs)
    delta_hs = delta_hs[delta_hs < (mean + 6 * std) ]

    plt.plot(delta_hs)
    plt.show()

    print("var: ", np.var(delta_hs))

    print(np.mean(delta_hs))

