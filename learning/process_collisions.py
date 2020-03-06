import os
import numpy as np
import matplotlib.pyplot as plt
4
if __name__ == "__main__":
    data = []
    with open( os.path.expanduser("~/.ros") + "/children_in_collision.txt") as f:
        data = f.readlines()
    data = [datum.split() for datum in data]
    expansions_collisions = np.array( [[datum[0], datum[-1]] for datum in
        data], dtype=int )
    collisions_base = expansions_collisions[expansions_collisions[:,1] == 4]
    collisions_fullbody = expansions_collisions[expansions_collisions[:,1] == 18]
    print(expansions_collisions.shape)

    plt.plot(collisions_base[:,0]/4.0, 'r', label="base")
    # plt.plot(collisions_fullbody[:,0]/18.0, 'b', label="fullbody")
    plt.legend()
    plt.show()
