import numpy as np

def readData(file_name):
    data = []
    with open(file_name, 'r') as f:
        line = f.readline()
        while(line):
            data.append([int(val) for val in line.split()])
            line = f.readline()
    return np.array(data)

def writeToFile(data, file_name):
    with open(file_name, 'w') as f:
        for context, params in data.items():
            string = ""
            for ele in context:
                string += str(ele)
                string += "\t"
            for param in params:
                string += str(param)
                string += "\t"
            string += "\n"
            f.write(string)


