import numpy as np
import torch as torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

from utils import readData, writeToFile

class DNN(nn.Module):
    def __init__(self):
        super(DNN, self).__init__()
        self.fc1 = nn.Linear(5, 100)
        self.fc2 = nn.Linear(100, 10)
        self.fc3 = nn.Linear(10, 2)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = torch.sigmoid(self.fc3(x))
        return x


if __name__ == "__main__":
    # TensorBoard
    writer = SummaryWriter()

    # Get data
    file_name = "data/context_rewards.txt"
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    data = readData(file_name)

    print(X.shape, Y.shape)

    # Define a model
    model = DNN()
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.9)

    EPOCHS = 1000
    training_loss = []
    for epoch in range(EPOCHS):
        optimizer.zero_grad()
        output = model(X)
        loss = criterion(output, Y)
        loss.backward()
        optimizer.step()
        print("Loss: %.3f" % loss.item())
        # training_loss.append(loss.item())
        writer.add_scalar("Loss/train", loss.item(), epoch)
    writer.close()


    print("Finished training")

