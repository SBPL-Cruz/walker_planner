import os
import random

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

from model_constants import *


class CVAE(nn.Module):

    def __init__(self,
                 x_dmin=X_DIM,
                 c_dmin=C_DIM,
                 run_id=1,
                 experiment_path_prefix=EXPERIMENT_PATH_PREFIX,
                 hiddens=HIDDEN_LAYERS,
                 initial_learning_rate=INITIAL_LEARNING_RATE,
                 lr_update_cycles=LEARNING_RATE_UPDATE_CYCLES,
                 weight_decay=WEIGHT_DECAY,
                 cuda_available=CUDA_AVAILABLE,
                 use_tboard=USE_TENSORBOARD,
                 tf_sub_path=TF_PATH):
        """
        :param num_nets: number of networks in the ensemble
        :param x_dim: state dimension
        :param c_dim: action dimension
        :param learning_rate:
        """
        super(CVAE, self).__init__()
        # self.sess = tf.Session()
        self.x_dim = x_dmin
        self.c_dim = c_dmin

        self.cuda_available = cuda_available
        self.device = torch.device('cuda' if CUDA_AVAILABLE else 'cpu')
        # K.set_session(self.sess)


        self.hiddens = hiddens

        self.model = self.create_network()
        self.initial_learning_rate = initial_learning_rate
        self.current_lr = self.initial_learning_rate
        self.weight_decay = weight_decay

        self.use_tboard = use_tboard
        self.tf_sub_path = tf_sub_path
        self.run_id = run_id
        self.experiment_path_prefix = experiment_path_prefix
        if self.cuda_available:
            self.models = [self.models[i].cuda() for i in range(self.num_nets)]
        if self.use_tboard:
            from tensorboardX import SummaryWriter
            self.tf_path = '{}/{}/{}/'.format(self.experiment_path_prefix, self.run_id, self.tf_sub_path)
            if not os.path.exists(self.tf_path):
                os.makedirs(self.tf_path)
            self.tboard = SummaryWriter(self.tf_path)

    def update_learning_rate(self, lr):
        for optimizer in self.optimizers:
            for param_group in optimizer.param_groups:
                param_group['lr'] = lr

    def get_current_lr(self, episode_num):
        return self.initial_learning_rate / pow(2.0, (episode_num // self.lr_update_cycles))

    def reset_current_lr(self, episode_num):
        this_lr = self.get_current_lr(episode_num)
        if ((self.current_lr > 1.1 * this_lr) or (self.current_lr < 0.9 * this_lr)):
            self.current_lr = this_lr
            self.update_learning_rate(self.current_lr)

    def load(self, model_path):
        self.model.load_state_dict(torch.load(model_path)['state_dict'])

    def save_model_weights(self, suffix):
        # Helper function to save your model / weights.
        model_path = os.path.join(self.experiment_path_prefix, self.run_id, 'model-{}.pkl'.format(suffix))
        torch.save({'state_dict': self.model.state_dict()}, model_path)

    def predict(self, x, c, model_idx):
        self.model.eval()
        x = x.to(self.device)
        c = c.to(self.device)
        return self.model(torch.cat([x, c], dim=1))

    def forward(self, model_input):
        self.model.train()
        return self.model(model_input)

    def create_network(self):
        if len(self.hiddens) > 0:
            modules = [
                nn.Linear(self.x_dim + self.c_dim, self.hiddens[0]),
                nn.ReLU(inplace=True)
            ]
            for i in range(1, len(self.hiddens)):
                modules.extend([
                    nn.Linear(self.hiddens[i - 1], self.hiddens[i]),
                    nn.ReLU(inplace=True)
                ])
            modules.extend([nn.Linear(self.hiddens[-1], 2 * self.x_dim)])
        else:
            modules = [nn.Linear(self.x_dim + self.c_dim, 2 * self.x_dim)]

        model = nn.Sequential(*modules)
        print(model)
        return model

    def train(self, replay_buffer, batch_size=128, epochs=5):
        """
        Arguments:
          inputs: state and action inputs.  Assumes that inputs are standardized.
          targets: resulting states
        """
        pass

    # TODO: Write any helper functions that you need