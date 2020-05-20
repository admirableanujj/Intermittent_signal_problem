import torch
import torchvision
import torchvision.transforms as transforms
import numpy as np
from global_vars import global_vars
import logging
PATH = '.\\weights\\weight.pth'
class Neuralnet:

    def __init__(self, x_in, y_in, load_flag):
        self.neural_net_train(x_in, y_in, load_flag)

    # def __init__(self):
    #     pass

    def neural_net_train(self, x_in, y_in, load_flag):
        # N is batch size; D_in is input dimension;
        # H is hidden dimension; D_out is output dimension.
        N, D_in, H, D_out = 64, 1000, 30, 10
        # Create random Tensors to hold inputs and outputs
        # x = torch.randn(N, D_in)
        # y = torch.randn(N, D_out)
        # print(np.shape(x_in))
        self.x = torch.from_numpy(x_in)
        self.y = torch.from_numpy(y_in)

        # self.linear1 = torch.nn.Linear(self.x.size()[1], H) 
        # print(f'x.size():{self.x.size()[1]},H:{H}')
        # self.ReLU = torch.nn.ReLU() 
        # print(f'y.size():{self.y.size()[1]},H:{H}')
        # self.linear2 = torch.nn.Linear(H, self.y.size()[1]) 


        # Use the nn package to define our model and loss function.
        # self.model = torch.nn.Sequential(
        #     self.linear1(self.x.size()[1], H),
        #     self.ReLU,
        #     self.linear2(H, self.y.size()[1]),
        # )
        if load_flag:
            self.model.load_state_dict(torch.load(PATH))
        else:            
            self.model = torch.nn.Sequential(
                    torch.nn.Linear(self.x.size()[1], H),
                    torch.nn.ReLU(),
                    torch.nn.Linear(H, H),
                    torch.nn.ReLU(),
                    torch.nn.Linear(H, self.y.size()[1]),
                ).double()

            self.loss_fn = torch.nn.MSELoss(reduction='mean')

            # Use the optim package to define an Optimizer that will update the weights of
            # the model for us. Here we will use Adam; the optim package contains many other
            # optimization algoriths. The first argument to the Adam constructor tells the
            # optimizer which Tensors it should update.
            learning_rate = 1e-4
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
            # self.optimizer = torch.optim.Adagrad(self.model.parameters(), lr=learning_rate)
            # x_new = x
            loss_array = []
            # print(self.x.shape)
            # print(self.y.shape)
            for t in range(2000):
                y_pred = self.model(self.x)
                self.loss = self.loss_fn(y_pred, self.y)
                # loss  = self.neural_net_update(self.y,y_pred,t)
                self.optimizer.zero_grad()
                self.loss.backward()
                self.optimizer.step()
                loss_array.append(self.loss.detach().numpy().tolist())
            # torch.save(self.model.state_dict(),PATH)    
            global_vars.file_write( 'loss_array_train', loss_array)
   
    def neural_net_predict(self, x_new):
        model = self.model
         # Forward pass: compute predicted y by passing x to the model.
        y_pred = model(torch.from_numpy(x_new))
        self.model = model
        return y_pred

    def neural_net_update(self, y_new, y_pred, t):
        ##################
        loss_fn = self.loss_fn
        
        # print(f'y_new: {y_new}, y_pred: {y_pred}')
        # Compute and print loss.
        self.loss = loss_fn(y_pred, y_new)
        # logging.info(y_pred)
        # logging.info(y_new)
        
        # print(t, self.loss.item())
        
        # Before the backward pass, use the optimizer object to zero all of the
        # gradients for the variables it will update (which are the learnable
        # weights of the model). This is because by default, gradients are
        # accumulated in buffers( i.e, not overwritten) whenever .backward()
        # is called. Checkout docs of torch.autograd.backward for more details.
        # self.optimizer.zero_grad()

        # Backward pass: compute gradient of the loss with respect to model
        # parameters
        # self.loss.backward()

        # Calling the step function on an Optimizer makes an update to its
        # parameters
        # self.optimizer.step()
            # self.model = model
            # self.loss_fn = loss_fn
            # self.optimizer = optimizer
        return self.loss
    
    @staticmethod
    def data_in_prrocessor(rows):
        x_in  = []
        y_in = []
        for row in range(1,len(rows)-3):
            if not rows[row]:
                pass
            else:
                temp = rows[row]
                # px,py,pz,dpx,dpy,dpz,phi,theta,psi,ax,ay,az,w1,w2,w3,w4
                # print(f'x:{row}')
                # if row ==1 or row == len(rows)-4:
                #     print(f'x:{temp}')
                x_in.append(list(map(float,temp)))
        for row in range(3,len(rows)-1):
            if not rows[row]:
                pass
            else:
                # px,py,pz,dpx,dpy,dpz,phi,theta,psi
                temp = rows[row]
                # if row ==3 or row == len(rows)-2:
                #     print(f'y:{temp}')
                # y_in.append([temp[0], temp[1], temp[2], temp[3], temp[4],temp[5], temp[6], temp[7],temp[8]])
                y_in.append(list(map(float,temp[0:9])))
        return np.asarray(x_in), np.asarray(y_in)