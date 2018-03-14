# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import torch, numpy as np

class MLP(torch.nn.Sequential):

    def __init__(self, shape, name="MLP"):

        # architecture; e.g. [3, 20, 20, 20, 2]
        self.shape = shape

        # number of inputs
        self.ni = self.shape[0]

        # number of outputs
        self.no = self.shape[-1]

        # number of layers
        self.nl = len(self.shape)

        # training and testing loss data
        self.ltrn, self.ltst = list(), list()

        # name of network
        self.name = name

        # network operations
        self.ops = [torch.nn.BatchNorm3d(self.shape[0])]
        #self.ops = list()

        # apply operations
        for i in range(self.nl - 1):

            # linear layer
            self.ops.append(torch.nn.Linear(self.shape[i], self.shape[i + 1]))

            # batch normalisation
            self.ops.append(torch.nn.BatchNorm3d(self.shape[i + 1]))


            # if penultimate layer
            if i == self.nl - 2:

                # output between 0 and 1
                #self.ops.append(torch.nn.Hardtanh(min_val=0, max_val=1))
                self.ops.append(torch.nn.Sigmoid())

            # if hidden layer
            else:

                # activation
                self.ops.append(torch.nn.LeakyReLU())
                pass

        # initialise neural network
        torch.nn.Sequential.__init__(self, *self.ops)
        self.double()

    @staticmethod
    def format(x):
        if isinstance(x, np.ndarray):
            x = torch.autograd.Variable(torch.from_numpy(x)).double()
        elif isinstance(x, torch.DoubleTensor):
            x = torch.autograd.Variable(x).double()
        elif isinstance(x, torch.autograd.Variable):
            x = x.double()
        else:
            raise TypeError("Unable to format input.")
        return x

    def forward(self, x):

        # format input
        x = self.format(x)

        # apply operations
        for op in self.ops:
            x = op(x)

        # return transformation
        return x

    def predict(self, x):

        # format
        x = self.format(x).data.numpy()

        # predict
        return self.forward(x).data.numpy()

    def train(self, idat, odat, epo=50, batches=10, lr=1e-4, ptst=0.1):

        # idat, odat = torch tensor arrays


        # numpy
        idat, odat = [dat.numpy() for dat in [idat, odat]]

        # number of testing samples
        ntst = int(ptst*idat.shape[0])

        # training data
        itrn, otrn = [dat[ntst:] for dat in [idat, odat]]

        # testing data
        itst, otst = [dat[:ntst] for dat in [idat, odat]]

        # delete original data
        del idat; del odat

        # batch data
        itrn, otrn, itst, otst = [np.array_split(dat, batches) for dat in [itrn, otrn, itst, otst]]

        # tensor
        itrn, otrn, itst, otst = [[torch.from_numpy(d) for d in dat] for dat in [itrn, otrn, itst, otst]]

        # variables
        itrn, otrn, itst, otst = [[torch.autograd.Variable(d) for d in dat] for dat in [itrn, otrn, itst, otst]]

        # optimiser
        opt = torch.optim.SGD(self.parameters(), lr=lr, momentum=0.9)
        #opt = torch.optim.Adadelta(self.parameters(), lr=lr)

        # loss function
        lf = torch.nn.MSELoss()

        # for each episode
        for t in range(epo):

            # average episode loss
            ltrne, ltste = list(), list()

            # zero gradients
            opt.zero_grad()

            # for each batch
            for itrnb, otrnb, itstb, otstb in zip(itrn, otrn, itst, otst):

                # training loss
                ltrn = lf(self.forward(itrnb), otrnb)

                # testing loss
                ltst = lf(self.forward(itstb), otstb)

                # record loss
                ltrne.append(float(ltrn.data[0]))
                ltste.append(float(ltst.data[0]))

                # progress
                print(self.name, t, ltrn.data[0], ltst.data[0])

                # backpropagate training error
                ltrn.backward()

            # update weights
            opt.step()

            self.ltrn.append(np.average(ltrne))
            self.ltst.append(np.average(ltste))

        return self
