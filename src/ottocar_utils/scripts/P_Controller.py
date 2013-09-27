class P_Controller(object):
    """docstring for P_Controller"""
    def __init__(self,kp):
        super(P_Controller, self).__init__()

        self.kp = kp
        self.target = 0
        self.input = 0
        self.output = 0

    def update(self):
        e = self.input - self.target
        self.output = self.input + self.kp * e