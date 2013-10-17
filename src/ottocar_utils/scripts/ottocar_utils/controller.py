class P_Controller(object):
    """docstring for P_Controller"""
    def __init__(self,kp=0.5):
        super(P_Controller, self).__init__()

        self.kp = kp
        self.target = 0
        self.input = 0
        self.output = 0

    def update(self, new_input = None):
        if not(new_input is None):
            self.input = new_input
            
        e = self.input - self.target
        self.output = self.input + self.kp * e

        return self.output