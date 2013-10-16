
import rospy

class MeasureSampleRate(object):
    """docstring for MeasureSampleRate"""
    def __init__(self, update_interval, gain):
        super(MeasureSampleRate, self).__init__()
        self.sample_rate = 1
        self.gain = gain
        self.update_interval = update_interval #in number of samples
        self.first_samplerate = False
        self.n_samples = 0
        self.last_time = rospy.Time.now().to_sec()

    def add_sample(self):
        self.n_samples = (self.n_samples + 1) % self.update_interval
        if self.n_samples == 0:
            self.update_sample_rate(self.update_interval)

    def update_sample_rate(self, n_new_samples = 1):
        dtime = rospy.Time.now().to_sec() - self.last_time
        self.last_time = rospy.Time.now().to_sec()

        current_sample_rate = n_new_samples / dtime

        if self.first_samplerate:
            self.first_samplerate = False
            self.sample_rate = current_sample_rate

        self.sample_rate = self.sample_rate * (1-self.gain) + current_sample_rate * self.gain        

    def __complex__(self):
        return complex(self.sample_rate)
    def __int__(self):
        return int(self.sample_rate)
    def __long__(self):
        return long(self.sample_rate)
    def __float__(self):
        return float(self.sample_rate)

__all__ = ['MeasureSampleRate']