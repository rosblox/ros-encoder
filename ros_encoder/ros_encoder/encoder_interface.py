import pigpio

class Encoder:
    def __init__(self, pi, pin_a, pin_b, scale=1):
        self.pi = pi
        self.scale = scale
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0

        self.pi.set_mode(self.pin_a, pigpio.INPUT)
        self.pi.set_mode(self.pin_b, pigpio.INPUT)
        self.pi.set_pull_up_down(self.pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_b, pigpio.PUD_UP)

        self.cb_a = self.pi.callback(self.pin_a, pigpio.EITHER_EDGE, self._pulse)
        self.cb_b = self.pi.callback(self.pin_b, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, pin, level, tick):
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        if pin == self.pin_a:
            if level == a:
                self.position += 1 if a == b else -1
        else:  # pin_b
            if level == b:
                self.position += 1 if a != b else -1

    def get_position(self):
        return self.position * self.scale

    def reset_position(self):
        self.position = 0
