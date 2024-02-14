import lgpio

class Encoder:
    def __init__(self, gpio_chip, pin_a, pin_b, scale=1):
        self.gpio_chip = gpio_chip
        self.scale = scale
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0

        lgpio.gpio_claim_input(self.gpio_chip, self.pin_a)
        lgpio.gpio_claim_input(self.gpio_chip, self.pin_b)

        self.last_state_a = lgpio.gpio_read(self.gpio_chip, self.pin_a)
        self.last_state_b = lgpio.gpio_read(self.gpio_chip, self.pin_b)

        lgpio.gpio_set_alert_func(self.gpio_chip, self.pin_a, self._pulse)
        lgpio.gpio_set_alert_func(self.gpio_chip, self.pin_b, self._pulse)

    def _pulse(self, gpio_chip, gpio, level, tick):
        if gpio == self.pin_a:
            new_state_a = level
            new_state_b = lgpio.gpio_read(self.gpio_chip, self.pin_b)
        else:
            new_state_a = lgpio.gpio_read(self.gpio_chip, self.pin_a)
            new_state_b = level
        if (new_state_a != self.last_state_a) or (new_state_b != self.last_state_b):
            if new_state_a != new_state_b:
                self.position += 1
            else:
                self.position -= 1
            self.last_state_a = new_state_a
            self.last_state_b = new_state_b

    def get_position(self):
        return self.position * self.scale

    def reset_position(self):
        self.position = 0
