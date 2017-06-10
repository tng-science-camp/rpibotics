from rover.servo import Servo

LANCE_CONFIG = {
    'gpio_pin': 21,
    'duty_cycle_open': 3,
    'duty_cycle_closed': 12
}


class Lance(Servo):
    def __init__(self, config=LANCE_CONFIG):
        logging.info('Initializing a Lance.')
        Servo.__init__(self,
                       gpio_pin=config['gpio_pin'],
                       duty_cycle_0=config['duty_cycle_open'],
                       duty_cycle_180=config['duty_cycle_closed'])
        self.disarm()

    def arm(self):
        self.go_to_0()

    def disarm(self):
        self.go_to_180()
