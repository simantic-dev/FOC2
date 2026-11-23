# The ESC1's user button (PC10, external pull-up): held released so FOC2's main loop does not
# re-run calibration.
class Slave:
    def __init__(self, ctx):
        ctx.gpio(0).set(True)

    def on_gpio(self, line, level):
        pass
