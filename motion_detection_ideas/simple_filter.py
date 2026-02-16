from console_reader import MotionConsole

class Filter:
    def __init__(self):
        console = MotionConsole(rx_callback=self._process_line)
    

    def _process_line(self, line_full: str):
        pass
if __name__ == "__main__":
    filter = Filter()