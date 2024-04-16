import pyvisa as visa
from Config.colorbar import vmin


class device:
    def __init__(self):
        self.rm = visa.ResourceManager('@py')
        self.VISA_ADDRESS = self.rm.list_resources()[0]
        self.session = self.rm.open_resource(self.VISA_ADDRESS, baud_rate = 9600)

    def open(self):
        print('Setting angle sensor...')
        del self.session.timeout
        for _ in range(2):
            self.read_power()
        print('OK!!')
        return self

    def read_power(self):
        try:
            if self.session.read().strip() == "":
                result = float(-50)
            else:
                result = float(self.session.read().strip())

            return result

        except UnicodeDecodeError:
            result = float(-50)
            print('Error: Could not decode data.')
            
            return result
        
        except visa.Error or ValueError:
            return vmin

    def close(self):
        self.session.close()
        self.rm.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


if __name__ == '__main__':
    from Utils import timeit

    with device() as s:
        for i, _ in enumerate(range(500000)):
            with timeit(title='time= '):
                print(f'power= {s.read_power():.3f}, ', end='')
