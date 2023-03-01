import time

class MyTimer:
    def __init__(self, interval, callback=None):
        self.interval = interval
        self.callback = callback
        self.t = time.time() + self.interval
    
    def update(self):
        if (time.time() > self.t):
            while self.t < time.time():
                self.t += self.interval
            if self.callback is not None:
                self.callback()
            return True
        return False

if __name__ == '__main__':
    def dummy_cb():
        print('callback!')
    timer = MyTimer(1, dummy_cb)
    while True:
        if timer.update():
            print('update: {}'.format(time.time()))
