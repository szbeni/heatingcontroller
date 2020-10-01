import schedule
import time



class HeatingControllerScheduler:
    def __init__(self, callback):
        self.callback = callback

    def weekday(self, time, temp, intensity):
        schedule.every().monday.at(time).do(self.callback,temp,intensity)
        schedule.every().tuesday.at(time).do(self.callback,temp,intensity)
        schedule.every().wednesday.at(time).do(self.callback,temp,intensity)
        schedule.every().thursday.at(time).do(self.callback,temp,intensity)
        schedule.every().friday.at(time).do(self.callback,temp,intensity)

    def weekend(self, time, temp, intensity):
        schedule.every().saturday.at(time).do(self.callback,temp,intensity)
        schedule.every().sunday.at(time).do(self.callback,temp,intensity)

    def everyday(self, time, temp, intensity):
        schedule.every().day.at(time).do(self.callback,temp,intensity)

    def init(self):
        self.weekday("06:45", 19, "low")
        self.weekday("07:45", 13, "high")

        self.weekend("09:00", 19, "low")
        self.weekend("09:30", 19, "high")
        self.everyday("23:00", 15, "low")

    def start(self):
        self.init()
        self.running = True

    def stop(self):
        schedule.clear()
        self.running = False
    
    def tick(self):
        if self.running:
            schedule.run_pending()

if __name__ == "__main__":
    def callback(temp=None,intensity=None,mode=None):
        if temp:
            print(temp)
        if intensity:
            print(intensity)
        if mode:
            print(mode)


    s = HeatingControllerScheduler(callback)
    s.start()
    s.stop()
    s.start()
    while(True):
        s.tick()
        time.sleep(1)
        print(schedule.jobs)

