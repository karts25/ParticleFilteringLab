"""
Reads sensor data from a log and sends it back on request
"""

class LogStreamer:
    def __init__(self,logName):
        self.fp = open("../data/log/"+logName,"r")
        if not self.fp:
            print "Error: Could not open",mapName
            return -1
        print "Reading log",logName

    def getNext(self):
        l = self.fp.readline()
        if not l:
            return None
        readings = l.rsplit(" ")
        readings[1:] = [float(reading) for reading in readings[1:]]
        return readings

if __name__ == "__main__":
    myLogStream = LogStreamer("robotdata1.log")
    myLogStream.getNext()
    print "\n\n\n"
    myLogStream.getNext()
