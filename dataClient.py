from threading import Thread
import socket, time

def debug(text):
    if VERBOSE:
        print("Debug:---", text)

class Receiver(Thread):
    def run(self):
        debug("Receiver thread started")
        while True:
            try:
                rxData = self.readServerData()
                #debug("rxData = {}".format(rxData))
                if (rxData == 'quit'):
                    break
            except:
                debug("Exception in Receiver.run()")
                isReceiverRunning = False
                closeConnection()
                break
        
        debug("Receiver thread terminated")
        isReceiverRunning = False
        closeConnection()

    def readServerData(self):
        debug("readServerData: ")
        bufSize = 4096

        try:
            blk = sock.recv(bufSize).decode()
            if blk != None:
                debug("Received data block from server, len: " + str(len(blk)))
            else:
                debug("sock.recv() returned with None")
        except:
            raise Exception("Exception from blocking sock.recv()")
        print("Data received: '{}'".format(blk))
        return blk

def startReceiver():
    debug("Starting Receiver thread")
    receiver = Receiver()
    receiver.start()

def sendCommand(cmd):
    debug("sendCommand() with cmd = " + cmd)
    try:
        sock.sendall(cmd.encode())
    except:
        debug("Exception in sendCommand()")
        closeConnection()

def closeConnection():
    global isConnected
    debug("Closing socket")
    sock.close()
    isConnected = False

def connect():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    debug("Connecting...")
    try:
        sock.connect((IP_ADDRESS, IP_PORT))
    except:
        debug("Connection failed.")
        return False
    startReceiver()
    return True

def main():
    #debug
    global VERBOSE
    VERBOSE = True

    global done
    done = False

    #IP of server attempting to connect to
    global IP_ADDRESS
    IP_ADDRESS = "10.85.81.155"
    #IP_ADDRESS = "192.168.1.18"

    #arbitrary port, use one no one else does
    global IP_PORT
    IP_PORT = 32101

    sock = None
    global isConnected
    isConnected = False

    if connect():
        isConnected = True
        print("Connection established")
        time.sleep(1)
        while isConnected and not done:
            cmd = input("Enter Command: ")
            sendCommand(cmd)
    else:
        print("Connection to %s:%d failed" % (IP_ADDRESS, IP_PORT))

    print("Client exiting")

if __name__ == "__main__":
    main()
