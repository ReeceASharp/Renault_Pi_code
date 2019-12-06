from threading import Thread, Event
import socket
import time
import RPi.GPIO as GPIO
from math import pi

VERBOSE = True
IP_PORT = 32101


def debug(text):
    if VERBOSE:
        print("Debug:---", text)

class SocketHandler(Thread):
    def __init__(self, conn):
        Thread.__init__(self)
        self.conn = conn
        self.right_rotations = 0
        self.left_rotations = 0
        self.right_duty = 0
        self.left_duty = 0
        self.setup()


    def setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(18, GPIO.OUT) #pwm0
        GPIO.setup(13, GPIO.OUT) #pwm1

        GPIO.setup(6, GPIO.OUT)  #motor clk     YELLOW
        GPIO.setup(16, GPIO.OUT) #motor enable  GREEN
        GPIO.setup(19, GPIO.OUT) #motor data    BLUE
        GPIO.setup(26, GPIO.OUT) #motor latch   PURPLE
        GPIO.setup(23, GPIO.IN)  #Encoder left
        GPIO.setup(24, GPIO.IN)  #Encoder right

        #global    # Initialize PWM0 on pwmPin 12 55Hz frequency
        self.right_pwm = GPIO.PWM(18, 55)
        GPIO.output(18, GPIO.LOW)

        #global    # Initialize PWM1 on pwmPin 33 55Hz frequency
        self.left_pwm = GPIO.PWM(13, 55)
        GPIO.output(13, GPIO.LOW)

        # latchreset pwm values, and start the engines off with no power
        # the benefit here is that the power output can be dynamically
        # changed to help direct the robot
        self.latch(0)
        self.right_pwm.start(self.right_duty)
        self.left_pwm.start(self.left_duty)


    def run(self):
        global isConnected
        debug("SocketHandler started")
        while True:
            debug("Running")
            cmd = ""
            try:
                debug("Calling blocking conn.recv()")
                cmd = self.conn.recv(1024).decode()
            except:
                debug("exception in conn.recv()")
                # happens when connection is reset from the peer
                break
            debug("Received cmd: "+ cmd + " len: " + str(len(cmd)))
            if len(cmd) == 0:
                break
            self.executeCommand(cmd)
        conn.close()
        print("Client disconnected. Waiting for next client...")
        isConnected = False
        debug("SocketHandler terminated")

    def executeCommand(self, cmd):
        debug("executeCommand(): cmd: {}".format(cmd))
        value = ""
        valid = True

        commands = cmd.split()
        if len(commands) != 2:
            valid = False

            #return command will stop client
            if (len(commands) == 1 and commands[0] == 'quit'):
                value = 'quit'
                self.conn.sendall(value.encode())
                return
            else:
                value += "INVALID # OF COMMANDS: 'direction distance' i.e. 'forward 2'\n"
        else:
            debug("{} - {}".format(commands[0], commands[1]))

        #give the motors power, start at full power for each
        self.right_duty = 100
        self.left_duty = 100
        self.right_pwm.ChangeDutyCycle(self.right_duty)
        self.left_pwm.ChangeDutyCycle(self.left_duty)

        #decide which motors should move
        if not self.setLatch(commands[0]):
            valid = False
            if (commands[0] != 'quit'):
                value += "INVALID Direction: 'forward', 'back', 'left', 'right'\n"

        #if valid code, begin execution
        if valid:
            #all values are in inches
            wheel_diameter = 2.75
            wheel_circum = wheel_diameter * pi
            radius = 5                                  #radius from center of car to middle of wheel
            wheel_base_circum = ((radius * 2) * pi)

            #used for forward, back, converting 1 -> 12 inches
            wheel_rotations = (12 / wheel_circum) * float(commands[1])
            #used for left, right, converting 1 -> 360 degrees
            full_spin_rotation = wheel_base_circum / wheel_circum

            max_difference = 0
            current_difference = 0
            #value to be set depending on either turning, or moving
            duration = 0

            if (commands[0] == 'left' or commands[0] == 'right'):
                duration = full_spin_rotation * (float(commands[1]) / 360)
            elif (commands[0] == 'forward' or commands[0] == 'back'):
                duration = wheel_rotations

            #tenth of a millisecond, needed to accurately measure encoder changes
            sleep_time = 1 / (1000)
            left_count = 0
            left_current = 0
            self.right_rotations = 0

            right_count = 0
            right_current = 0
            self.left_rotations = 0
            i = 0


            #create outside timer that can periodically update the
            flag = Event()
            thread = self.TimeThread(flag, self)
            thread.start()

            debug("Duration: " + str(duration))
            #while wheels still have to rotate, or the base timeout is hit
            while (self.right_rotations < duration and self.left_rotations < duration):
                #current value of encoder (1 = spoke, 0 = space)
                left_encode = GPIO.input(23)
                right_encode = GPIO.input(24)

                #we care about the numbre of value changes, not what the value is
                if left_current != left_encode:
                    left_count += 1
                    left_current = left_encode
                if right_current != right_encode:
                    right_count += 1
                    right_current = right_encode

                if (i % 100 == 0):
                    debug("Running: " + str(i))


                current_difference = abs(left_count - right_count)
                if abs(left_count - right_count) > max_difference:
                    max_difference = current_difference


                #these values give the current # of rotations of the wheels, this is specific
                #to the encoder/wheel used.
                self.right_rotations = left_count / 180
                self.left_rotations = right_count / 180

                i += 1
                if (i > 10000):
                    break
                #let the encoders rest before reading them again
                time.sleep(sleep_time)

            debug("Finished movement")
            #stop sending updates to the client
            flag.set()

            #execute reverse of command to stop momentum
            self.brake(commands[0])
            value = "Finished Command, it ran successfully with a max encoder difference of {:.4}%".format(float(max_difference * 100 / 180))
            #print("R_r: {}, L_r: {}, L_c: {} R_c{}".format(self.right_rotations, self.left_rotations, left_count, right_count))
        

        #turn power off to motors
        self.right_pwm.ChangeDutyCycle(0)
        self.left_pwm.ChangeDutyCycle(0)
        #turn signals off of motors
        self.latch(0)
        time.sleep(1)
        print("Max_diff: {}".format(max_difference))
        #send value back to client
        
        self.conn.sendall(value.encode())

    def brake(self, direction):
        debug("STOPPING")
        #get direction send its reverse to setup the direction of the motors
        if direction == 'forward':
            self.setLatch('back')
        elif direction == 'back':
            self.setLatch('forward')
        elif direction == 'left':
            self.setLatch('right')
        elif direction == 'right':
            self.setLatch('left')
        else:
            self.latch(0)
        #hard run for a small increment (simulates brakes)
        self.right_pwm.ChangeDutyCycle(100)
        self.left_pwm.ChangeDutyCycle(100)
        time.sleep(.1)


    def setLatch(self, direction):
        #pull proper direction commands
        d = directions()

        if direction == 'forward':
            self.latch(d.fwd_fwd)
        elif direction == 'back':
            self.latch(d.bck_bck)
        elif direction == 'left':
            self.latch(d.bck_fwd)
        elif direction == 'right':
            self.latch(d.fwd_bck)
        else:
            return False
        return True


    def latch(self, latch_code):
        #sets output to motots to nothing
        GPIO.output(26, GPIO.LOW)
        GPIO.output(19, GPIO.LOW)

        #move through bits in the byte sent via latch_code, and set
        #them to high as neccesary
        for i in range(8):
            time.sleep(.000001)
            GPIO.output(6, GPIO.LOW)

            if latch_code & 1 << (i):
                GPIO.output(19, GPIO.HIGH)
            else:
                GPIO.output(19, GPIO.LOW)

            time.sleep(.000001)
            GPIO.output(6, GPIO.HIGH)

        GPIO.output(26, GPIO.HIGH)

    def updateSpeeds(self):
        #stops a divide by 0
        if self.right_rotations == 0:
            print ("R = 0")
            return
        if self.left_rotations == 0:
            print("L = 0")
            return

        #gives a ratio of current work done by the right : left
        ratio = self.right_rotations / self.left_rotations

        # right is faster, decrement it's power, or increment the other power if it's below a certain threshhold
        # this allows for a balancing of power adjusts vs. just constantly decrementing
        if ratio > 1:
            print("L < R - {} vs {}, {} - {}, {}".format(self.left_rotations, self.right_rotations, self.left_duty, self.right_duty, ratio))
            if ratio > 1.005:
                if(self.right_duty > 80): 
                    self.right_duty -= 10
                    self.right_pwm.ChangeDutyCycle(self.right_duty)
                elif self.left_duty <= 90:
                    self.left_duty += 10
                    self.left_pwm.ChangeDutyCycle(self.left_duty)
        #left is faster
        elif ratio < 1:
            print("L > R - {} vs {}, {} - {}, {}".format(self.left_rotations, self.right_rotations, self.left_duty, self.right_duty, ratio))
            if ratio < 0.995:
                if(self.left_duty > 80):
                    self.left_duty -= 10
                    self.left_pwm.ChangeDutyCycle(self.left_duty)
                elif self.right_duty <= 90:
                    self.right_duty += 10
                    self.right_pwm.ChangeDutyCycle(self.right_duty)
        else:
            print("L = R - {} vs {}, {} - {}, {}".format(self.left_rotations, self.right_rotations, self.left_duty, self.right_duty, ratio))

    #allows for periodic checking of the speeds, and updating them as necessary
    class TimeThread(Thread):
        def __init__(self, event, currentObj):
            Thread.__init__(self)
            self.stopped = event
            self.obj = currentObj
            self.timer = 0.01

        def run(self):
            while not self.stopped.wait(self.timer):
                self.obj.updateSpeeds()

class directions:
    #motors are able to move forward, or backwards
    two_fwd   = 0b00001000
    two_bck   = 0b01000000
    three_fwd = 0b00000001
    three_bck = 0b00000100

    # ^ above values or'd to get combination of two motors
    fwd_fwd   = 0b00001001    #forward
    bck_bck   = 0b01000100    #back
    bck_fwd   = 0b00001100    #left
    fwd_bck   = 0b01000001    #right


serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# close port when process exits:
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
debug("Socket created")
HOSTNAME = "" # Symbolic name meaning all available interfaces
try:
    serverSocket.bind((HOSTNAME, IP_PORT))
except socket.error as msg:
    print("Bind failed", msg[0], msg[1])
    sys.exit()
serverSocket.listen(10)

print("Waiting for a connecting client...")
isConnected = False
while True:
    debug("Calling blocking accept()...")
    conn, addr = serverSocket.accept()
    print("Connected with client at " + addr[0])
    isConnected = True
    socketHandler = SocketHandler(conn)
    # necessary to terminate it at program termination:
    socketHandler.setDaemon(True)
    socketHandler.start()
    t = 0
    while isConnected:
        print("Server connected at", t, "s")
        time.sleep(10)
        t += 10

    #socketHandler.shutdown()
    #socketHandler.close()
