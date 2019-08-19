from NatNetClient       import NatNetClient
from EStop              import EStop
import time
import serial
from threading          import Thread
from threading          import Event

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    pass
    
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(frameID, pos, orient, trackingValid): #NOTE: assign 4 markers to the leader(#1), 5 to copter number 2, 6 to copter number 3, and so on!
    global positions, orientations, trackingFlags, numCopters
    global payloadPose
    if (frameID == 3):
        payloadPose = [pos[0], -pos[2], pos[1], orient[0], orient[1], orient[2], orient[3]]
    else:
        index = frameID - 4
        tempPos = [pos[0], -pos[2], pos[1]] #To transform the camera frame to the inertial frame
        positions[index] = tempPos
        orientations[index] = orient
        trackingFlags[index] = trackingValid
        optitrackThread.callCounter += 1
        if (optitrackThread.callCounter % numCopters == 0):
            event.set()
def stopButtonListener():
    global EStop_failsafe
    global stopBtnPressed
    while True:
        time.sleep(0.05)
        EStop_failsafe.updateArmingState()
        if(EStop_failsafe.armingState == ord('0')):
            stopBtnPressed = True
        else:
            stopBtnPressed = False
def serialSender():
    global positions, orientations, stopBtnPressed
    #Must send the stop button info as well as the position and orientation information
    serialLink = serial.Serial(             #Set up connection from Flight controller to linux machine
        port="COM12",
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout = 0
    )
    if(serialLink.isOpen()):
        print("Serial port opened successfully")
    else:
        print("Serial port not opened")

    #Writes data to serial port as ascii characters of 8 bits with UTF-8 encoding
        #Use "mystring".encode('utf-8')
    #serial.write returns the nummber of bytes written

    #Serial communication structure:
        #Data is sent as x, y, z, quart(x), quart(y), quart(z), quart(w), stopButtonVal
        #Each digit of each entry is sent as a character with commas separating entries
        #A "|" character will denote the end of a full message
    #Timing:
        #5 characters per x, y, z
            #18 total (if negative)
        #5 characters per quart index
            #24 total (if negative)
        #1 stop button character
        #1 end transmission character
        #7 commas
        # 10 bit packets sent at 115200baud
        #115200/510 = 225 messages per second => time per 1000 messages test verifies this
        #Will meet 120Hz requirement
    sendCounter = 0
    while True:
        posX = str("{:.3f}".format(positions[0][0]))
        posY = str("{:.3f}".format(positions[0][1]))
        posZ = str("{:.3f}".format(positions[0][2]))

        quartW = str("{:.3f}".format(orientations[0][0]))
        quartX = str("{:.3f}".format(orientations[0][1]))
        quartY = str("{:.3f}".format(orientations[0][2]))
        quartZ = str("{:.3f}".format(orientations[0][3]))

        stopButtonVal =  "1"
        if(stopBtnPressed):
            stopButtonVal = "0"

        totalString = posX+","+posY+","+posZ+","+quartW+","+quartX+","+quartY+","+quartZ+","+stopButtonVal+"\n"
        print(totalString)
        serialLink.write(totalString.encode('utf-8'))
        sendCounter = sendCounter+1
        if(sendCounter%1000 == 0):
            print("Sent: " + str(sendCounter) + " messages")

    serialLink.close()
    
if (__name__=='__main__'):
    numCopters = 1
    positions = []
    orientations = []
    trackingFlags = []
    stopBtnPressed = False
    for i in range (numCopters):
        positions.append([])
        orientations.append([])
        trackingFlags.append(False)
    event = Event() # Event object to sync the main thread and the optitrack thread

    #To run in the optitrackThread
    optitrackThread                   = NatNetClient()        # This will create a new NatNet client to connect to motive
    optitrackThread.newFrameListener  = receiveNewFrame       # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    optitrackThread.rigidBodyListener = receiveRigidBodyFrame
    
    EStop_failsafe = EStop('COM4', 115200)

    optitrackThread.run()                       #Start up the streaming client now that the callbacks are set up. This will run perpetually, and operate on a separate thread.
    print("Comunication with cameras established. (Thread #1)")

    time.sleep(3)
    
    stopThread = Thread(target = stopButtonListener)  #Thread to communicate with the copters. (Send commands only)
    stopThread.start()                           #Start up thread to constantly send rx data
    print("Stop button enabled. (Thread #2)")    

    time.sleep(3)

    communicationThread = Thread(target = serialSender)
    communicationThread.start()
    print("Serial link established (Thread #3)")
