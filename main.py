# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#
# qube.setRGB(r, g, b) - Sets the LED color of the QUBE. Color values range from [0, 999].
# qube.setMotorSpeed(speed) - Sets the motor speed. Speed ranges from [-999, 999].
# qube.setMotorVoltage(volts) - Applies the given voltage to the motor. Volts range from (-24, 24).
# qube.resetMotorEncoder() - Resets the motor encoder in the current position.
# qube.resetPendulumEncoder() - Resets the pendulum encoder in the current position.
import numpy as np

# qube.getMotorPosition() - Returns the cumulative angular positon of the motor.
# qube.getPendulumPosition() - Returns the cumulative angular position of the pendulum.
# qube.getMotorRPM() - Returns the newest rpm reading of the motor.
# qube.getMotorCurrent() - Returns the newest reading of the motor's current.
# ------------------------------------- AVAILABLE FUNCTIONS --------------------------------#

from QUBE import *
from logger import *
from com import *
from liveplot import *
from time import time
import threading

# Replace with the Arduino port. Can be found in the Arduino IDE (Tools -> Port:)
port = "COM8"
baudrate = 115200
qube = QUBE(port, baudrate)

# Resets the encoders in their current position.
qube.resetMotorEncoder()
qube.resetPendulumEncoder()

# Enables logging - comment out to remove
enableLogging()

t_last = time()

m_target = 0
p_target = 0
pid = PID()
y = 0
x = 0
dt = 0.05
X = 0
i = 0


def control(data, lock):
    global m_target, p_target, pid, dt, y, x, i

    while True:
        # Updates the qube - Sends and receives data
        qube.update()
        m_target = 0

        # Gets the logdata and writes it to the log file
        logdata = qube.getLogData(m_target, p_target)
        save_data(logdata)
        pid = PID()

        y=PID.regulate(pid,QUBE.getMotorAngle(qube),QUBE.getMotorRPM(qube),45)
        print(y)



        qube.setMotorVoltage(y)
        # Multithreading stuff that must happen. Don't mind it.
        with lock:
            doMTStuff(data)

        # Get deltatim
        dt = getDT()
        ### Your code goes here


def getDT():
    global t_last
    t_now = time()
    dt = t_now - t_last
    t_last += dt
    return dt


def doMTStuff(data):
    packet = data[7]
    pid.copy(packet.pid)
    if packet.resetEncoders:
        qube.resetMotorEncoder()
        qube.resetPendulumEncoder()
        packet.resetEncoders = False

    new_data = qube.getPlotData(m_target, p_target)
    for i, item in enumerate(new_data):
        data[i].append(item)


if __name__ == "__main__":
    _data = [[], [], [], [], [], [], [], Packet()]
    lock = threading.Lock()
    thread1 = threading.Thread(target=startPlot, args=(_data, lock))
    thread2 = threading.Thread(target=control, args=(_data, lock))
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
