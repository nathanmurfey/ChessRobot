import sys
import os
import platform
import time
import ctypes
from emotiv import Emotiv
import serial
import time

conn = serial.Serial("COM10", 9600)
print conn

emot = Emotiv()
counter = 0

print "---------------------------------------"
print "---------BRAINS OF THE OPERATION-------"
print "---------------------------------------"

letter = 'a'
number = '0'
action = 'M'

while True:
    if conn.is_open:
        message = conn.readline()
        print message

        if message[0:6] == "[REDY]":

            print "[REDY] Lets get going! - COMPUTER"
            print ""
            break
time.sleep(0)

start = raw_input("Ready to start?")

while True:

    print "[USER] Requesting Input"

    print "[USER] ? Move or Take ?"

    input = emot.is_yes()

    if input:
        action = 'M'
    else:
        action = 'T'

    print "[USER] ? A - D ? [Y/N]:"

    input = emot.is_yes()

    if input:
        print "[USER] ? A - B ? [Y/N]:"
        input = emot.is_yes()
        if input:
            print "[USER] ? A     ? [Y/N]:"
            input = emot.is_yes()
            if input:
                letter = 'A'
            else:
                letter = 'B'
        else:
            print "[USER] ? C     ? [Y/N]:"
            input = emot.is_yes()
            if input:
                letter = 'C'
            else:
                letter = 'D'
    else:
        print "[USER] ? E - F ? [Y/N]:"
        input = emot.is_yes()
        if input:
            print "[USER] ? E    ? [Y/N]:"
            input = emot.is_yes()
            if input:
                letter = 'E'
            else:
                letter = 'F'
        else:
            print "[USER] ? G    ? [Y/N]:"
            input = emot.is_yes()
            if input:
                letter = 'G'
            else:
                letter = 'H'

    print "[USER] ? 1 - 4 ? [Y/N]:"
    input = emot.is_yes()

    if input:
        print "[USER] ? 1 - 2 ? [Y/N]:"
        input = emot.is_yes()
        if input:
            print "[USER] ? 1     ? [Y/N]:"
            input = emot.is_yes()
            if input:
                number = '1'
            else:
                number = '2'
        else:
            print "[USER] ? 3     ? [Y/N]:"
            input = emot.is_yes()
            if input:
                number = '3'
            else:
                number = '4'
    else:
        print "[USER] ? 5 - 6 ? [Y/N]:"
        input = emot.is_yes()
        if input:
            print "[USER] ? 5    ? [Y/N]:"
            input = emot.is_yes()
            if input:
                number = '5'
            else:
                number = '6'
        else:
            print "[USER] ? 7    ? [Y/N]:"
            input = emot.is_yes()
            if input:
                number = '7'
            else:
                number = '8'

    command = letter + number + ":" + action
    print command




    while True:
        if conn.is_open:
            message =  conn.readline()
            print message

            if message[0:6] == "[RQST]":
                print "[OPEN] Recieved Communication PLACE REQUEST"
                conn.write("[COMM]! %s" %(command))
                while True:
                    if conn.is_open:
                        message = conn.readline()
                        print message
                        if message[0:6] == "[FCOM]":
                            break
                        time.sleep(2)
                        print "[INFO] Waiting ..."
                break
    time.sleep(0)





    if emot.is_yes():
        print "--------------------------------------------"



emot.terminate()