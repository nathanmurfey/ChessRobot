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

#emot = Emotiv()
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

    input = raw_input( "[USER] ? Move or Take ?")

    if input == "Y":
        action = 'M'
    else:
        action = 'T'

    #print "[USER] ? A - D ? [Y/N]:"
    input = raw_input( "[USER] ? A - D ? [Y/N]:")
    #input = emot.is_yes()

    if input == "Y":
        #print "[USER] ? A - B ? [Y/N]:"
        input = raw_input( "[USER] ? A - B ? [Y/N]:")
        #input = emot.is_yes()
        if input == "Y":
            #print "[USER] ? A     ? [Y/N]:"
            input = raw_input( "[USER] ? A     ? [Y/N]:")
            if input == "Y":
                letter = 'A'
            else:
                letter = 'B'
        else:
            #print "[USER] ? C     ? [Y/N]:"
            input = raw_input( "[USER] ? C     ? [Y/N]:")
            if input == "Y":
                letter = 'C'
            else:
                letter = 'D'
    else:
        #print "[USER] ? E - F ? [Y/N]:"
        input = raw_input("[USER] ? E - F ? [Y/N]:")
        if input == "Y":
            #print "[USER] ? E    ? [Y/N]:"
            input = raw_input("[USER] ? E    ? [Y/N]:")
            if input == "Y":
                letter = 'E'
            else:
                letter = 'F'
        else:
            #Dprint "[USER] ? G    ? [Y/N]:"
            input = raw_input("[USER] ? G    ? [Y/N]:")
            if input == "Y":
                letter = 'G'
            else:
                letter = 'H'

   # print "[USER] ? 1 - 4 ? [Y/N]:"
    input = raw_input("[USER] ? 1 - 4 ? [Y/N]:")

    if input == "Y":
        #print "[USER] ? 1 - 2 ? [Y/N]:"
        input = raw_input("[USER] ? 1 - 2 ? [Y/N]:")
        if input == "Y":
            #print "[USER] ? 1     ? [Y/N]:"
            input = raw_input("[USER] ? 1     ? [Y/N]:")
            if input == "Y":
                number = '1'
            else:
                number = '2'
        else:
            #print "[USER] ? 3     ? [Y/N]:"
            input = raw_input("[USER] ? 3     ? [Y/N]:")
            if input == "Y":
                number = '3'
            else:
                number = '4'
    else:
        #print "[USER] ? 5 - 6 ? [Y/N]:"
        input = raw_input("[USER] ? 5 - 6 ? [Y/N]:")
        if input == "Y":
            #print "[USER] ? 5    ? [Y/N]:"
            input = raw_input("[USER] ? 5    ? [Y/N]:")
            if input == "Y":
                number = '5'
            else:
                number = '6'
        else:
            #print "[USER] ? 7    ? [Y/N]:"
            input = raw_input("[USER] ? 5    ? [Y/N]:")
            if input == "Y":
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
                conn.write("[COMM]!_%s" %(command))
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





    #if emot.is_yes():
    #    print "--------------------------------------------"



emot.terminate()