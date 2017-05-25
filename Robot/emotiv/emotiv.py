import sys
import os
import platform
import time
import ctypes

from ctypes import *

class Emotiv:
    def __init__(self):
        self.libEDK = cdll.LoadLibrary("C:\Users\Nathan\PycharmProjects\MindReader\edk.dll")
        self.write = sys.stdout.write
        self.IEE_EmoEngineEventCreate = self.libEDK.IEE_EmoEngineEventCreate
        self.IEE_EmoEngineEventCreate.restype = c_void_p
        self.eEvent = self.IEE_EmoEngineEventCreate()

        self.IEE_EmoEngineEventGetEmoState = self.libEDK.IEE_EmoEngineEventGetEmoState
        self.IEE_EmoEngineEventGetEmoState.argtypes = [c_void_p, c_void_p]
        self.IEE_EmoEngineEventGetEmoState.restype = c_int

        self.IS_GetTimeFromStart = self.libEDK.IS_GetTimeFromStart
        self.IS_GetTimeFromStart.argtypes = [ctypes.c_void_p]
        self.IS_GetTimeFromStart.restype = c_float

        self.IEE_EmoStateCreate = self.libEDK.IEE_EmoStateCreate
        self.IEE_EmoStateCreate.restype = c_void_p
        self.eState = self.IEE_EmoStateCreate()

        self.IS_GetWirelessSignalStatus = self.libEDK.IS_GetWirelessSignalStatus
        self.IS_GetWirelessSignalStatus.restype = c_int
        self.IS_GetWirelessSignalStatus.argtypes = [c_void_p]

        self.IS_MentalCommandGetCurrentAction = self.libEDK.IS_MentalCommandGetCurrentAction
        self.IS_MentalCommandGetCurrentAction.restype = c_int
        self.IS_MentalCommandGetCurrentAction.argtypes = [c_void_p]

        self.IS_MentalCommandGetCurrentActionPower = \
            self.libEDK.IS_MentalCommandGetCurrentActionPower
        self.IS_MentalCommandGetCurrentActionPower.restype = c_float
        self.IS_MentalCommandGetCurrentActionPower.argtypes = [c_void_p]

        self.userID = c_uint(0)
        self.user = pointer(self.userID)
        self.ready = 0
        self.state = c_int(0)
        self.systemUpTime = c_float(0.0)

        self.batteryLevel = c_long(0)
        self.batteryLevelP = pointer(self.batteryLevel)
        self.maxBatteryLevel = c_int(0)
        self.maxBatteryLevelP = pointer(self.maxBatteryLevel)

        self.systemUpTime = c_float(0.0)
        self.wirelessStrength = c_int(0)

        if self.libEDK.IEE_EngineConnect("Emotiv Systems-5") != 0:
            print "[EMOT] Emotiv Engine start up failed."
        print "[EMOT] Engine Success"
        self.composerPort = 3008
        if self.libEDK.IEE_EngineRemoteConnect("127.0.0.1", self.composerPort) != 0:
            print "[EMOT] Cannot connect to EmoComposer on"
        print "[EMOT] Connect to Composer success"

        print "[INFO] EMOTIVE successfully connected - check the eeg signal qualities"

    def get_states(self, userID , eState):
        current_action = self.IS_MentalCommandGetCurrentAction(eState)
        current_action_power = self.IS_MentalCommandGetCurrentActionPower(eState)
        #print "[ELOG] GCA", current_action, "GCAP", current_action_power
        return current_action, current_action_power

    def run_emotive(self):
        while (1):
            state = self.libEDK.IEE_EngineGetNextEvent(self.eEvent)
            if state == 0:
                eventType = self.libEDK.IEE_EmoEngineEventGetType(self.eEvent)
                self.libEDK.IEE_EmoEngineEventGetUserId(self.eEvent, self.user)
                if eventType == 64:  # libEDK.IEE_Event_enum.IEE_EmoStateUpdated
                    self.libEDK.IEE_EmoEngineEventGetEmoState(self.eEvent, self.eState)
                    timestamp = self.IS_GetTimeFromStart(self.eState)
                    state, power = self.get_states(self.userID, self.eState)
                    print state, power
                    return state, power
            elif state != 0x0600:
                print "Internal error in Emotiv Engine ! "

    def is_yes(self):
        counter = 0
        start = time.time()
        while True:
            if (time.time() - start) > 5 and counter > 5:
                print "------------- YOU CHOSE YES!"
                return True
            elif (time.time() - start) > 5 and counter < 5:
                print "------------- YOU CHOSE NO!"
                return False
            else:
                a, p = self.run_emotive()
                if a == 2:
                    counter = counter + 1
            #print round(time.time() - start), "--" , counter


    def terminate(self):
        self.libEDK.IEE_EngineDisconnect()
        self.libEDK.IEE_EmoStateFree(self.eState)
        self.libEDK.IEE_EmoEngineEventFree(self.eEvent)