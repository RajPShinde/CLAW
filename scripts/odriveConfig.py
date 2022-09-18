#!/usr/bin/env python3.8
import sys
import odrive
from odrive.enums import *
import time
import math
from odrive.utils import OperationAbortedException

def getOdrive():
    print('Looking for ODrive...')
    odrv = odrive.find_any()
    print('Found.')
    return odrv

def resetOdrive(odrv):
    print('Erasing config and rebooting...')
    odrv.erase_configuration()
    reboot_odrive(odrv)

def runState(axis, requested_state, wait):
    axis.requested_state = requested_state
    timeout_ctr = 100;
    if wait:
        while(timeout_ctr > 0):
            time.sleep(0.2)
            timeout_ctr -= 1
            if axis.current_state == AXIS_STATE_IDLE:
                break
    return timeout_ctr > 0

def calibrateMotor(odrv, axis):
    time.sleep(0.5)
    print('Calibrating Motor',end='',flush=True)
    runState(axis, AXIS_STATE_MOTOR_CALIBRATION, True)
    print('Done');

def calibrateEncoder(odrv, axis):
    time.sleep(0.5)
    print('Calibrating Encoder',end='',flush=True)
    runState(axis, AXIS_STATE_ENCODER_INDEX_SEARCH, True)
    runState(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, True)
    print('Done');


def main():

    print("Enter the Odrive Number [Follow ther Odrive Connection Diagram]")
    odriveNo = int(input())

    odrv0 = getOdrive()

    odrv0.can.config.baud_rate = 250000

    print('Configuring Axis 0');
    odrv0.axis0.config.can.node_id = (odriveNo -1)*2
    odrv0.axis0.motor.config.pole_pairs=20
    odrv0.axis0.encoder.config.cpr = 2000
    odrv0.axis0.motor.config.torque_constant=0.091888889

    odrv0.axis0.motor.config.current_lim = 30
    odrv0.axis0.controller.config.vel_limit=10

    calibrateMotor(odrv0, odrv0.axis0)
    odrv0.axis0.motor.config.pre_calibrated =True

    odrv0.axis0.encoder.config.use_index =True
    calibrateEncoder(odrv0, odrv0.axis0)
    odrv0.axis0.encoder.config.pre_calibrated =True
    odrv0.axis0.config.startup_encoder_index_search = True
    odrv0.axis0.config.startup_encoder_offset_calibration = False

    odrv0.axis0.config.startup_closed_loop_control = True

    print('Configuring Axis 1');
    odrv0.axis1.config.can.node_id = (odriveNo -1)*2 + 1
    odrv0.axis1.motor.config.pole_pairs=20
    odrv0.axis1.encoder.config.cpr = 2000
    odrv0.axis1.motor.config.torque_constant=0.091888889

    odrv0.axis1.motor.config.current_lim = 30
    odrv0.axis1.controller.config.vel_limit=10

    calibrateMotor(odrv0, odrv0.axis1)
    odrv0.axis1.motor.config.pre_calibrated =True

    odrv0.axis1.encoder.config.use_index =True
    calibrateEncoder(odrv0, odrv0.axis1)
    odrv0.axis1.encoder.config.pre_calibrated =True
    odrv0.axis1.config.startup_encoder_index_search = True
    odrv0.axis1.config.startup_encoder_offset_calibration = False

    odrv0.axis1.config.startup_closed_loop_control = True

    odrv0.save_configuration()


main()

