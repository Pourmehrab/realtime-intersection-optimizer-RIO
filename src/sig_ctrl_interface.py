#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright pending (c) 2017, Aschkan Omidvar <aschkan@ufl.edu>
# Created on Jan. 2017
# Updated on Feb. 2019
# University of Florida
# UF Transportation Institute
# Dept. of Civil and Coastal Engineering
# @author: aschkan

from multiprocessing import current_process
from pysnmp.hlapi import *
from src.config import get_sig_ctrl_interface_params

IP = '169.254.91.71'
PORT = 161

def snmpSet(OID, Value):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    global IP
    global PORT

    assert type(OID) == str
    errorIndication, errorStatus, errorIndex, varBinds = next(
        setCmd(SnmpEngine(),
               CommunityData('public', mpModel=0),  # snmp v1. delete mpModel for v2c),
               UdpTransportTarget((IP, PORT)), # Target IP + UPD port (hard coded)
               ContextData(),
               ObjectType(ObjectIdentity(str(OID)), Integer(Value))))

    if errorIndication:
        print(errorIndication)
    elif errorStatus:
        print('%s at %s' % (errorStatus.prettyPrint(),
                            errorIndex and varBinds[int(errorIndex) - 1][0] or '?'))
    #else:
    #    for varBind in varBinds:
    #        print(' = '.join([x.prettyPrint() for x in varBind]))

def snmpTranslate(List):
    """
    Example: 2^^3 phase translation breakdown:

    - Bit 7 = Ring number = (ringControlGroupNumber * 8)
    - Bit 6 = Ring number = (ringControlGroupNumber * 8) - 1
    - Bit 5 = Ring number = (ringControlGroupNumber * 8) - 2
    - Bit 4 = Ring number = (ringControlGroupNumber * 8) - 3
    - Bit 3 = Ring number = (ringControlGroupNumber * 8) - 4
    - Bit 2 = Ring number = (ringControlGroupNumber * 8) - 5
    - Bit 1 = Ring number = (ringControlGroupNumber * 8) - 6
    - Bit 0 = Ring number = (ringControlGroupNumber * 8) - 7

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        This module translates the phase numbers in a given list into snmp legible
        integers according to NTCIP 1202. The code encrypts the list of the phases
        into a binary string and then parses it to an snmp int value.

    """

    Power = max(List)
    Binary = ''
    for i in range(Power):
        if i + 1 in List:
            Binary += '1'
        else:
            Binary += '0'
    revB = Binary[::-1]
    snmpCode = int(revB, 2)
    return snmpCode


def snmpOmit(List):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.2.1 to the corresponding phase number and
        omit it. Hold is a command that causes omission of a selected phase.
    """
    assert type(List) == list  #
    OmitTemp = sorted(List)
    if List == [0]:
        snmpTerminate()
    else:
        snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', snmpTranslate(OmitTemp))


def snmpHold(List):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.4.1 to the corresponding phase number and
        hold it. Hold is a command that retains the existing Green interval.
    """
    assert type(List) == list  #
    HoldTemp = sorted(List)
    if List == [0]:
        snmpTerminate()
    else:
        snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', snmpTranslate(HoldTemp))


def snmpForceOff(List):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.5.1 to the corresponding phase number and
        Force Off it. Force off is A command to force the termination of the green
        interval in the actuated mode or Walk Hold in the nonactuated mode of the
        associated phase. Termination is subject to the presence of a serviceable
        conflicting call. The Force Off function shall not be effective during the
        timing of the Initial,Walk, or Pedestrian Clearance. The Force Off shall
        only be effective as long as the condition is sustained. If a phase
        specific Force Off is applied, the Force Off shall not prevent the start
        of green for that phase
    """
    assert type(List) == list
    ForceOffTemp = sorted(List)
    if List == [0]:
        snmpTerminate()
    else:
        snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', snmpTranslate(ForceOffTemp))


def snmpVehCall(List):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.6.1 to the corresponding phase number and
        call a vehicle on it.
    """
    assert type(List) == list
    VehCallTemp = sorted(List)
    if List == [0]:
        snmpTerminate()
    else:
        snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', snmpTranslate(VehCallTemp))


def snmpTerminate():
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        This module terminates all the commands and resets the signal controller to
        the default mode (actauted)
    """
    snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', 0)
    snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', 0)
    snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', 0)
    snmpSet('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', 0)


def snmp_phase_ctrl(Phase, inter_name):
    """
    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    .. note::
        Send command to ASC
    """
    num_phase, al, non, nonConflict = get_sig_ctrl_interface_params(inter_name)
    # For debugging
    # num_phase = 4
    # al = range(1, num_phase + 1)
    # non = [0]
    # nonConflict = [[1,2], [4, 3]]

    snmpHold(list(al))
    snmpHold(list(non))

    for p in range(len(nonConflict)):
        if Phase in nonConflict[p]:
            snmpVehCall(nonConflict[p])
            snmpOmit([i for i in al if i not in nonConflict[p]])

def main(intersection, ip_, port_, parent_proc_conn, proc_conn):
    """
    Main function for real-time communication with the signal
    controller.

    Communication between main RIO process and this process is handled
    with Pipes (see https://docs.python.org/3.6/library/multiprocessing.html#pipes-and-queues).

    Communication between this process and the signal controller
    is handled by the SNMP lib.
    """
    # set SNMP conn info
    global IP
    global PORT
    IP = ip_
    PORT = port_

    # prepare Pipes
    parent_proc_conn.close()
    p = current_process()
    print("Signal controller interface running on proc {}".format(p.pid))

    # Loop continuously until KeyboardInterrupt,
    # reading incoming msgs from Pipe conn
    try:
        while True:
            cmd, data = proc_conn.recv()
            if cmd == "SPaT":
                snmp_phase_ctrl(data, intersection)
            elif cmd == "done":
                proc_conn.close()
                break
    except KeyboardInterrupt:
        print("Signal controller interface got KeyboardInterrupt, exiting")


# Quickstart Test
#snmp_phase_ctrl(4, "RTS")
