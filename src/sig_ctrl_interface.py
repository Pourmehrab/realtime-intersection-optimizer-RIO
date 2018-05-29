#!/usr/bin/env python3

# Copyright pending (c) 2017, Aschkan Omidvar <aschkan@ufl.edu>
# Created on Jan. 2017
# Updated on May 2018
# University of Florida
# UF Transportation Institute
# Dept. of Civil and Coastal Engineering
# @author: aschkan

from pysnmp.hlapi import *
from data.data import get_sig_ctrl_interface_params


def snmp_set(OID, value):
    """
    todo: a line what it does


    :param OID:
    :param value:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(OID) == str
    errorIndication, errorStatus, errorIndex, varBinds = next(
        setCmd(SnmpEngine(),
               CommunityData('public', mpModel=0),
               # snmp v1. delete mpModel for v2c),
               UdpTransportTarget(('169.254.91.71', 161)), ContextData(),
               ObjectType(ObjectIdentity(str(OID)), Integer(value))))

    if errorIndication:
        print(errorIndication)
    elif errorStatus:
        print('%s at %s' % (errorStatus.prettyPrint(),
                            errorIndex and varBinds[int(errorIndex) - 1][0] or '?'))
    else:
        for varBind in varBinds:
            print(' = '.join([x.prettyPrint() for x in varBind]))


def snmp_translate(_list):
    """
    .. note::
        This module translates the phase numbers in a given list into snmp legible
        integers according to NTCIP 1202. The code encrypts the list of the phases
        into a binary string and then parses it to an snmp int value.

    Example
    -------
     2^^3 phase translation breakdown:

    * Bit 7 = Ring number = (ringControlGroupNumber * 8)
    * Bit 6 = Ring number = (ringControlGroupNumber * 8) - 1
    * Bit 5 = Ring number = (ringControlGroupNumber * 8) - 2
    * Bit 4 = Ring number = (ringControlGroupNumber * 8) - 3
    * Bit 3 = Ring number = (ringControlGroupNumber * 8) - 4
    * Bit 2 = Ring number = (ringControlGroupNumber * 8) - 5
    * Bit 1 = Ring number = (ringControlGroupNumber * 8) - 6
    * Bit 0 = Ring number = (ringControlGroupNumber * 8) - 7

    :param _list:
    :return:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017

    """

    power = max(_list)
    binary = ''
    for i in range(power):
        if i + 1 in _list:
            binary += '1'
        else:
            binary += '0'
    revB = binary[::-1]
    snmp_code = int(revB, 2)
    return snmp_code


def snmp_omit(_list):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.2.1 to the corresponding phase number and
        omit it. Hold is a command that causes omission of a selected phase.

    :param _list:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(_list) == list  #
    omit_temp = sorted(_list)
    if _list == [0]:
        snmp_terminate()
    else:
        snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', snmp_translate(omit_temp))


def snmp_hold(_list):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.4.1 to the corresponding phase number and
        hold it. Hold is a command that retains the existing Green interval.

    :param _list:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(_list) == list  #
    hold_temp = sorted(_list)
    if _list == [0]:
        snmp_terminate()
    else:
        snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', snmp_translate(hold_temp))


def snmp_force_off(_list):
    """
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

    :param _list:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(_list) == list
    force_off_temp = sorted(_list)
    if _list == [0]:
        snmp_terminate()
    else:
        snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', snmp_translate(force_off_temp))


def snmp_veh_call(_list):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.6.1 to the corresponding phase number and
        call a vehicle on it.

    :param _list:

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(_list) == list
    veh_call_temp = sorted(_list)
    if _list == [0]:
        snmp_terminate()
    else:
        snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', snmp_translate(veh_call_temp))


def snmp_terminate():
    """
    .. note::
        This module terminates all the commands and resets the signal controller to
        the default mode (actuated)

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', 0)
    snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', 0)
    snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', 0)
    snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', 0)


def snmp_phase_ctrl(phase, inter_name):
    """
    .. note::
        Sends command to ASC todo: what is an ASC


    :param phase: the phase to be called for green assignment
    :type phase: int
    :param inter_name:
    :type inter_name: str

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    num_phases, al, non, non_conflict = get_sig_ctrl_interface_params(inter_name)

    snmp_hold(list(al))
    snmp_hold(list(non))

    for p in range(len(non_conflict)):
        if phase in non_conflict[p]:
            snmp_veh_call(non_conflict[p])
            snmp_omit([i for i in al if i not in non_conflict[p]])

# Quickstart Test
# snmp_phase_ctrl(4)
