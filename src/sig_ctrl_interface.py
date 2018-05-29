#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright pending (c) 2017, Aschkan Omidvar <aschkan@ufl.edu>
# Created on Jan. 2017
# Updated on May 2018
# University of Florida
# UF Transportation Institute
# Dept. of Civil and Coastal Engineering
# @author: aschkan

from pysnmp.hlapi import *
from data.data import get_sig_ctrl_interface_params  # grab intersection parameters from data directory


def snmp_set(OID, Value):
    """
    todo: a line what it does

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(OID) == str
    errorIndication, errorStatus, errorIndex, varBinds = next(
        setCmd(SnmpEngine(),
               CommunityData('public', mpModel=0),  # snmp v1. delete mpModel for v2c),
               UdpTransportTarget(('169.254.91.71', 161)),
               ContextData(),
               ObjectType(ObjectIdentity(str(OID)), Integer(Value))))

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
        integers according to NTCIP 1202. The code encripts the list of the phases
        into a binary string and then parses it to an snmp int value.

    Example
    -------
     2^^3 phase translation breakdown:
            
    Bit 7 = Ring number = (ringControlGroupNumber * 8)
    Bit 6 = Ring number = (ringControlGroupNumber * 8) - 1
    Bit 5 = Ring number = (ringControlGroupNumber * 8) - 2
    Bit 4 = Ring number = (ringControlGroupNumber * 8) - 3
    Bit 3 = Ring number = (ringControlGroupNumber * 8) - 4
    Bit 2 = Ring number = (ringControlGroupNumber * 8) - 5
    Bit 1 = Ring number = (ringControlGroupNumber * 8) - 6
    Bit 0 = Ring number = (ringControlGroupNumber * 8) - 7
    
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
    snmpCode = int(revB, 2)
    return snmpCode


def snmp_omit(List):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.2.1 to the corresponding phase number and
        omit it. Hold is a command that causes omission of a selected phase.

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(List) == list  #
    omit_temp = sorted(List)
    if List == [0]:
        snmp_terminate()
    else:
        snmp_set('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', snmp_translate(omit_temp))


def snmp_hold(_list):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.4.1 to the corresponding phase number and
        hold it. Hold is a command that retains the existing Green interval.

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


def snmp_veh_call(List):
    """
    .. note::
        This module transforms the bit matrix values for OID
        enterprise::1206.4.2.1.1.5.1.6.1 to the corresponding phase number and
        call a vehicle on it.

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(List) == list
    veh_call_temp = sorted(List)
    if List == [0]:
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


def snmp_phase_ctrl(phase, inter_naame):
    """
    .. note::
        Sends command to ASC todo: what is an ASC

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    num_phases, al, non, non_conflict = get_sig_ctrl_interface_params(inter_naame)

    snmp_hold(list(al))
    snmp_hold(list(non))

    for p in range(len(non_conflict)):
        if phase in non_conflict[p]:
            snmp_veh_call(non_conflict[p])
            snmp_omit([i for i in al if i not in non_conflict[p]])

# Quickstart Test
# snmpPhaseCtrl(4)
