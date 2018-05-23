from pysnmp.hlapi import *


def Ashsnmpset(OID, Value):
    """

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


# ----------------------------------------------------------------------------

def snmpTranslate(List):
    """
    This module translates the phase numbers in a given list into snmp legible
    integers according to NTCIP 1202. The code encripts the list of the phases
    into a binary string and then parses it to an snmp int value.

    Example: 2^^3 phase translation breakdown:

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


def AshOmit(List):
    """
    This module transforms the bit matrix values for OID
    enterprise::1206.4.2.1.1.5.1.2.1 to the corresponding phase number and
    omit it. Hold is a command that causes omission of a selected phase.

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(List) == list  #
    OmitTemp = sorted(List)
    if List == [0]:
        AshTerminate()
    else:
        Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', snmpTranslate(OmitTemp))

def AshHold(List):
    """
    This module transforms the bit matrix values for OID

    enterprise::1206.4.2.1.1.5.1.4.1 to the corresponding phase number and
    hold it. Hold is a command that retains the existing Green interval.

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(List) == list  #
    HoldTemp = sorted(List)
    if List == [0]:
        AshTerminate()
    else:
        Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', snmpTranslate(HoldTemp))


def AshForceOff(List):
    """
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
    assert type(List) == list
    ForceOffTemp = sorted(List)
    if List == [0]:
        AshTerminate()
    else:
        Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', snmpTranslate(ForceOffTemp))

def AshVehCall(List):
    """
    This module transforms the bit matrix values for OID


    enterprise::1206.4.2.1.1.5.1.6.1 to the corresponding phase number and
    call a vehicle on it.

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    assert type(List) == list
    VehCallTemp = sorted(List)
    if List == [0]:
        AshTerminate()
    else:
        Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', snmpTranslate(VehCallTemp))


def AshTerminate():
    """
    This module terminates all the commands and resets the signal controller to
    the default mode (actuated)

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """


Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1', 0)
Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1', 0)
Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1', 0)
Ashsnmpset('1.3.6.1.4.1.1206.4.2.1.1.5.1.6.1', 0)
# ----------------------------------------------------------------------------

NoPhase = 8  # insert the maximum phase number
al = range(1, NoPhase + 1)
non = [0]
AshHold(al)
AshHold(non)


def AshPhaseCtrl(Phase):  # set up according to SPaT on ASC (currently for TERL)
    """

    :Author:
        Aschkan Omidvar <aschkan@ufl.edu>
    :Date:
        Jan-2017
    """
    if Phase == 3 or Phase == 8:
        AshVehCall([3, 8])
        AshOmit([1, 2, 4, 5, 6, 7])
    elif Phase == 7 or Phase == 4:
        AshVehCall([4, 7])
        AshOmit([1, 2, 3, 5, 6, 8])
    else:
        AshVehCall([Phase])
        AshOmit([i for i in al if i != Phase])


AshPhaseCtrl(4)
# for i in range(10):
#    AshPhaseCtrl(mod(i, 4) + 1)
#    time.sleep(4.6)
