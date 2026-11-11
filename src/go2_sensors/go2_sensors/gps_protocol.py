"""NMEA checksum, GGA used satellites, RMC valid position. No external dependencies."""
import math

def parse_nmea(line):
    if isinstance(line, bytes):
        line=line.decode('ascii','ignore')
    line=line.strip()
    if not line.startswith('$') or '*' not in line:
        return None
    payload,checksum=line[1:].split('*',1)
    if len(checksum)<2:
        return None
    chk=0
    for c in payload:
        chk ^= ord(c)
    try:
        if int(checksum[:2],16)!=chk:
            return None
    except ValueError:
        return None
    fields=payload.split(',')
    if len(fields[0])!=5 or fields[0][0:2] not in ('GP','GN','GL','GA','GB','BD'):
        return None
    typ=fields[0][2:]
    try:
        if typ=='GGA' and len(fields)>=8:
            quality=int(fields[6] or 0)
            used=int(fields[7] or 0)
            return ('GGA',quality,used)
        if typ=='RMC' and len(fields)>=7:
            valid=fields[2]=='A'
            if not valid:
                return ('RMC',False,None,None)
            lat=nmea_degree(fields[3],fields[4],90,'NS')
            lon=nmea_degree(fields[5],fields[6],180,'EW')
            return ('RMC',True,lat,lon)
    except (ValueError,IndexError,TypeError):
        return None
    return None

def nmea_degree(value,hem,limit,allowed):
    if not value or hem not in allowed:
        raise ValueError('Missing position')
    degrees=math.floor(float(value)/100)
    minutes=float(value)-100*degrees
    if not 0<=minutes<60:
        raise ValueError('Invalid minutes')
    n=degrees+minutes/60
    if not math.isfinite(n) or n>limit:
        raise ValueError('Invalid coordinate')
    return -n if hem in ('S','W') else n
