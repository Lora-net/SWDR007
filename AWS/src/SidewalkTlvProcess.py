"""
The Clear BSD License
Copyright Semtech Corporation 2024. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Semtech corporation nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

# Base64 to encode/decode from digital bytes to ASCII text
from base64 import b64decode, b64encode
# JSON for package uplink and downlink
import json
# HTTP.CLIENT for HTTP connections
import http.client
# Use struct for logical structure decoding
import struct
# Datetime for Date and Timezone calcs
from datetime import datetime, timezone
# Iteration for decoding TLV
from itertools import islice
# Include OS for accessing environment variables
import os
# Include logging for better print control
import logging
# Help with decimal values
import decimal
# The boto3 and botocore libraries for using Python with AWS IoT Core
import boto3
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

# Dynamodb as a database module
dynamodb = boto3.resource('dynamodb')
# IoTWireless library supporting Sidewalk and AICL (LoRaWAN)
client = boto3.client('iotwireless')
# Dynamodb table for storing uplink fragments (wdid(S) as primary key, TSTAMPMS(N) as sort key)
frag_table = dynamodb.Table(os.environ['FragStoreTableName'])
# Dynamodb table for storing previously defragmented packets
# (deduplication: wdid(S) as primary key, TSTAMPMS(N) as sort key)
dedup_table = dynamodb.Table(os.environ['DedupTableName'])
# Dynamodb table for storing uplink/downlink stats (wdid(S) as primary key, TSTAMPMS(N) as sort key)
mirror_table = dynamodb.Table(os.environ['MirrorTableName'])
# Dynamodb table for storing uplink/downlink scans (wdid(S) as primary key, TSTAMPMS(N) as sort key)
scan_table = dynamodb.Table(os.environ['ScanTableName'])
# Dynamodb table for storing DEVEUI to wdid mapping (DEVEUI(S) as primary key, wdid(s) as a value)
deveui_table = dynamodb.Table(os.environ['DEVEUITableName'])
# Target URI for processing LoRa Cloud messages
API_URI = 'mgs.loracloud.com'
# API for sending data messages
DEV_API = '/api/v1/uplink/send'
ADD_DEV_URI = API_URI + DEV_API
# Key for LoRa Cloud user
# !!! NOTE !!! Must be replaced by customer's LoRa Cloud key (loracloud.com) in samconfig.toml
MGS_KEY = os.environ['MgsKey']
# Header for processing data
mgs_header = {'Authorization': MGS_KEY, 'Content-Type': 'application/json'}

# Destination URI for posting location results
# !!! NOTE !!! Must be replaced by customer's destination URI
DESTINATION_URI = os.environ['DestinationUri']
# Destination POST/GET URI for posting location results
# !!! NOTE !!! Must be replaced by customer's destination API
DESTINATION_POST_API = os.environ['DestinationPostApi']
# Destination KEY for posting location results
# !!! NOTE !!! Must be replaced by customer's destination authorization key
DESTINATION_KEY = os.environ['DestinationKey']
# Destination header for posting location results
destination_post_header = {'X-API-Key': DESTINATION_KEY, 'Content-Type': 'application/json'}

# Downlink transmit modes (ACK or NOACK)
TX_NOACK = 0
TX_ACK = 1

# List of wdid to aggregate wifi
WDID_WIFI_LIST = ['3bb80362-7a14-4837-9397-fedb57e5fee0', 'aed1c40d-3870-4286-aa3c-cb4a2278bbbe',
                  'c5aeff44-b97b-40ad-96c1-de6fa405abf0']

# Convert ISO time format to timestamp
iso2ts = lambda iso: datetime.strptime(iso[0:19] + 'Z', '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()
iso2ts_tz = lambda iso: datetime.strptime(iso, '%Y-%m-%dT%H:%M:%S.%f%z').timestamp()
ts2iso = lambda ts: datetime.utcfromtimestamp(ts).strftime('%Y-%m-%dT%H:%M:%SZ')

# Setup logging for this file
LAMBDA_FUNC = True
if LAMBDA_FUNC:
    if logging.getLogger().hasHandlers():
        logger = logging.getLogger()
        logger.setLevel(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.INFO)
        logger = logging.getLogger('__lambda__')
else:
    import logging.config
    logging.config.fileConfig('logging.conf')
    # Define logging name
    logger = logging.getLogger('quiet')


# Helper class to convert a DynamoDB item to JSON.
class DecimalEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, decimal.Decimal):
            return str(o)
        if isinstance(o, set):   # <---resolving sets as lists
            return list(o)
        return super(DecimalEncoder, self).default(o)


# Parse TLV fields
def tlv_parser(tlv_string):
    TAG_FIELD_LENGTH = 2
    LENGTH_FIELD_LENGTH = 2
    it = iter(tlv_string)
    while tag := "".join(islice(it, TAG_FIELD_LENGTH)):
        length = int("".join(islice(it, LENGTH_FIELD_LENGTH)), 16)
        value = "".join(islice(it, length * 2))
        yield (tag, length, value)


# Send a downlink message via LoRaWAN
def SendDownlinkLoRaWAN(id: str, transmitMode: int, payload: str, port: int) -> dict:
    downResp = client.send_data_to_wireless_device(Id=id,
                                                   TransmitMode=transmitMode,
                                                   PayloadData=payload,
                                                   WirelessMetadata={'LoRaWAN': {'FPort': port}}
                                                   )
    return downResp


# Send a downlink message via Sidewalk
def SendDownlinkSidewalk(id: str, transmitMode: int, payload: str, seq: int) -> dict:
    downResp = client.send_data_to_wireless_device(Id=id,
                                                   TransmitMode=transmitMode,
                                                   PayloadData=payload,
                                                   WirelessMetadata={'Sidewalk': {'Seq': seq}}
                                                   )
    return downResp


# PayloadFragmentPush will store a current fragment from a device
def PayloadFragmentPush(params: dict):
    item = {}
    item['wdid'] = params['wdid']
    item['TSTAMPMS'] = int(params['timestamp'] * 1000)  # input  is floating-point seconds
    item['counter_up'] = params['counter_up']
    item['fragTot'] = params['fragTot']
    item['fragSeq'] = params['fragSeq']
    item['partHexPayload'] = params['hexASCIIPayload'][2:]
    logger.info('PUSH fragment:: {}'.format(item))
    response = frag_table.put_item(Item=item)
    logger.debug('Fragment PUSH response: {}'.format(response))


# FullPayloadPush will store a current full payload from a device
def FullPayloadPush(params: dict, recovered: dict):
    item = {}
    item['wdid'] = params['wdid']
    item['TSTAMPMS'] = recovered['timestamp_ms']
    item['fcnt'] = recovered['fcnt']
    item['incFcnts'] = json.dumps(recovered['incFcnts'], cls=DecimalEncoder)
    item['fragTot'] = recovered['fragments']
    item['hexPayload'] = recovered['payload']
    logger.info('PUSH recovered payload:: {}'.format(item))
    response = dedup_table.put_item(Item=item)
    logger.debug('Recovered payload PUSH response: {}'.format(response))


# Sorting function by counter_up
def sortByCounter(e):
    return (e['counter_up'])


# Sorting function by fragmentation segment
def sortBySeq(e):
    return (e['fragSeq'])


# Check function to determine if the defragmented payload is unique
def PayloadIsNew(params: dict, recovered: dict) -> bool:
    isNew = True
    wdid = params['wdid']
    try:
        response = dedup_table.query(
            KeyConditionExpression=Key("wdid").eq(wdid),
            Limit=5,
            ScanIndexForward=False  # true = ascending, false = descending
        )
        logger.info('dedup table response: {}'.format(response))
    except Exception:
        logger.error('Failure: dedup table read')
        # Save payload
        FullPayloadPush(params, recovered)
        return (isNew)

    # check that the current has not already been saved
    fcnt = recovered.get('fcnt', 0)
    fcnts = recovered.get('incFcnts', [])
    logger.info('Checking dedup history\nfcnts:{}'.format(fcnts))
    if 'Items' in response:
        for item in response['Items']:
            if 'incFcnts' in item:
                readIncFcnts = json.loads(item['incFcnts'])
                logger.info('included fcnts={}'.format(readIncFcnts))
                for readFcnt in readIncFcnts:
                    if str(readFcnt) in fcnts:
                        # this item was included in a previous complete payload
                        isNew = False
            elif (fcnt == item['fcnt']):
                # this max fcnt was included a previous complete payload
                isNew = False
        if isNew:
            logger.info('Save NEW payload: {}'.format(recovered))
            FullPayloadPush(params, recovered)
        else:
            logger.info('Duplicate payload found: {}'.format(recovered))

    return (isNew)


# Use past fragments to see if a completely defragmented payload can
#    be completed. If a completed payload is recovered, it is returned to the
#    calling routine
def DefragmentPayload(params: dict) -> dict:
    retDict = {}
    wdid = params['wdid']
    fragTotRef = int(params['fragTot'])
    fcntUpRef = int(params.get('counter_up', -2))
    tfLimit = fragTotRef * 2
    try:
        response = frag_table.query(
            KeyConditionExpression=Key("wdid").eq(wdid),
            Limit=tfLimit,
            ScanIndexForward=False  # true = ascending, false = descending
        )
        logger.debug('frag table response: {}'.format(response))
    except Exception:
        logger.error('Failure: frag table read')
        return (retDict)

    if 'Items' in response:
        fragList = []
        TSTAMPMS = 0
        FRAG = ''
        for item in response['Items']:
            logger.debug('defrag item: {}'.format(item))
            TSTAMPMS = item['TSTAMPMS']
            FRAG = item['partHexPayload']
            counter_up = item['counter_up']
            fragTot = item['fragTot']
            fragSeq = item['fragSeq']
            vDict = {'partPayload': FRAG, 'fragTot': fragTot, 'fragSeq': fragSeq,
                     'counter_up': counter_up, 'timestamp_ms': TSTAMPMS}
            fragList.append(vDict)
        fragList.sort(key=sortByCounter, reverse=True)
        logger.info('payload recall:{} '.format(fragList))
        # If the device resets, the fragList might have higher-numbered counter_up values
        # Skip to the index where the difference is within the max frag difference
        # Note we DON'T use the fcntUpRef directly because fcnt/counter_up may be out-of-order
        # We want the first value that is reasonable
        startIdx = 0
        while ((fragList[startIdx]['counter_up'] - fcntUpRef) > 7) and (startIdx < len(fragList)):
            startIdx = startIdx + 1
        if startIdx >= len(fragList):
            # Something weird happened, default to zero and flag error
            startIdx = 0
            logger.error('using default idx for reference fcnt ({})'
                         ' was not in fragList counter_up list'.format(fcntUpRef))
        # now check the list for a full frame
        defragValid = False
        frames = [fragList[startIdx]]
        fragSeqSet = {frames[0]['fragSeq']}
        fcnts = [frames[0]['counter_up']]
        # fragTot = frames[0]['fragTot']
        if (frames[0]['fragTot'] != fragTotRef):
            # This should never happen
            logger.error('the last frag record has the wrong fragTot')
            return retDict
        minFcnt = frames[0]['counter_up']
        maxFcnt = minFcnt
        recoveredPayload = ''
        recoveredTime = 0
        # Step through frag list finding unique frag numbers with the same fragTot
        for frag in fragList[startIdx:]:
            fcnt = frag['counter_up']
            fragSeq = frag['fragSeq']
            if (fragSeq not in fragSeqSet) and (frag['fragTot'] == fragTotRef):
                # frag makes sense
                if (abs(minFcnt - fcnt) < tfLimit) and (abs(maxFcnt - fcnt) < tfLimit):
                    # fcnt makes sense
                    frames.append(frag)
                    fcnts.append(fcnt)
                    fragSeqSet.add(fragSeq)
                    if (fcnt < minFcnt):
                        minFcnt = fcnt
                    if (fcnt > maxFcnt):
                        maxFcnt = fcnt
        # the frames list should now contain a list of unique fragSeq
        # First check, is it "full frame"
        if (0 in fragSeqSet) and (fcntUpRef in fcnts) and\
           ((fragTotRef - 1) in fragSeqSet) and (fragTotRef == len(fragSeqSet)):
            # the frame has the start, end and correct length of fragments
            # the latest fcnt needs to be in the completed frames fcnt list
            # now check fcnt to make sure it agrees with the number of fragments
            # this is basically a double-check as the logic should not allow this
            if ((maxFcnt - minFcnt + 1) < tfLimit):
                # last check, make sure highest frag is highest fCnt
                frames.sort(key=sortBySeq, reverse=False)    # sort by seq
                if frames[-1]['counter_up'] == maxFcnt:   # the highest seq number must have the highest fcnt
                    defragValid = True
                    recoveredTime = frames[-1]['timestamp_ms']
                    for frame in frames:
                        recoveredPayload = recoveredPayload + frame['partPayload']
        if defragValid:
            retDict = {'payload': recoveredPayload, 'fcnt': maxFcnt, 'fragments': fragTotRef,
                       'incFcnts': fcnts, 'timestamp_ms': recoveredTime}
        else:
            logger.warning('Could not recover valid sequence')
    else:
        logger.warning('no Items in defrag response')

    return retDict


# Send an HTTPS request (returns a dict)
def send_https_dict(uri: str, method: str, api: str, myData: str, mgs_header: str) -> dict:
    logger.info('method:{}, uri:{}, api:{}, data:{}'.format(method, uri, api, myData))
    conn = http.client.HTTPSConnection(uri)
    conn.request(method, api, myData, mgs_header)
    response = conn.getresponse()
    logger.debug('Raw http type:{}, response:{}'.format(type(response), response))
    retVal = json.loads(response.read().decode())
    conn.close()
    return (retVal)


# Send an HTTPS request (returns a string)
def send_https_str(uri: str, method: str, api: str, myData: str, mgs_header: str) -> str:
    logger.info('method:{}, uri:{}, api:{}, data:{}'.format(method, uri, api, myData))
    conn = http.client.HTTPSConnection(uri)
    conn.request(method, api, myData, mgs_header)
    response = conn.getresponse()
    logger.debug('Raw http type:{}, response:{}'.format(type(response), response))
    retVal = response.read().decode()
    conn.close()
    return (retVal)


# Convert a wireless device ID to EUI by using the last 6 bytes of the WDID
#    Prepend '0200' to complete the ficticious DevEUI
def Wdid2DevEUI(wdid: str) -> dict:
    dev_eui_nd = '0200' + wdid[-12:]
    deveui = '-'.join(dev_eui_nd[i:i + 2] for i in range(0, len(dev_eui_nd), 2)).upper()
    retDict = {'deveui': deveui, 'wdid': wdid}
    # Get the item table entry
    try:
        response = deveui_table.get_item(Key={'DEVEUI': deveui})
        logger.info(f'device query response: {response}')
    except ClientError as e:
        response = {}
        print(e.response['Error']['Message'])
    # Create an item if it down not exist
    if 'Item' not in response:
        # Create a default item in the DEVEUI table
        multiframe = False
        wifi_aggregate = False
        mf_length = 1
        item = {'DEVEUI': deveui, 'wdid': wdid,
                'WIFI_AGGREGATE': wifi_aggregate,
                'MULTIFRAME': multiframe,
                'MF_LENGTH': mf_length}
        logger.info('save deveui/wdid new table entry: {}'.format(item))
        response = deveui_table.put_item(Item=item)
        logger.info('wdid2deveui response:{}'.format(response))
        return (item)

    # Item is in response, proceed to use the data
    item = response['Item']
    if 'wdid' in item:
        # First, verify that the item and calling wdid agree
        if wdid == item['wdid']:
            if 'DEVEUI' in item:
                deveui = item['DEVEUI']
            multiframe = item.get('MULTIFRAME', False)
            wifi_aggregate = bool(item.get('WIFI_AGGREGATE', False))
            mf_length = int(item.get('MF_LENGTH', 1))
            retDict = {'deveui': deveui, 'wdid': wdid,
                       'MULTIFRAME': multiframe, 'WIFI_AGGREGATE': wifi_aggregate,
                       'MF_LENGTH': mf_length}
        else:
            retDict = {'deveui': deveui, 'wdid': wdid}

    logger.info(f'device setup retDict: {retDict}')
    return retDict


def Save2Mirror(item: dict) -> dict:
    logger.info('mirror table insert: {}'.format(item))
    mirrorResponse = mirror_table.put_item(Item=item)
    logger.debug('mirror table response: {}'.format(mirrorResponse))

    return (mirrorResponse)


# Store a scan record to the table
def StoreScanToDb(wdid: str, TSTAMPMS: int, port: int, hexASCIIPayload: str):
    item = {}
    item['wdid'] = wdid
    item['TSTAMPMS'] = TSTAMPMS
    item['port'] = port
    if port == 197:
        type = 'WIFI'
    elif port == 192:
        type = 'GNSSNG'
    elif port == 198:
        type = 'GNSS'
    else:
        type = 'UNKNOWN'
    item['SCAN_TYPE'] = type
    item['SCAN'] = hexASCIIPayload
    response = scan_table.put_item(Item=item)
    logger.debug('scan table insert response:{}'.format(response))

    return


# Retrieve a scan records from the table
def RecallScanTypeFromDb(wdid: str, type: str, num: int) -> list:
    # First read "num" items
    try:
        logger.info(f'scan wdid:{wdid} for type:{type} and len: {num}')
        response = scan_table.query(
            KeyConditionExpression=Key('wdid').eq(wdid),
            FilterExpression=Attr('SCAN_TYPE').eq(type),
            Limit=num,
            ScanIndexForward=False  # true = ascending, false = descending
        )
        logger.info(f'scan return: {response}')
    except Exception:
        logger.info('Failure: scan table read')
        return []
    if 'Items' in response:
        return response['Items']
    else:
        return []


# Turn a payload wifi string into a wifi dict
def DecodeWiFiScan(scan: str) -> dict:
    retD = {}
    logger.debug('scan:{}, scantype:{}'.format(scan, type(scan)))
    for i in range(0, len(scan), 12):
        mac = scan[i: i + 12]
        macAddress = ':'.join(mac[i: i + 2] for i in range(0, len(mac), 2))
        logger.debug('mac:{},macAddr:{}'.format(mac, macAddress))
        retD[macAddress] = mac
    logger.debug('scan dict:{}'.format(retD))
    return retD


def WifiScansToAggregate(inS: list) -> str:
    aps = {}
    for rec in inS:
        scan = rec['SCAN']
        timestamp = int(rec['TSTAMPMS']) / 1000.0
        logger.info('timestatmp:{},scan:{}'.format(timestamp, scan))
        # NOTE: skip the first byte because we know its the '00' formatter
        dws = DecodeWiFiScan(scan[2:])
        for k, v in dws.items():
            if k not in aps:
                aps[k] = v
    logger.debug('Aggregated APs:{}'.format(aps))
    wifiPayload = '00'
    for k, v in aps.items():
        wifiPayload = wifiPayload + v

    logger.info('aggregated wifiPayload:{}'.format(wifiPayload))
    return wifiPayload


def ProcessOrigFormat(params: dict, response: dict) -> bool:
    logger.info('ProcOrig input params:{}, input response: {}'.format(params, response))
    payloadProcessed = False
    port = 0
    fragTyp = params['fragTyp']
    if fragTyp == 0:
        port = 198  # frag type 0 is regular GNSS scan
    elif fragTyp == 1:
        port = 197  # frag type 1 is a WIFI scan
    elif fragTyp == 2:
        port = 199  # frag type 2 is a modem message
    else:
        port = 192  # frag type 3 is a GNSS-NG scan
    wdid = params['wdid']
    deviceDict = Wdid2DevEUI(wdid)
    deveui = deviceDict['deveui']
    wifi_aggregate = bool(deviceDict.get('WIFI_AGGREGATE', False))
    mf_length = int(deviceDict.get('MF_LENGTH', 1))
    payload = response['payload']

    if (port in [192, 197, 198]):
        # This is a scan message
        StoreScanToDb(wdid, response['timestamp_ms'], port, response['payload'])
    if (port in [192, 197, 198, 199]):
        # Sending a reference tracker message to MGS
        # First, aggregate WIFI (if enabled)
        if wifi_aggregate:
            scanList = RecallScanTypeFromDb(wdid, 'WIFI', mf_length)
            logger.info(f'scanList={scanList}')
            # now use that scan list
            oldPayload = payload
            payload = WifiScansToAggregate(scanList)
            logger.debug(f'oldPayload={oldPayload}, newPayload={payload}')

        # Now process the data
        timestamp = int(float(response['timestamp_ms']) / 1000.0)
        dmsmsg = json.dumps({deveui: {"fcnt": int(response['fcnt']),
                                      "port": port,
                                      "payload": payload,
                                      "timestamp": timestamp}
                             })
        print('dmsmsg:{}'.format(dmsmsg))
        dmsResponse = send_https_str(API_URI, 'POST', DEV_API, dmsmsg, mgs_header)
        print('dmsResponse type:{}, dmsResponse:{}'.format(type(dmsResponse), dmsResponse))
        jsonResponse = json.loads(dmsResponse)
        if 'result' in jsonResponse:
            theDevEUI = list(jsonResponse['result'].keys())[0]
            if 'result' in jsonResponse['result'][theDevEUI]:
                result = jsonResponse['result'][theDevEUI]['result']
                if result['position_solution'] is not None:
                    xmitMsg = json.dumps({'deveui': result['deveui'],
                                          'counter_up': int(response['fcnt']),
                                          # 'scan_count': scanCount,
                                          # 'app_count_up': appCountUp,
                                          'port': port,
                                          'position_solution': result['position_solution'],
                                          'solution_type': result['operation']
                                          }, cls=DecimalEncoder)
                    dlist = list(result['deveui'])
                    dlist[4] = '2'  # change the deveui in the Production send
                    deveui_2 = "".join(dlist)
                    xmitMsg_2 = json.dumps({'deveui': deveui_2,
                                            'counter_up': int(response['fcnt']),
                                            # 'scan_count': scanCount,
                                            # 'app_count_up': appCountUp,
                                            'port': port,
                                            'position_solution': result['position_solution'],
                                            'solution_type': result['operation']
                                            }, cls=DecimalEncoder)
                    logger.info('POST to DESTINATION |{}| to URI:{} \
                        on API:{}'.format(xmitMsg_2, DESTINATION_URI, DESTINATION_POST_API))
                    destinationResponse = send_https_str(DESTINATION_URI, 'POST',
                                                     DESTINATION_POST_API, xmitMsg_2, destination_post_header)
                    logger.info('Response to DESTINATION POST:{}'.format(destinationResponse))
                    payloadProcessed = True

    return payloadProcessed


# Once a payload is defragmented, process it by LoRa Cloud
def ProcessPayload(params: dict, network: dict) -> bool:
    # Stats collection
    scanCount = -1
    fcntUp = params.get('counter_up', -2)
    appCountUp = 0
    lastUpCount = 0
    upType = -1
    rssi = 99
    snr = 99
    positionSolution = '{}'
    fullHexPayload = ''
    msgType = ''
    # Set response value
    payloadProcessed = False
    countReceived = False
    # First save the latest fragment
    PayloadFragmentPush(params)

    # Next,save the input to the mirror database
    deviceDict = Wdid2DevEUI(params['wdid'])
    deveui = deviceDict['deveui']
    wdid = deviceDict['wdid']
    wifi_aggregate = bool(deviceDict.get('WIFI_AGGREGATE', False))
    mf_length = int(deviceDict.get('MF_LENGTH', 1))

    # Setup and save mirror data for post-processing
    item = {}
    item['DEVEUI'] = deveui
    item['wdid'] = wdid
    item['TSTAMPMS'] = int(params['timestamp'] * 1000)
    item['fragTot'] = params['fragTot']
    item['fragSeq'] = params['fragSeq']
    item['partHexPayload'] = params['hexASCIIPayload'][2:]
    item['fcnt'] = fcntUp
    logger.info('FRAG-mirror: save table item: {}'.format(item))
    mirrorResponse = Save2Mirror(item)
    logger.debug('FRAG-mirror: write table response: {}'.format(mirrorResponse))

    # Run defragment logic to see if this is the last in frag chunks
    response = DefragmentPayload(params)

    # Payload in the response is the indication that a complete payload can be defraged
    if 'payload' in response:
        logger.info('Valid payload recovery: {}'.format(response))
        if PayloadIsNew(params, response) is False:
            return (payloadProcessed)
        # Payload is new (non-duplicate)
        rawPayload = response['payload']
        # FCnts included in new data
        incFcnts = response.get('incFcnts', [])
        # Save complete payload for output
        fullHexPayload = rawPayload
        # Check if first byte is a TLV tag
        tag = rawPayload[0:2]
        if tag.upper() not in ['50', '51', '52', '53', '54']:
            logger.warning('WARNING! Unrecognized TLV tag: {}\nProcess as Orig format'.format(tag))
            payloadProcessed = ProcessOrigFormat(params, response)
        else:
            # Iterate through the TLV data
            for tlv in tlv_parser(rawPayload):
                tag, length, val = tlv
                logger.info('received tag:{},length:{},value:{}'.format(tag, length, val))
                vl = int(len(val) / 2)
                TAG = tag.upper()
                if length > vl:
                    logger.warning('WARNING! reported length ({}) is greater \
                                    than actual length ({})'.format(length, vl))
                if TAG not in ['50', '51', '52', '53', '54']:
                    logger.warning('WARNING! Unrecognized TLV tag: {}'.format(tag))
                port = -1
                if (TAG == '50'):
                    # GNSS scan data
                    msgType = 'GNSS'
                    scanCount = int(struct.unpack(">H", bytes.fromhex(val[0:4]))[0])
                    logger.info('scanCount: {}'.format(scanCount))
                    payload = val[4:]
                    port = 198
                elif (TAG == '51'):
                    # WIFI scan data
                    msgType = 'WIFI'
                    scanCount = int(struct.unpack(">H", bytes.fromhex(val[0:4]))[0])
                    logger.info('scanCount: {}'.format(scanCount))
                    payload = val[4:]
                    # payload = val
                    port = 197
                elif (TAG == '52'):
                    # Modem uplink
                    msgType = 'modem'
                    payload = val
                    port = 199
                elif (TAG == '53'):
                    # GNSS-NG
                    msgType = 'GNSS-NG'
                    payload = val
                    port = 192
                elif (TAG == '54'):
                    # Count packet
                    lastUpCount, appCountUp, rssiRAW, snrRAW, upType = struct.unpack(">HHbbb", bytes.fromhex(val))
                    rssi = rssiRAW - 64
                    snr = snrRAW / 4
                    port = 54
                    logger.info('Received data: lastUpCount={} appUpCount={} rssi={} \
                        snr={} upType={} port={}'.format(lastUpCount, appCountUp, rssi, snr, upType, port))
                    countReceived = True
                else:
                    logger.warning('Unrecognized tag value: {}'.format(TAG))

                if (port == 54):
                    # If we have received the count packet, send the downlink message with stats
                    byteValues = struct.pack('>BBHH', 0x55, 4, abs(fcntUp) % 65535, abs(appCountUp) % 65535)
                    downPayload = b64encode(byteValues).decode('utf-8')  # downlink encode
                    logger.info('sending downlink: wdid={},ack={},payload={},\
                        seq={}'.format(wdid, TX_NOACK, downPayload, params['counter_up']))
                    downResponse = SendDownlinkSidewalk(wdid, TX_NOACK, downPayload, params['counter_up'])
                    logger.debug('downResponse = {}'.format(downResponse))

                if (port in [192, 197, 198]):
                    # This is a scan message
                    StoreScanToDb(wdid, response['timestamp_ms'], port, response['payload'])
                if (port in [192, 197, 198, 199]):
                    # Sending a reference tracker message to MGS
                    # First, aggregate WIFI
                    if wifi_aggregate:
                        scanList = RecallScanTypeFromDb(wdid, 'WIFI', mf_length)
                        logger.info(f'scanList={scanList}')
                        # now use that scan list
                        oldPayload = payload
                        payload = WifiScansToAggregate(scanList)
                        logger.debug(f'oldPayload={oldPayload}, newPayload={payload}')

                    timestamp = int(float(response['timestamp_ms']) / 1000.0)
                    dmsmsg = json.dumps({
                                        deveui: {
                                            "fcnt": fcntUp,
                                            "port": port,
                                            "payload": payload,
                                            "timestamp": timestamp
                                        }
                                        }, cls=DecimalEncoder)
                    logger.info('dmsmsg:{}'.format(dmsmsg))
                    dmsResponse = send_https_str(API_URI, 'POST', DEV_API, dmsmsg, mgs_header)
                    logger.info('dmsResponse type:{}, dmsResponse:{}'.format(type(dmsResponse), dmsResponse))
                    jsonResponse = json.loads(dmsResponse)
                    if 'result' in jsonResponse:
                        theDevEUI = list(jsonResponse['result'].keys())[0]
                        if 'result' in jsonResponse['result'][theDevEUI]:
                            result = jsonResponse['result'][theDevEUI]['result']
                            if result['position_solution'] is not None:
                                positionSolution = result['position_solution']
                                dlist = list(result['deveui'])
                                dlist[4] = '2'  # change the deveui in the Production send
                                deveui_2 = "".join(dlist)
                                xmitMsg_2 = json.dumps({'deveui': deveui_2,
                                                        'counter_up': fcntUp,
                                                        'scan_count': scanCount,
                                                        'app_count_up': appCountUp,
                                                        'port': port,
                                                        'position_solution': positionSolution,
                                                        'solution_type': result['operation']
                                                        }, cls=DecimalEncoder)
                                logger.info('POST to DESTINATION |{}| to URI:{} \
                                    on API:{}'.format(xmitMsg_2, DESTINATION_URI, DESTINATION_POST_API))
                                destinationResponse = send_https_str(DESTINATION_URI, 'POST',
                                                                 DESTINATION_POST_API, xmitMsg_2, destination_post_header)
                                logger.info('Response to DESTINATION POST:{}'.format(destinationResponse))
                                payloadProcessed = True
                else:
                    logger.info('invalid MGS port number: {}'.format(port))

    if (countReceived or payloadProcessed):
        # We received a count or a payload
        # Save what we saw to mirror db
        if countReceived:
            item['rssi'] = str(rssi)
            item['snr'] = str(snr)
            item['lastUpCount'] = lastUpCount
            item['appCountUp'] = appCountUp
            item['upType'] = upType
            item['scanCount'] = scanCount
        item['msgType'] = msgType
        item['posFix'] = payloadProcessed
        item['positionSolution'] = json.dumps(positionSolution, cls=DecimalEncoder)
        item['incFcnts'] = json.dumps(incFcnts, cls=DecimalEncoder)
        item['fullHexPayload'] = fullHexPayload
        logger.info('COMBO-mirror item:{}'.format(item))
        mirrorResponse = Save2Mirror(item)
        logger.debug('COMBO-mirror write response: {}'.format(mirrorResponse))

    return (payloadProcessed)


# Take the Sidewalk input and apply defragmentation and LoRa Cloud processing
def ProcessSidewalkInput(qsp: dict) -> tuple:
    statusCode = 200
    responseBody = 'Accepted'
    # promote data from input structures
    params = {}
    if 'wdid' in qsp:
        params['wdid'] = qsp['wdid']
    if 'seq' in qsp:
        params['counter_up'] = qsp['seq']
    if 'payload' in qsp:
        params['payload'] = qsp['payload']
        hexPayload = b64decode(qsp['payload']).decode('ascii')  # sidewalk encodes ascii hex this way
        params['hexASCIIPayload'] = hexPayload
        fragTypeByte = int(hexPayload[0:2], 16)
        fragSeq = fragTypeByte & 0x07          # lowest 3 bits, fragment counter
        fragTot = (fragTypeByte & 0x38) >> 3    # next 3 bits, total fragments
        fragTyp = (fragTypeByte & 0xC0) >> 6    # highest 2 bits, message type
        params['fragTot'] = fragTot
        params['fragSeq'] = fragSeq
        params['fragTyp'] = fragTyp
    if 'timestamp' in qsp:
        timestampNIX = float(qsp['timestamp'])  # this has fractional seconds, thus float
        timestampISO = ts2iso(int(timestampNIX))
        params['timestampISO'] = timestampISO
        params['timestamp'] = timestampNIX
    if 'type' in qsp:
        msgType = qsp['type']
        params['type'] = msgType
    logger.info('Sidewalk params: {}'.format(params))
    if ('wdid' in params) and ('timestamp' in params):
        if ('payload' in params) and ('counter_up' in params) and (msgType == 'uplink'):
            # This is an actual uplink
            network = dict(name='Sidewalk', type='sidewalk',
                           topic='sidewalk/downlink', id=params['wdid'],
                           payloadName='payload_raw', payloadFormat='base64',
                           DEVEUI=params['wdid'].upper(), timestamp=timestampNIX)
            logger.info('up_counter: {}, base64_payload: {}, \
                  hex_payload: {}'.format(params['counter_up'], params['payload'],
                                          params['hexASCIIPayload']))
            if ProcessPayload(params, network):
                logger.info('Payload processing complete')
        else:
            logger.error('Missing payload, or fcnt')
    else:
        logger.error('No wdid, payload, counter_up or timestamp')

    return (statusCode, responseBody)


# AWS Lambda serverless function to process input packets
def lambda_handler(event, context):
    logger.info('event: {}'.format(event))
    statusCode = 200
    responseBody = 'Accepted'
    # First check if this is an AICL
    if 'WirelessMetadata' in event:
        if 'LoRaWAN' in event['WirelessMetadata']:
            data = {}
            lw = event['WirelessMetadata']['LoRaWAN']
            DevEui = lw['DevEui']
            dev_eui_nd = DevEui
            data['dev_eui_nd'] = dev_eui_nd
            deveui = '-'.join(DevEui[i:i + 2] for i in range(0, len(DevEui), 2))
            data['deveui'] = deveui.upper()
            DEVEUI = deveui.upper()
            deveui = DEVEUI.lower()
            data['fcnt'] = int(lw.get('FCnt', 0))
            data['port'] = int(lw.get('FPort', 0))
            data['dr'] = lw['DataRate']
            data['freq'] = lw['Frequency']
            data['timestamp'] = iso2ts(lw['Timestamp'])
            data['id'] = event['WirelessDeviceId']
            data['devaddr'] = lw['DevAddr']
            data['payload'] = event['PayloadData']
            if 'DataUp' in lw['MType']:
                data['type'] = 'uplink'
            else:
                data['type'] = 'unknown'
            data['lora'] = lw
            logger.info('Input for AICL processing: {}'.format(data))
            # This implementation NOT for AICL
            # statusCode, responseBody = ProcessAICLInput(data)
        elif 'Sidewalk' in event['WirelessMetadata']:
            data = {}
            sw = event['WirelessMetadata']['Sidewalk']
            wdid = event['WirelessDeviceId']
            data['wdid'] = wdid
            data['WDID'] = wdid.upper()
            data['seq'] = int(sw.get('Seq', 0))
            data['timestamp'] = datetime.timestamp(datetime.now())
            data['id'] = event['WirelessDeviceId']
            data['sidewalkid'] = sw['SidewalkId']
            data['payload'] = event['PayloadData']
            data['type'] = 'uplink'
            data['sidewalk'] = sw
            logger.info('Input for Sidewalk processing: {}'.format(data))
            # This implementation NOT for AICL
            statusCode, responseBody = ProcessSidewalkInput(data)

    return {
        'statusCode': statusCode,
        'body': responseBody
    }
