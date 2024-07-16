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

# Include OS for accessing environment variables
import os
# JSON for package uplink and downlink
import json
# Help with decimal values
import decimal
# Datetime for Date and Timezone calcs
from datetime import datetime, timedelta, timezone
# The boto3 and botocore libraries for using Python with AWS IoT Core
import boto3
from botocore.exceptions import ClientError
from boto3.dynamodb.conditions import Key, Attr

# Dynamodb Aliases
dynamodb = boto3.resource('dynamodb')
deveui_table = dynamodb.Table(os.environ['DEVEUITableName'])
mirror_table = dynamodb.Table(os.environ['MirrorTableName'])

# Convert ISO time format to timestamp
iso2ts = lambda iso: datetime.strptime(iso[0:19] + 'Z', '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=timezone.utc).timestamp()
iso2ts_tz = lambda iso: datetime.strptime(iso, '%Y-%m-%dT%H:%M:%S.%f%z').timestamp()
ts2iso = lambda ts: datetime.utcfromtimestamp(ts).strftime('%Y-%m-%dT%H:%M:%SZ')


# Helper class to convert a DynamoDB item to JSON.
class DecimalEncoder(json.JSONEncoder):
    def default(self, o):
        if isinstance(o, decimal.Decimal):
            return str(o)
        if isinstance(o, set):   # <---resolving sets as lists
            return list(o)
        return super(DecimalEncoder, self).default(o)


# Find wdid from DEVEUI
def GetWDID(DEVEUI: str) -> str:
    devices_response = deveui_table.query(
        KeyConditionExpression=Key('DEVEUI').eq(DEVEUI),
    )
    print('wdid response: {}'.format(devices_response))
    items = devices_response.get('Items', [])
    wdid = ''
    if len(items) > 0:
        wdid = items[0]['wdid']

    return wdid


def QueryItemsWithLimit(table, query_parameters, LIMIT):
    maxChunkSize = 1500
    items = []
    response = {}
    if LIMIT > maxChunkSize:
        query_parameters['Limit'] = maxChunkSize
        print('query_params:{}'.format(query_parameters))
        response = table.query(**query_parameters)
        print('RESPONSE {}'.format(response))
        items.extend(response.get('Items', []))
        while ('LastEvaluatedKey' in response) and (len(items) < LIMIT):
            if (len(items) + maxChunkSize) > LIMIT:
                query_parameters['Limit'] = (LIMIT - len(items))
            response = table.query(
                **query_parameters,
                ExclusiveStartKey=response['LastEvaluatedKey']
            )
            print('RESPONSE {}'.format(response))
            items.extend(response.get('Items', []))
    else:
        query_parameters['Limit'] = LIMIT
        print('query_params:{}'.format(query_parameters))
        response = table.query(query_parameters)
        print('RESPONSE {}'.format(response))
        items = response.get('Items', [])

    return items


def get_last_record_for_device(wdid: str) -> dict:
    try:
        record = QueryItemsWithLimit(mirror_table, dict(
            Limit=1, ScanIndexForward=False,
            KeyConditionExpression=Key('wdid').eq(wdid)), 1)[0]
        # # Build outDict
        # outDict = {}
        # for key, value in record.items():
        #     print('record key:{} value:{}'.format(key, value))
        #     outDict[key] = value
    except ClientError as e:
        return (400, e.response['Error'])

    return record


def lambda_handler(event, context):
    # Show event debug
    print('event:{}'.format(event))
    incomingBody = {}
    httpMethod = event.get('httpMethod', '')
    if httpMethod == 'POST':
        # This is a POST
        try:
            incomingBody = json.loads(event['body'])
        except Exception:
            httpResponse = {"statusCode": 400,
                            "body": 'Invalid JSON body: {}'.format(event['body']),
                            "isBase64Encoded": False
                            }
            print('httpResponse:{}'.format(httpResponse))
            return httpResponse
    elif httpMethod == 'GET':
        # This is a GET
        try:
            incomingBody = event['queryStringParameters']
        except Exception:
            httpResponse = {"statusCode": 400,
                            "body": 'Invalid qsp: {}'.format(event['queryStringParameters']),
                            "isBase64Encoded": False
                            }
            print('httpResponse:{}'.format(httpResponse))
            return httpResponse
    else:
        httpResponse = {"statusCode": 400,
                        "body": 'Invalid httpRequest method: {}'.format(httpMethod),
                        "isBase64Encoded": False
                        }
        print('httpResponse:{}'.format(httpResponse))
        return httpResponse

    DEVEUI = ''
    query_mode = 'LAST_RECORD'
    LIMIT = 10000000
    # last_evaluated_key = None
    wdid = ''

    if 'LIMIT' in incomingBody:
        LIMIT = incomingBody['LIMIT']
        query_mode = 'KEY'

    if 'END_DATE' in incomingBody:
        end_date_ms = int(iso2ts(incomingBody['END_DATE'])) * 1000
        query_mode = 'KEY'
    else:
        end_date_ms = int(datetime.timestamp(datetime.utcnow())) * 1000

    if 'START_DATE' in incomingBody:
        start_date_ms = int(iso2ts(incomingBody['START_DATE'])) * 1000
        query_mode = 'KEY'
    else:
        start_date_ms = int(datetime.timestamp(datetime.today() - timedelta(days=7))) * 1000

    wdid = incomingBody.get('wdid', '')
    if 'DEVEUI' in incomingBody:
        DEVEUI = incomingBody['DEVEUI']
        WDID = GetWDID(DEVEUI)
        if len(WDID) > 0:
            wdid = WDID

    # if 'LAST_EVALUATED_KEY' in incomingBody:
    #     last_evaluated_key = incomingBody['LAST_EVALUATED_KEY']
    #     query_mode = 'KEY'

    if incomingBody.get('ONLY_LAST_RECORD_PER_DEVICE', False):
        query_mode = 'LAST_RECORD'

    if query_mode == 'KEY':
        print('In Key Mode   wdid={} start_date_ms={} end_date_ms={}'.format(wdid, start_date_ms, end_date_ms))
        try:
            query_parameters = dict(
                Limit=LIMIT,
                ScanIndexForward=False,  # False = forward in time (oldest first)
                KeyConditionExpression=(Key('wdid').eq(wdid) & Key('TSTAMPMS').between(start_date_ms, end_date_ms))
            )
            obtained_items = QueryItemsWithLimit(mirror_table, query_parameters, LIMIT)
        except ClientError as e:
            print(e.response['Error']['Message'])
            return (400, e.response['Error'])

        httpResponse = {"statusCode": 200,
                        "body": json.dumps(obtained_items, cls=DecimalEncoder),
                        "isBase64Encoded": False
                        }
        print('httpResponse:{}'.format(httpResponse))
        return httpResponse
    elif query_mode == 'LAST_RECORD':
        print("Last Record Per Device\nwdid:{} DEVEUI:{}".format(wdid, DEVEUI))
        lastRecord = json.dumps(get_last_record_for_device(wdid),
                                cls=DecimalEncoder)
        print('device result:{}'.format(lastRecord))

        httpResponse = {"statusCode": 200,
                        "body": lastRecord,
                        "isBase64Encoded": False
                        }
        print('httpResponse:{}'.format(httpResponse))
        return httpResponse
    else:
        print('No query mode set')

    httpResponse = {"statusCode": 200,
                    "body": 'No data for provided wdid/DEVEUI',
                    "isBase64Encoded": False
                    }
    print('httpResponse:{}'.format(httpResponse))
    return httpResponse
