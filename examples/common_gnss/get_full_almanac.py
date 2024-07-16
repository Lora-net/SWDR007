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

import base64
import requests

import sys
from argparse import ArgumentParser


def main():

    # The file in which the almanac will be written to
    filename_default = "almanac.h"
    parser = ArgumentParser(
        description="Companion software that generates almanac header file to be compiled for LR1110/LR1120 embedded full almanac update."
    )
    parser.add_argument(
        "mgs_token", help="MGS LoRa Cloud token to use to fetch the almanac"
    )
    parser.add_argument(
        "-f",
        "--output_file",
        help="file that will contain the results",
        default=filename_default,
    )
    args = parser.parse_args()

    mgs_token = args.mgs_token
    filename = args.output_file

    # Build request URL
    url = "https://mgs.loracloud.com/api/v1/almanac/full"
    print("Requesting latest full almanac image available...")

    # HTTP request to MGS
    my_header = {"Authorization": mgs_token}
    res = requests.get(url, headers=my_header)

    if res.status_code != 200:
        print("ERROR: failed to get almanac - " + str(res))
        sys.exit(2)
    else:
        print("Success")

    raw_bytes = bytes(base64.b64decode(res.json()["result"]["almanac_image"]))

    # Build the byte array containing the almanac to be written to LR11xx
    my_almanac_in_hex = "static const uint8_t full_almanac[( LR11XX_GNSS_FULL_UPDATE_N_ALMANACS * LR11XX_GNSS_SINGLE_ALMANAC_WRITE_SIZE ) + 20] = { "
    my_almanac_in_hex += ", ".join("0x{:02X}".format(byt) for byt in raw_bytes)
    my_almanac_in_hex += " };"

    # Write C file to be included to the full_almanac_update application for writting to LR11xx
    file_header = (
        "/* This file has been auto-generated by the get_full_almanac.py script */\n\n"
    )
    file_header += '#include "lr11xx_gnss.h"\n\n'
    with open(filename, "w") as f:
        f.write(file_header + my_almanac_in_hex + "\n")

    print("Almanac image written to " + filename + " file")


if __name__ == "__main__":
    main()
