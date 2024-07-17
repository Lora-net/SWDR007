# SWDR007 - LR11xx Sidewalk Driver for EFR32MG24

This repository contains the software driver that enables the [LR11xx family](https://www.semtech.com/products/wireless-rf/lora-edge) of silicon to support the Sidewalk protocol when paired with the [SiLabs EFR32MG24 MCU](https://www.silabs.com/development-tools/wireless/proprietary/pro-kit-for-amazon-sidewalk?tab=overview) and  [Amazon Sidewalk - SoC Bluetooth Sub-GHz Hello Neighbor](https://docs.silabs.com/amazon-sidewalk/latest/sidewalk-getting-started/create-and-compile-application#create-an-amazon-sidewalk-project) SDK.

It contains four examples, *example_lr11xx, example_wifi, example_gnss and example_gnss_wifi*.

AWS sub-directory contains the AWS Lambda implementation that enables re-assembly of the packet stream in AWS IOT Core.

You can learn more about the source code details from the [readme-SiLabs](./readme-SiLabs.md).

## Documentation

Browse the official Sidewalk documentation:

- Amazon Sidewalk: https://docs.sidewalk.amazon
- Silicon Labs for Sidewalk SDK: https://docs.silabs.com/amazon-sidewalk/latest/sidewalk-getting-started/

## Prerequisites

### Hardware

- 1x xG24-BRD4187C Rev A01-A Radio Board
- 1x BRD8042A Rev A02 CSS/FSK Adapter Board or 1x E707V02A Adapter Board
- [LR11xx Supported Shields](#lr11xx-supported-shields)

- 1x 915 MHz Antenna

#### LR11xx Supported Shields

The list of compatible Semtech LR1110 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        | Firmware |
| ------------ | -------- | ------------------ | -------------------------------------- | -------- |
| LR1110MB1DIS | E516V02B | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna | 0x0308   |
| LR1110MB1DJS | E592V01B | 868/915 MHz        | GNSS without LNA                       | 0x0308   |

The list of compatible Semtech LR1120 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        | Firmware |
| ------------ | -------- | ------------------ | -------------------------------------- | -------- |
| LR1120MB1DIS | E655V01A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna | 0x0102   |
| LR1120MB1DJS | E656V01A | 868/915 MHz        | GNSS without LNA                       | 0x0102   |

The list of compatible Semtech LR1121 shields is:

| Shield       | PCB      | Frequency matching | Firmware |
| ------------ | -------- | ------------------ | -------- |
| LR1121MB1DIS | E655V01A | 868/915 MHz        | 0x0102   |

#### GNSS LNA Control

There are two ways to control GNSS LNA:

- LR11xx DIO7 controls GNSS LNA: You need to make sure that **R17 populated and R18 NC** on the LR11xx shield board.

- MCU GPIO controls GNSS LNA: You need to make sure that **R17 NC and R18 populated** on the LR11xx shield board. 
  - If you use the E707V02A Adapter Board, you can directly run the code. It has been configured by the macro definition - `LR11XX_E707`. 
  - If you use the BRD8042A CSS/FSK Adapter Board, you must fly wire to connect MCU PB05 with J4-4 as GNSS LNA control pin. And you must remove the macro definition - `LR11XX_E707`.

### Software

- Using the **Gecko SDK - 32-bit and Wireless MCUs** version is v4.4.3
- Using the **Sidewalk** version is v2.0.1
- Silicon Labs Matter 2.2.2
- GCC - GNU ARM v12.2.1
- LR11xx family: [v2.3.0 of SWDR001](https://github.com/Lora-net/SWDR001.git) 

## Steps to use

1. Clone this repository. There is a submodule - `semtech_radio\SWDR001` in it. 

   - If you clone this repository for the first time, you can directly run this command as follow:

     ```c
     git clone git@github.com:Lora-net/SWDR007.git --recursive
     ```

   - If you have cloned this repository, you can run this command in root directory as follow:

     ```c
     git submodule update --init --recursive
     ```

   Using the above command, you will successfully clone all the repositories.

2. There are a few things that need to be checked.

   1. There are four files from GSDK that are modified. So, you need to make sure that ones from this repository are applied and not your local ones. Here are the four files as follow:

      ```c
      swdr007\sidewalk_x.x.x\component\includes\projects\sid\sal\silabs\sid_pal\include\gpio.h
      swdr007\sidewalk_x.x.x\component\includes\projects\sid\sal\silabs\sid_pal\interfaces\platform_init_types\sid_pal_platform_init_types.h
      swdr007\sidewalk_x.x.x\component\sources\projects\sid\sal\silabs\sid_pal\common.c
      swdr007\sidewalk_x.x.x\component\sources\projects\sid\sal\silabs\sid_pal\serial_bus\sid_pal_serial_bus_spi.c
      ```

   2. As we use LR1110 instead of SX1262 in the Hello Neighbor demo, we need to check the files, which are applied by SX1262, as follows, have been excluded from building in your Simplicity Studio. If not, you can set as this: **Select the folder -> Right-click -> Resource Configurations -> Exclude from Build...** .

      ```c
      sidewalk_x.x.x\component\sources\platform\sid_mcu\semtech\hal\sx126x
      sidewalk_x.x.x\component\includes\platform\sid_mcu\semtech\hal\sx126x
      ```

3. This project provides four examples for testing. You must select one and exclude the other three before building project. You can set in Simplicity Studio: **select the example -> right-click -> Resource Configurations -> Exclude from Build...** .

   | Example           | Description                                                  |
   | ----------------- | ------------------------------------------------------------ |
   | example_lr11xx    | LoRa or FSK connectivity                                     |
   | example_wifi      | LoRa or FSK connectivity <br />WIFI scan and uplink to LoRa Locator or AWS Lambda |
   | example_gnss      | LoRa or FSK connectivity <br />GNSS scan and uplink to LoRa Locator or AWS Lambda |
   | example_gnss_wifi | LoRa or FSK connectivity <br />GNSS+WIFI scan and uplink to LoRa Locator or AWS Lambda |

4. Please check the required macro definitions as follows.

   | Macro                   | Description                                                  |
   | ----------------------- | ------------------------------------------------------------ |
   | LR11XX_E707             | If using E707V02A Adapter Board, it must be added. Add default. |
   | FSK_SUPPRESS_RX_TIMEOUT | If in FSK mode, it must be added. If in LoRa mode, add or not add is the same. Add default. |
   | LR1121                  | If using LR1121 chip, it must be added. Not add default.     |

5. Compile and flash the binary.

## GNSS Scan Feature

If you use the GNSS scan feature, you may need to install the full almanac into the LR11xx chip. It doesn't have to be updated every time, but it may need to be updated every few months. The update script - `examples\common_gnss\get_full_almanac.py` will download the latest almanac from LoRa Cloud and write it into `almanac.h`.

**NOTE**: Almanac update is only applicable to LR1110 / LR1120 chips.

### 1. Update Almanac

The full almanac update is executed in three steps:

1. Generate an almanac C header file(`almanac.h`) with the python script *get_full_almanac.py*. This header file needs to be put into the including subdirectory.

   ```python
   $ python ./get_full_almanac.py --help
   usage: get_full_almanac.py [-h] [-f OUTPUT_FILE] mgs_token
   
   Companion software that generates almanac header file to be compiled for LR1110/LR1120 embedded full almanac update.
   
   positional arguments:
     mgs_token             MGS LoRa Cloud token to use to fetch the almanac
   
   optional arguments:
     -h, --help            show this help message and exit
     -f OUTPUT_FILE, --output_file OUTPUT_FILE
                           file that will contain the results
   ```

2. Add the macro `ALMANAC_UPDATE=1` in the Simplicity Studio.

3. Build project and flash to device. It is written after Sidewalk initialization, but prior to Sidewalk starting.

#### Expected Behavior

The successful completion of the full almanac update is indicated by:

```c
<info> Local almanac doesn't match LR11XX almanac -> start update
<info> Almanac update succeeded
```

### 2. Initialization Location

You need to change the latitude and longitude to your own place before you run GNSS examples. Please check the macro definitions in the file `examples\common_gnss\gnss.c`. 

```c
#define ASSIST_LATITUDE xxx.xx
#define ASSIST_LONGITUDE xxx.xx
```

### GNSS Performance Evaluation Notice

The included GNSS example source code is provided solely to demonstrate the GNSS scan functionality under ideal conditions. The source code and GNSS scan results are not representative of the optimal configuration or performance characteristics of the silicon. The LR11xx product family is flexible and can be emobodied and configured in a multitude of ways to realize various trade-offs regarding performance, battery life, PCB size, cost, etc. The GNSS example included in this release and the corresponding evaluation kits are designed & configured by the included source code in a default capacity which is sufficient to demonstrate functional GNSS scan capability only. Care must be taken if/when attempting to assess performance characterstics of GNSS scan functionality and we strongly encourage those conducting such analysis to contact Semtech via the provided support channels so that we can ensure appropriate configuration settings are employed for a given performance evaluation use-case.

## Create Static Library for SWDR001

If you want to create a static library for SWDR001 driver, here are the reference steps.

1. Create a clean project on the Simplicity Studio. Here I advice you can directly copy the *Amazon Sidewalk - SoC Bluetooth Sub-GHz Hello Neighbor* project because it may avoid some configuration incompatibilities between static library and your project you will use it.

2. Add SWDR001. Delete other files but SWDR001 folder. Include the header directories. **Properties -> C/C++ Build -> Settings -> Tool Settings ->GNU ARM C Compiler -> Includes -> Add..** 

   ```
   ${workspace_loc:/${ProjName}/SWDR001/src}
   ```

3. Build artifact. **Properties -> C/C++ Build -> Settings -> Build Artifact -> Artifact Type**. Change *Executable* to *Static Library*. Then, click *Apply and Close*.

4. Build Project. You will find the static library in the *GNU ARM XXX* folder after built.

## Known Issues

### 1. Miss *on_msg_sent* callback occasionally on GFSK mode

From Sidewalk's protocol dictates, with GFSK mode, the end node will normally receive an acknowledgement from gateway after it uses *sid_put_msg()* to send an uplink. And the same packet payload will be retried 3 times until it gets an ACK. Here, there's three outcomes: 

1. The first packet is sent successfully, and the APP is notified via the *on_msg_sent* callback.
2. If all retries fail, the APP is notified via the *on_send_error* callback.
3. After receiving the ACK packet, the APP is notified via the *on_msg_received* callback.

A third scenario is possible while running this demo with GFSK mode. So, the device sometimes missed *on_msg_sent* callback.

### 2. Print error logs while doing GNSS or WIFI scan in FSK mode

While conducting validation test and field trials for this repository, several different error traces were observed. This is because the Sidewalk MAC layer attempts an operation on the LR11xx chip while doing GNSS or WIFI scan in FSK mode. But they don't interfere with work. It may occur some error logs as follows:

```plaintext
<info> rx initiate error -13
<error> BCN:bcn miss:1
<info> FskRx:rx_st:-5
```

## Support

Ask support for Sidewalk on Silicon Labs Community at here:  https://www.silabs.com/support.

Ask support for LR11xx radio at here: https://semtech.my.site.com/ldp/ldp_support.