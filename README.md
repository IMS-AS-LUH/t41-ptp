![example workflow](https://github.com/IMS-AS-LUH/t41-ptp/actions/workflows/compile-examples.yml/badge.svg)

# t41-ptp
This [repository](https://github.com/IMS-AS-LUH/t41-ptp) provides the source code for our research paper "Sub-Microsecond Time Synchronization for Network-Connected Microcontrollers" ([Final published article](https://doi.org/10.1109/ICCE59016.2024.10444401), [Open Access: accepted version](https://doi.org/10.15488/16561)). This Arduino compatible library has been created to provide high precision time synchronization for the Teensy 4.1 microcontroller platform.

This library was created by the Architectures and Systems Group of the Institute of Microelectronic Systems ([IMS/AS](https://www.ims.uni-hannover.de/de/institut/architekturen-und-systeme/)) at the [Leibniz University](https://www.uni-hannover.de) in Germany.

![Screenshot](/doc/0.png?raw=true)

## Getting Started

### Prerequisites
- Teensy 4.1 with Ethernet adapter
- Working Teensy Arduino environment
- PTP-branch of QNEthernet Library with our patches: https://github.com/HedgeHawk/QNEthernet/tree/ieee1588-2-fix


### Installation
- Clone the library folder to your Arduino-libraries path
- Restart Arduino IDE

### Examples
We provide examples for a Teensy 4.1 to act as PTP Slave and PTP Master 

### Test Setup
A GPS module ([Adafruit Ultimate GPS](https://www.adafruit.com/product/746)) is connected to the PTP Master ([Teensy 4.1](https://www.pjrc.com/store/teensy41.html)) via a PPS signal. The PTP Slave ([Teensy 4.1](https://www.pjrc.com/store/teensy41.html)) is synchronized to the PTP Master via a [network connection](https://www.pjrc.com/store/ethernet_kit.html).
![Setup](/doc/1.jpg?raw=true)

## Citation
If you use our PTP library (or parts of it) in scientific work,
please cite our ICCE 2024 paper ([Final published article](https://doi.org/10.1109/ICCE59016.2024.10444401), [Open Access: accepted version](https://doi.org/10.15488/16561)).

    @inproceedings{schleusner2024sub,
     title={Sub-Microsecond Time Synchronization for Network-Connected Microcontrollers},
     author={Schleusner, Jens and Fahnemann, Christian and Pfleiderer, Richard and Blume, Holger},
     booktitle={2024 IEEE International Conference on Consumer Electronics (ICCE)},
     month=jan,
     year=2024,
     keywords={Microcontrollers;Hardware;Synchronization;Phase locked loops;Standards;Global Positioning System;Clocks;PTP;Precision Time Protocol;Microcontroller;Embedded System;TSN;Time Sensitive Networking},
     doi={10.1109/ICCE59016.2024.10444401}
    }

## License
Distributed under the MIT license. Please see LICENSE for more information.
