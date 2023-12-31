![example workflow](https://github.com/IMS-AS-LUH/t41-ptp/actions/workflows/compile-examples.yml/badge.svg)

# t41-ptp
This [repository](https://github.com/IMS-AS-LUH/t41-ptp) provides the source code for our research paper "A Bare-Metal Implementation of the
Precision Time Protocol for Microcontrollers". This Arduino compatible library has been created to provide high precision time synchronization for the Teensy 4.1 microcontroller platform.

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
please cite our upcoming paper (**not published yet**).

    @unpublished{schleusner2024sub,
     title={Sub-Microsecond Time Synchronization for Network-Connected Microcontrollers},
     author={Jens Schleusner and Christian Fahnemann and Richard Pfleiderer and Holger Blume},
     booktitle="2024 IEEE International Conference on Consumer Electronics (ICCE) (2024 ICCE)",
     month=jan,
     year=2024,
    }

The correct precise citation will be updated as soon as possible.

## License
Distributed under the MIT license. Please see LICENSE for more information.
