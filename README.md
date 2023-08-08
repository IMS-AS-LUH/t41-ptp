# t41-ptp
This repository provides the source code for our research paper "A Bare-Metal Implementation of the
Precision Time Protocol for Microcontrollers". This Arduino compatible library has been created to provide high precision time synchronization for the Teensy 4.1 microcontroller platform.

This tool was created by the Architectures and Systems Group of the Institute of Microelectronic Systems ([IMS/AS](https://www.ims.uni-hannover.de/de/institut/architekturen-und-systeme/)) at the [Leibniz University](https://www.uni-hannover.de) in Germany.

![Screenshot](/doc/0.png?raw=true)

## Getting Started

### Prerequisites
- Teensy 4.1 with Ethernet adapter
- Working Teensy Arduino envirnment 
- PTP-branch of QNEthernet Library with our patches: https://github.com/HedgeHawk/QNEthernet/tree/ieee1588-2-fix


### Installation
- Clone the library folder to your Arduino-libraries path
- Restart Arduino IDE

### Examples
We provide examples for a Teensy 4.1 to act as PTP Slave and PTP Master 

## Citation

If you use our PTP library (or parts of it) in scientific work,
please cite our upcoming paper (**not published yet**).

    @unpublished{schleusner2024bare,
     title={A Bare-Metal Implementation of the Precision Time Protocol for Microcontrollers},
     author={Jens Schleusner, Christian Fahnemann, Richard Pfleiderer, Holger Blume},
    }

The correct precise citation will be updated as soon as possible.

## License

Distributed under the MIT license. Please see LICENSE for more information.
