# Introduction
This repository contains code for developing and controlling robots that are based on Raspberry Pi. In particular, this code currently assumes the use of Raspberry Pi v3. This set of code has been architected to be used as a package within Python3.

# Installation
Before installing, you must have python3 and pip3 installed and properly configuredon on a Raspberry Pi. This instruction assumes that you have Raspberry Pi v3 running Raspbian OS. It assumes that internet is available on the Rasberry Pi and you should have the following already installed and that you are logged inot the Raspberry Pi as the admin, generally the user `pi`:
* Python 3
* `pip` Python package
* `picamera` Python package
* `smbus` Python package
* `Adafruit_Python_DHT` Python package
For instructions on installing the requirements, refer to the Installation Instruction for the Requirements section.

Once all the required software has been installed, follow the instruction below to install the `rpibotics` Python package:
1. In the desired directory, checkout this repository: `$ git clone https://github.com/tng-spacecamp/rpibotics.git`
2. Change into the directory where the repository has been checkout: `$ cd rpibotics`
3. Install: `sudo pip3 install .`

# Uninstalling
```$ sudo pip3 uninstall rpibotics```

# Installation Instructions for the Requirements
This instruction assumes that you have Raspberry Pi v3 running Raspbian OS. It also assume that internet is available on the Rasberry Pi.

## Installing `Adafruit_Python_DHT`
```
$ git clone https://github.com/adafruit/Adafruit_Python_DHT.git
$ cd Adafruit_Python_DHT
$ sudo python3 setup.py install
```

## Installing `pip`
```
$ sudo apt-get install python3-pip
```

## Installing `picamera`
```
$ sudo apt-get install python3-picamera
```

## Installing `smbus`
```
$ sudo apt-get install python3-smbus
```
