Software for AMSAT PacSat project

## About
This repository contains flight software for the PacSat project.  The
documentation is under companion Repository "PacSatDocs.  The code is
set up as a TI CCS (Eclipse) project.  You should be able to clone it
into your own copy of CCS.

## Setup

### Connecting to the launchpad
You connect the TI launchpad board with a USB cable.  No other
hardware is needed.  Once you have installed TIs CCS it should be
recognized.

Downloaded version 12.8.1 of TI CCS from Texas Instruments website.
You can downloaded the "on-demand" version.  Extracted from the file.
On linux you can pick an installation folder in ~/ti, on Windows
probably use the default.

Run the installer.  It may say some dependancies were missing, though. This is likely easier on Windows.
On Linux, if necessary you can install missing dependancies with 
```sudo apt serach <name> ```
To find the package that contained the dependancy.  You may have to omit the .so extension.
Then use 
```sudo apt install ```
to add it.
e.g. for one lib it was this:
```
sudo apt search libusb-0.1
Sorting... Done
Full Text Search... Done
libusb-0.1-4/jammy 2:0.1.12-32build3 amd64
  userspace USB programming library

sudo apt install libusb-0.1-4
```
Then go back then forward in the installer to rerun the dependency check until all were met.

Then custom installation
There is a list of devices.  You can click on them to see what each is.  The TMS570 is under 
![image](https://github.com/user-attachments/assets/bccb2661-04c3-45c5-94a7-e4b7a36ef4db)


If you don't have any special Debug probes (and you likely don't), then ignored the next screen

Then it downloads about 850MB..

On windows this may not be needed, On Liunux then run the install_drivers.sh file with sudo

On first run of CCS it asks for the workspace.  Pick a suitable folder, but if you have another version of Eclipse installed then pick a new workspace folder. The default is probablly OK.

Run this to copy the code to a folder outside of the workspace:
 git clone https://github.com/AMSAT-NA/PacSatSW.git
(If you try it inside the workspace you get an error message when you try to import).

The "getting started" page is shown at first launch of CCS. It has import project as an option.  I used that to select the PacSatSW folder and selected "Copy into workspace". It creates a project in the workspace.  (Which is why it would not work if the folder is already in the workspace, because the name did not match).

Then select the main.c file and it should open fine.
With Project > Build All it should compile, like this:
![image](https://github.com/user-attachments/assets/38456a9c-2e47-479d-8771-ab0a697b44f3)


# Connecting to the PacSat booster board
If you have the Pacsat board connected to the launchpad then you will need a serial connection to read debug information.  You can use a USB Serial adapter as long as it uses 3.3V levels.  That is almost all of them.  If it mentions 5V that is almost always a seperate pass though voltage and not the TTL levels.  But check to make sure. Make up a cable to attach to connector J3 on the purple board.  The pins are marked on the PCB.  It is still 38k4 baud.

### Connect the Launchpad board
The laucnhpad itself connects with a USB.  With it plugged in, In CCS press the "Flash" button:
It does an initial firmware update for a probe then loads the code.

On windows you can find the serial device from the Device Manager.
On linux, to confirm where the serial connection is run this:
```ls /dev > with
Then I unplugged the serial connection from the linux box and ran
ls /dev > without
```
Then diff with without gives:
```22d21
< gpiochip0
206d204
< ttyUSB0
```
So it is ttyUSB0

### Serial Terminal
On windows use Putty or something similar and configure for 38K4 baud.

On Linux use your favorite terminal emulator or:
sudo apt-get install minicom

Run minicom –s to enter setup mode and configure the ttyUSB0 serial port for 38k4
Note that hardware flow control MUST BE OFF or it just wont work!





## Credits
Bill Reed (originator)
Authors:
Burns Fisher, Chris Thompson, Corey Minyard, Jim McCullers, Bob Stricklin

Base OS and source code thanks to:
Burns Fisher, Rich Gopstein, Zach Metzinger, Jonathan Brandenberg and many others from the AMSAT Golf project and the old ASCENT team

## Category

## Tags

