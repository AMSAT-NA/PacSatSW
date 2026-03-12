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

Downloaded version 12.8.1 of TI CCS from Texas Instruments website.  You may be tempted by the newer v20, but it does not support the Hercules processor that we are using.  It is apparently possible to make it work, but it is easier to use the old version, perhaps.

You can downloaded the "on-demand" version or full versions.  You might have more luck with the full version on newer versions of Linux.
On linux you can pick an installation folder in ~/ti or ~/bin/ti, on Windows
probably use the default.

Look at the README and then Run the installer.  It will say some dependancies were missing, though. This is likely easier on Windows.
On older Linux, such as Ubuntu v22, you can install missing dependancies relatively easily with:
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

However, on modern Linux, like Ubuntu v24, many of these dependancies have been depreciated.  Your first roadblock is a requirement for Python 2.7.  You can install the old python with:
```
echo "deb http://archive.ubuntu.com/ubuntu jammy universe" | sudo tee /etc/apt/sources.list.d/jammy-universe.list
sudo apt update
```
Then you can install the following:
```
sudo apt install libpython2.7
sudo apt install libgconf-2-4
```
And then remove this old repo like this:
```
sudo rm /etc/apt/sources.list.d/jammy-universe.list
sudo apt update
```

This one works as is:
```
sudo apt install libusb-0.1-4
```

You may need to pull down these directly like this:
```
wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb
sudo dpkg -i libtinfo5_6.3-2ubuntu0.1_amd64.deb
wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libncurses5_6.3-2ubuntu0.1_amd64.deb
sudo dpkg -i libncurses5_6.3-2ubuntu0.1_amd64.deb
```
And this one needs to be installed like this:
```
sudo dpkg --add-architecture i386
sudo apt update
sudo apt install libc6-i386 lib32stdc++6 lib32z1
```

Then, once you can progress past the dependancies, pick custom installation.

There is a list of devices.  You can click on them to see what each is.  The TMS570 is under 
![image](https://github.com/user-attachments/assets/bccb2661-04c3-45c5-94a7-e4b7a36ef4db)


If you don't have any special Debug probes (and you likely don't), then ignored the next screen

Then, if you are using the ondemand version it downloads about 850MB.  Otherwise it just gets on with the install.

On Linux you must then run the install_drivers.sh file with sudo.  It is in ~/ti/ccs1281/ccs/install_scripts

On windows, you may need to install a TI driver for your debug probe, depending on what it is.

On first run of CCS it asks for the workspace.  Pick a suitable folder, but if you have another version of Eclipse installed then pick a new workspace folder. The default is probablly OK.

Run this to copy the code to a folder outside of the workspace:
 git clone https://github.com/AMSAT-NA/PacSatSW.git
(If you try it inside the workspace you get an error message when you try to import).

The "getting started" page is shown at first launch of CCS. It has import project as an option.  I used that to select the PacSatSW folder and selected "Copy into workspace". It creates a project in the workspace.  (Which is why it would not work if the folder is already in the workspace, because the name did not match).

Then select the main.c file and it should open fine.
With Project > Build All it should compile, like this:
![image](https://github.com/user-attachments/assets/38456a9c-2e47-479d-8771-ab0a697b44f3)

# Connecting to the V2 Pacsat Board
The easiest probe to use is an XDS110, which also provides a serial connection.  Connect to the V2 board with a ribbon cable.  In Linux the TI USB devices are installed as /dev/ttyACM0 and /dev/ttyACM1.  ACM0 is the serial terminal on my system and I can connect to it with:
```
minicom -D /dev/ttyACM0 -b 38400
```
Hitting return will show you a pacsat> prompt.  Type help or helpall to get the commands

If the code in CCS compiled then you should be able to flash the board using the ICON on the toolbar.

# Connecting to the PacSat booster board (Depreciated)
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

