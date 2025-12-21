Things that need to be done
===========================

# Pending

## Write code to put the CAN bus messages someplace

The base CAN bus driver is written; it will call a function when a CAN
bus message comes in, and can send CAN bus messages.  It does not do
the file I/O as described below.  For active/standby, CAN bus messages
will be used to keep the MRAMs in sync between two board.  So the
below is not valid for those messages.

From Chris:

Oh, and on stuff to do, yes, the CAN bus driver is the most important.
It should be a new task and it should write to a file based on the CAN
id info.  So multiple sources can send us information and some scheme
will determine which file they go in based on the bits in their can
id.

Files get written in a queue folder ending in .tmp and at some point,
based on size or the time, they are renamed and the .tmp is removed.
Then they are automatically added to the pacsat directory by a
background process.

With that being said, I cant remember if the automatic adding of files
was fully implemented.  I worked on it, but I can't remember it's
state.  I would have to check the commits or branches.

## For board version 3, fix the ACTIVE line

The ACTIVE_N line on board version 2 was changed to ACTIVE due to an
issue with the way the CPU pull lines at reset.  So the logic needs to
be positive on board 3.

## Create telemetry interfaces

## Allow PA output to be controlled from the ground station

## Use the time_valid

There is a time_valid bool to know if the Unix time is set properly,
but it's not used for anything but logging.  It probably needs to be
used for something useful.

## Read hardware watchdog reboot flag from the RTC

Only on board version 3 and greater.

## Telemetry

## Commands (from the ground or CAN bus)

## RX LED handling needs some work

LED2 is turned off by starting RX and is disabled on each received
packet, but is never re-enabled.  Probably not optimal.

# Add missing PC104 GPIOs

# Done

## Convert to thermsistor temperature measurement

## Base CAN bus driver written
