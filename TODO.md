Things that need to be done
===========================

# Pending

## Write a CAN bus driver

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

## Create telemetry interfaces

## Allow PA output to be controlled from the ground station

## Telemetry

## RX LED handling needs some work

LED2 is turned off by starting RX and is disabled on each received
packet, but is never re-enabled.  Probably not optimal.

# Done

## Convert to thermsistor temperature measurement
