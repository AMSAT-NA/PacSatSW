# Boostrap Loader for the PacSat board

This is a serial bootstrap loader that is able to load the processors
on the PacSat boards.  This is primarily here in case the software
needs to be updated after final assembly, since the debug connectors
will not be available then.

This implements a protocol that is built in to the auxiliary processor
(the MPSPM0L1228) as described in
https://www.ti.com/lit/ug/slau887a/slau887a.pdf?ts=1777909805211

The main CPU (the TMS570) does not have this built in, but it does
implement this protocol in bootstrap firmware loaded into FLASH that
runs at the beginning.

Both of these parts have a signal that turns on the bootstrap loader
when the processor powers up.  On version 3 boards this is on the
PC104 connectors, PC104\_GPIO2 for the main CPU and PC104\_GPIO8 for
the aux processor.  Pull these to ground and power on to start the
bootstrap.

On version 4 board this is also wired into the USB chip GPIOs.  See
the ICD.md document for details.

To invoke this, do:
```
bsl <elf file> <serial gensio>
```

where the elf file is the final output file from the build,
PacSatSw.out for the main processor or PacSatSPII2C.out for the
auxiliary processor.

The serial gensio is a description of the serial port of the chip.
These are generally at `/dev/ttyACM<n>`.  The value of `<n>` depends
on your system, but you can generally find it by plugging in the board
and seeing which serial ports appear.  The first new one will be the
main processor, the second will be the aux processor.

The main processor loads in a two-stage process.  The bootloader in
FLASH is only able to load into RAM because you cannot be executing
from FLASH and writing to it at the same time.  So you load another
bootloader that runs in RAM that is able to write to FLASH.

For the main processor, do:
```
bsl PacSatFlashLoader.out sdev,/dev/ttyACM<n>,38400n81
bsl PacSatSw.out sdev,/dev/ttyACM<n>,38400n81
```
The first operation loads the second stage loader and runs it.  The
second loads the actual software.

For the aux processor do:
```
bsl PacSatSPII2C.out sdev,/dev/ttyACM<n>,9600n81
```

The aux processor can only run this one time after power up.  If you
have an issue, you need to power it off and power it back on.  You can
do this with GPIOs on the main processor.
