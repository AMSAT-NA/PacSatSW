# Using Higher Speeds with PacSat

In order to use higher speeds, you need ground station software to be
able to send and transmit at the higher speeds.  You will need an SDR
for this.

You need to get the bleeding edge version of the gensio library from
git at https://github.com/cminyard/gensio

Eventually this will be mainstream, but not yet.

You need to read BUILDING.rst and get all the right libraries
installed, especially soapy, and then build gensio and install it.

Then you need to find your SDR device.  Run `SoapySDRUtil --find` to
find it.  Mine says:

```
Found device 4
  driver = uhd
  label = B210 6M41HXO
  name = LibreSDR_B220mini
  product = B210
  serial = 6M41HXO
  type = b200
```

along with the normal soundcards.  Those key=value pairs are what you
use to specify the SDR to use.  Usually driver is enough.

All the examples below assume you are using the AX5043 convolutional
coding (the "axfec" gensio).  If you are not using that, you will need
to use "hdlc(debug=0x18)" instead of "axfec(debug=0x1f)".  axfec is
strongly recommended.  It cuts the data rate in half, you can double the
data rate and still gain 2dB of performance.

Now run gensio to connect to the SDR and make a kiss interface:

```
gensiot -d -d -a -i 'axfec(debug=0x1f),
    fsk(debug=0x00,bps=19200,freqadj,uncert),
    soapy(rate=2.5M,bandwidth=200K,in_freq=435.7609M,in_channel=0,
          out_freq=145.7802M,out_channel=0,out_gain=100,in_gain=100),driver=uhd' \
  'kiss,tcp,localhost,8100'
```

This sets up a 19200 connection with FEC in both direction at
435.76MHz downlink and 145.78MHz uplink.  Notice I had to tune the
frequencies a little to get them to align properly.  It's not so bad
at 19200, but at 2400 it has to be very close.

Note that the gain setting will depend on your radio.  The rate and/or
bandwidth may need to increase depending on the speeds you are working
at and what your SDR can do.  They can be lower at lower speeds to
reduce CPU used for coding.

For the above, you would set the modulation on the PacSat board to
something like:
```
Pacsat>mod
channel 0: msk19200conv
channel 1: afsk1200
channel 2: afsk1200
channel 3: afsk1200
channel 4: msk19200conv
```
and if you are using hdlc you wouldn't put the "conv" at the end.

Kiss will be running on port 8100.  You will need to set up the
PacSatGround software for that.

On the PacSat you need to set the frequencies properly with the `freq`
command and set the modulation, with the `mod` command, to 19200fec on
the transmit and receive channels you are using.

There is more detailed information about this in the "FSK-SOAPY.rst"
file in the gensio library.

Note that a program needs to be written to tie all this up and make it
easy.

# Different up and down speeds

If you wanted to use a 9600 uplink, you could do:

```
gensiot -d -d -a -i 'axfec(debug=0x1f),
    fsk(debug=0x00,out_bps=9600,in_bps=19200,freqadj,uncert),
    soapy(rate=2.5M,bandwidth=200K,in_freq=435.7609M,in_channel=0,
          out_freq=145.7802M,out_channel=0,out_gain=100,in_gain=100),driver=uhd' \
  'kiss,tcp,localhost,8100'
```

You would, of course, need to set the uplink to msk9600conv with the
`mod` command.

If you wanted to use the 1200 uplink with AFSK, you would need to have
a normal radio hooked in with a signalink, digirig, or something like
that.  This is a lot more complicated.  The gensiot command would be
something like:

```
gensiot -d -d -a -i '
    iosplit(discard,outgen="afskmdm(tx-predelay=500,keytype=rts,
                                    key=\"sdev,/dev/serial/by-path/pci-0000:04:00.3-usb-0:1.1.4.4.1.2:1.0-port0\"),
	                    sound(48000-1-float),plughw:CARD=Device,DEV=0"),
    axfec(debug=0x1f),
    fsk(debug=0x00,in_bps=19200,tx=off,freqadj,uncert),
    soapy(rate=2.5M,bandwidth=200K,freq=435.7609M,in_channel=0,in_gain=100),driver=uhd' \
  'kiss,tcp,localhost,8100'
```

The iosplit splits the input and output into two different gensio
stacks.

The first part (`outgen=`) specifies the 1200bps uplink.  It's using a
specific sound card.  You can use "gsound -L" to list all the sound
cards and find yours.  You want to use the `plughw` one and specify
the card and device.  Then you need to figure out the key stuff.  For
a signalink you can just remove those since it's VOX.  For digirig you
need to find the proper serial port to key the transmitter.  You can
also key with a cm108 chip.  You can do `man 5 gensio` to get the
gensio docs, search for the `fsk` section, then look at `key=` and
`keytype=` under that.

The rest of this is pretty similar to the previous ones, except the
output devices are removed.

# FSK with a modulation index of 2

There is another modulation type you can use that will work better at
lower frequencies.  With MSK the modulation index is 0.5 and the
frequencies are very close to each other, half the data rate apart.
So at 1200bps, you would get one frequency at 600Hz and the other at
1200Hz, for instance.  This makes tuning very critical.

To avoid the issue, you can use the FSK2M modulation type, which will
space the frequencies `2 * data rate` apart.  So in the above example,
the lower frequency would be 1200Hz and the upper one would be 3600Hz.
The wider frequency spacing should also greatly improve the decoding
performance since the frequencies will bleed into each other less.

For the fsk gensio, you add `modidx=2` to get this, and use `fsk2m` as
the modulation type on the board, like:
```
Pacsat>mod
channel 0: fsk2m9600conv
channel 1: afsk1200
channel 2: afsk1200
channel 3: afsk1200
channel 4: fsk2m19200conv
```

# PSK

Phase shift keying (BPSK specifically) is supported by the board.
There is no support in gensio for this, though you might get it
working with GNU radio or something like that.

# AFSK1200 and GMSK9600

These are special modulation settings that are backwards compatible
with these modes.  gensio only has support for AFSK1200 going into a
standard radio, and it does not support GMSK.  You can use direwolf
for this, but unfortunately, it does not support different up and down
speeds.

You can turn on convolutional coding with this.  However, this turns
off all the special settings for these modes (differential, inverting,
and scrambling for gmsk9600) so direwolf won't work for these.
However, gensio will then work at 9600 coming through an SDR, and you
can use a normal radio for this at 1200.

Use of these modes is discouraged.
