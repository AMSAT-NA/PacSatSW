
CFLAGS = -g -I /usr/local/include

all: bsl

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

bsl: bsl.o elfc.o bsl_host.o bsl_protocol.o crc32.o
	gcc -o bsl $(CFLAGS) $^ -lgensio -lgensioosh

bsl.o: bsl.c bsl_host.h bsl_protocol.h

elfc.o: elfc.c elfc.h btree.h

bsl_host.o: bsl_host.c bsl_host.h bsl_protocol.h

bsl_protocol.o: bsl_protocol.c bsl_protocol.h

crc32.o: crc32.c crc32.h

clean:
	rm -f bsl.o elfc.o bsl_host.o bsl_protocol.o crc32.o bsl
