/*
 * cygpio - A program to turn on and off the USB power to a PacSat board.
 *
 * This program lets you turn on and off GPIO 9 on the CY7C65215 chip
 * on the PacSat board.  This controls the MOSFETs that turn on and
 * off power to the rest of the board from USB.
 *
 * The CY7C65215 must be programmed so that the serial devices are
 * 2-pin and GPIO 9 should be set to be 0 by default.
 *
 * You must run this program as root.
 */

#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

struct {
    uint16_t vendor;
    uint16_t product;
} supported_devices[] = {
    { 0x04b4, 0x0005 },
    {}
};

#define MAX_HANDLES 10

#define CY_VENDOR_REQUEST_DEVICE_TO_HOST 0xc0
#define CY_GPIO_SET_VALUE_CMD 0xdb
#define CY_USB_SERIAL_TIMEOUT 5000

int
main(int argc, char *argv[])
{
    ssize_t devlen;
    int usbrv;
    libusb_device **devs;
    struct libusb_device_descriptor desc;
    ssize_t i, j;
    libusb_device_handle *handle;
    unsigned int count, pos = 0;
    unsigned int value = 0;
    unsigned char buffer[1];
    int rv = 1;

    if (argc < 2) {
	fprintf(stderr, "Not enough arguments, run as:\n");
	fprintf(stderr, "  %s 0\n", argv[0]);
	fprintf(stderr, "to list all compatible USB devices and\n");
	fprintf(stderr, "  %s <dev nr> 0|1\n", argv[0]);
	fprintf(stderr, "to turn off/on the board\n");
	return 1;
    }

    count = strtoul(argv[1], NULL, 0);

    if (count > 0) {
	if (argc < 3) {
	    fprintf(stderr, "Not enough arguments\n");
	    return 1;
	}

	value = strtoul(argv[2], NULL, 0);
	if (value > 0)
	    value = 1;
    }

    usbrv = libusb_init(NULL);
    if (usbrv != LIBUSB_SUCCESS) {
	fprintf(stderr, "Unable init USB library: %s\n",
		libusb_error_name(devlen));
	return 1;
    }

    devlen = libusb_get_device_list(NULL, &devs);
    if (devlen < 0) {
	fprintf(stderr, "Unable to get USB device list: %s\n",
		libusb_error_name(devlen));
	return 1;
    }

    for (i = 0; i < devlen; i++) {
	usbrv = libusb_get_device_descriptor(devs[i], &desc);
	if (usbrv != LIBUSB_SUCCESS)
	    continue;
	for (j = 0; supported_devices[j].vendor != 0; j++) {
	    if (desc.idVendor == supported_devices[j].vendor
			&& desc.idProduct == supported_devices[j].product) {
		pos++;
		if (count == 0) {
		    printf("Device %u: %4.4x %4.4x\n",
			   pos, desc.idVendor, desc.idProduct);
		} else if (pos == count) {
		    goto found;
		}
	    }
	}
    }
    if (count == 0) {
	rv = 0;
	goto out_free;
    }

    fprintf(stderr, "Unable to find the given device\n");
    goto out_free;

 found:
    usbrv = libusb_open(devs[i], &handle);
    if (usbrv != LIBUSB_SUCCESS) {
	fprintf(stderr, "Unable to open USB device: %s\n",
		libusb_error_name(usbrv));
	fprintf(stderr, "You must run this program as root\n");
	goto out_free;
    }

    usbrv = libusb_control_transfer(handle, CY_VENDOR_REQUEST_DEVICE_TO_HOST,
				    CY_GPIO_SET_VALUE_CMD, 9, value,
				    buffer, 0,
				    CY_USB_SERIAL_TIMEOUT);
    if (usbrv != LIBUSB_SUCCESS) {
	fprintf(stderr, "Unable to set the value: %s\n",
		libusb_error_name(usbrv));
    } else {
	rv = 0;
    }

    libusb_close(handle);

 out_free:
    libusb_free_device_list(devs, 1);
    return rv;
}
