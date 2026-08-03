
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <gensio/gensio.h>
#include <gensio/gensio_time.h>

#include "elfc.h"
#include "bsl_host.h"

const char *argv0;

struct gensio_os_funcs *o;
struct gensio *serio;
struct gensio_waiter *write_waiter;
struct gensio_waiter *rsp_waiter;

/*
 * This is the default password.  If we ever want to set one, this
 * will need to be settable somehow.
 */
uint8_t bsl_password[32] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

struct bsl_host bsl_host;

static void
do_vlog(struct gensio_os_funcs *f, enum gensio_log_levels level,
	const char *log, va_list args)
{
    fprintf(stderr, "gensio %s log: ", gensio_log_level_to_str(level));
    vfprintf(stderr, log, args);
    fprintf(stderr, "\n");
}

static void
usage(void)
{
    fprintf(stderr, "Usage: %s <elf file> <serial gensio>\n", argv0);
    fprintf(stderr, "  <elf file> - Build output, generally ends in .out\n");
    fprintf(stderr, "  <serial gensio> - gensio describing the serial port\n");
}

static int
serio_event(struct gensio *io, void *user_data, int event, int err,
	    unsigned char *buf, gensiods *buflen,
	    const char *const *auxdata)
{
    struct coninfo *ci = user_data;
    gensiods len, i;
    int rv;

    switch (event) {
    case GENSIO_EVENT_READ:
	if (err) {
	    gensio_set_read_callback_enable(serio, false);
	    fprintf(stderr, "Error from io: %s\n", gensio_err_to_str(err));
	    return 0;
	}
	bsl_handle_buffer(&bsl_host.p, buf, *buflen);
	return 0;

    case GENSIO_EVENT_WRITE_READY:
	gensio_set_write_callback_enable(serio, false);
	gensio_os_funcs_wake(o, write_waiter);
	return 0;

    default:
	return GE_NOTSUP;
    }
}

static void
serio_data_ready(struct bsl_protocol *p)
{
    bsl_host_check(p->cb_data);
}

static uint8_t last_result;
static uint8_t last_data[BSL_MAX_BUFFER_SIZE];
static unsigned int last_len;

static void
bsl_op_done(struct bsl_host *h, uint8_t result, uint8_t *data, unsigned int len)
{
    last_result = result;
    if (len > sizeof(last_data))
	len = sizeof(last_data);
    last_len = len;
    if (len > 0)
	memcpy(last_data, data, len);
    if (result && len > 0) {
	unsigned int i;

	printf("Got error result:");
	for (i = 0; i < len; i++)
	    printf(" %2.2x", data[i]);
	printf("\n");
    }
    gensio_os_funcs_wake(o, rsp_waiter);
}

static uint8_t
bsl_connect_s(struct bsl_host *h)
{
    int rv;
    unsigned int retries = 5;
    gensio_time timeout;

    while (retries > 0) {
	rv = bsl_connect(h, bsl_op_done);
	if (rv)
	    return rv;

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, rsp_waiter, 1, &timeout);
	if (!rv)
	    return last_result;
	bsl_host_reset(h);
	retries--;
    }

    return BSL_INTERNAL_TIMED_OUT;
}

static uint8_t
bsl_unlock_s(struct bsl_host *h, uint8_t password[32])
{
    int rv;
    unsigned int retries = 5;
    gensio_time timeout;

    while (retries > 0) {
	rv = bsl_unlock(h, password, bsl_op_done);
	if (rv)
	    return rv;

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, rsp_waiter, 1, &timeout);
	if (!rv)
	    return last_result;
	bsl_host_reset(h);
	retries--;
    }

    return BSL_INTERNAL_TIMED_OUT;
}

static uint8_t
bsl_erase_range_s(struct bsl_host *h, uint32_t start_addr, uint32_t end_addr)
{
    int rv;
    unsigned int retries = 5;
    gensio_time timeout;

    while (retries > 0) {
	rv = bsl_erase_range(h, start_addr, end_addr, bsl_op_done);
	if (rv)
	    return rv;

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, rsp_waiter, 1, &timeout);
	if (!rv)
	    return last_result;
	bsl_host_reset(h);
	retries--;
    }

    return BSL_INTERNAL_TIMED_OUT;
}

static uint8_t
bsl_write_data_s(struct bsl_host *h, uint32_t addr, uint8_t *data, uint32_t len)
{
    int rv;
    unsigned int retries = 5;
    gensio_time timeout;

    while (retries > 0) {
	rv = bsl_write_data(h, addr, data, len, bsl_op_done);
	if (rv)
	    return rv;

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, rsp_waiter, 1, &timeout);
	if (!rv)
	    return last_result;
	bsl_host_reset(h);
	retries--;
    }

    return BSL_INTERNAL_TIMED_OUT;
}

static uint8_t
bsl_start_app_s(struct bsl_host *h)
{
    int rv;
    unsigned int retries = 5;
    gensio_time timeout;

    while (retries > 0) {
	rv = bsl_start_app(h, bsl_op_done);
	if (rv)
	    return rv;

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, rsp_waiter, 1, &timeout);
	if (!rv)
	    return last_result;
	bsl_host_reset(h);
	retries--;
    }

    return BSL_INTERNAL_TIMED_OUT;
}

static uint8_t
serio_write(struct bsl_protocol *p, uint8_t *buf, unsigned int len)
{
    int rv;
    gensiods written;
    gensio_time timeout;

    rv = gensio_write(serio, &written, buf, len, NULL);
    if (rv) {
	gensio_set_read_callback_enable(serio, false);
	fprintf(stderr, "Error from io: %s\n", gensio_err_to_str(rv));
	return BSL_INTERNAL_ERR;
    }
    while (written < len) {
	buf += written;
	len -= written;
	gensio_set_write_callback_enable(serio, true);

	timeout.secs = 2;
	timeout.nsecs = 0;
	rv = gensio_os_funcs_wait(o, write_waiter, 1, &timeout);
	if (rv) {
	    gensio_set_write_callback_enable(serio, false);
	    return BSL_INTERNAL_TIMED_OUT;
	}

	rv = gensio_write(serio, &written, buf, len, NULL);
	if (rv) {
	    gensio_set_read_callback_enable(serio, false);
	    fprintf(stderr, "Error from io: %s\n", gensio_err_to_str(rv));
	    return BSL_INTERNAL_ERR;
	}
    }

    return BSL_ACK;
}

static void
do_delay(struct bsl_host *h, unsigned int ms)
{
    gensio_time timeout;
    int rv;

    gensio_msecs_to_time(&timeout, ms);
    rv = gensio_os_funcs_service(o, &timeout);
    while (rv != GE_TIMEDOUT) {
	rv = gensio_os_funcs_service(o, &timeout);
    }
}

int
main(int argc, char *argv[])
{
    struct elfc *e;
    int fd;
    int nphdrs;
    int i;
    int rv;
    gensio_time timeout;
    Elf64_Addr min_erase, max_erase;

    argv0 = argv[0];

    if (argc < 3) {
	fprintf(stderr, "Not enough arguments\n");
	usage();
	return 1;
    }

    e = elfc_alloc();
    if (!e) {
	fprintf(stderr, "Unable to allocate an elf object\n");
	return 1;
    }

    fd = open(argv[1], O_RDONLY);
    if (fd == -1) {
	fprintf(stderr, "Unable to open %s: %s\n", argv[1], strerror(errno));
	return 1;
    }

    if (elfc_open(e, fd) == -1) {
	fprintf(stderr, "Unable to load elf file %s: %s\n", argv[1],
		strerror(elfc_get_errno(e)));
	return 1;
    }

    rv = gensio_alloc_os_funcs(0, &o, 0);
    if (rv) {
	fprintf(stderr, "Could not allocate OS handler: %s\n",
		gensio_err_to_str(rv));
	return 1;
    }
    gensio_os_funcs_set_vlog(o, do_vlog);

    write_waiter = gensio_os_funcs_alloc_waiter(o);
    if (!write_waiter) {
	fprintf(stderr, "Could not waiter, out of memory\n");
	return 1;
    }

    rsp_waiter = gensio_os_funcs_alloc_waiter(o);
    if (!rsp_waiter) {
	fprintf(stderr, "Could not waiter, out of memory\n");
	return 1;
    }

    rv = str_to_gensio(argv[2], o, serio_event, NULL, &serio);
    if (rv) {
	fprintf(stderr, "Could not allocate %s: %s\n", argv[1],
		gensio_err_to_str(rv));
	return 1;
    }

    rv = gensio_open_s(serio);
    if (rv) {
	fprintf(stderr, "Could not open %s: %s\n", argv[1],
		gensio_err_to_str(rv));
	return 1;
    }

    gensio_set_read_callback_enable(serio, true);

    bsl_host_setup(&bsl_host, serio_write, serio_data_ready, do_delay, NULL);

    rv = bsl_connect_s(&bsl_host);
    if (rv) {
	fprintf(stderr, "Could not connect to target: %2.2x\n", rv);
	return 1;
    }
    printf("Connected to target\n");
    printf("Max buffer size = %d\n", bsl_host.max_buffer_size);

    rv = bsl_unlock_s(&bsl_host, bsl_password);
    if (rv) {
	fprintf(stderr, "Could not unlock the target: %2.2x\n", rv);
	return 1;
    }
    printf("Target unlocked\n");

    min_erase = ~0;
    max_erase = 0;

    nphdrs = elfc_get_num_phdrs(e);

    /* First calculate the range and erase everything. */
    for (i = 0; i < nphdrs; i++) {
	GElf_Phdr hdr;
	Elf64_Off end, addr;
	uint8_t buf[1024];

	if (elfc_get_phdr(e, i, &hdr) == -1) {
	    fprintf(stderr, "Unable to get phdr %d: %s\n", i,
		    strerror(elfc_get_errno(e)));
	    return 1;
	}
#if 1
	printf("%2d: %4d %8.8llx %8.8llx %8.8llx %8.8llx %8.8llx %8.8llx %8.8llx\n",
	       i, hdr.p_type,
	       (unsigned long long) hdr.p_flags,
	       (unsigned long long) hdr.p_offset,
	       (unsigned long long) hdr.p_vaddr,
	       (unsigned long long) hdr.p_paddr,
	       (unsigned long long) hdr.p_filesz,
	       (unsigned long long) hdr.p_memsz,
	       (unsigned long long) hdr.p_align);
#endif
	if (hdr.p_type != PT_LOAD || hdr.p_filesz == 0)
	    continue;

	addr = hdr.p_paddr;
	if (addr < min_erase)
	    min_erase = addr;
	end = addr + hdr.p_filesz - 1;
	if (end > max_erase)
	    max_erase = end;
    }

    if (min_erase > max_erase) {
	fprintf(stderr, "No data to load\n");
	return 1;
    }

    printf("Erasing 0x%llx to 0x%llx\n",
	   (unsigned long long) min_erase,
	   (unsigned long long) max_erase);
    rv = bsl_erase_range_s(&bsl_host, min_erase, max_erase);
    if (rv) {
	fprintf(stderr, "Could not erase range: %8.8llx:%8.8llx: %x\n",
		(unsigned long long) min_erase,
		(unsigned long long) max_erase,
		rv);
	return 1;
    }

    for (i = 0; i < nphdrs; i++) {
	GElf_Phdr hdr;
	Elf64_Off j, len, addr;
	uint8_t buf[1024];

	if (elfc_get_phdr(e, i, &hdr) == -1) {
	    fprintf(stderr, "Unable to get phdr %d: %s\n", i,
		    strerror(elfc_get_errno(e)));
	    return 1;
	}
	if (hdr.p_type != PT_LOAD)
	    continue;
	addr = hdr.p_paddr;
	printf("Writing 0x%llx byte segment\n",
	       (unsigned long long) hdr.p_filesz);
	for (j = 0; j < hdr.p_filesz; j += sizeof(buf), addr += sizeof(buf)) {
	    len = sizeof(buf);
	    if (len > hdr.p_filesz - j)
		len = hdr.p_filesz - j;

	    memset(buf, 0, sizeof(buf));
	    if (elfc_phdr_read(e, i, j, buf, len) == -1) {
		fprintf(stderr, "Unable to read phdr %d at %llx: %s\n", i,
			(unsigned long long) j,
			strerror(elfc_get_errno(e)));
		return 1;
	    }
	    
	    printf("\rWriting at 0x%llx", (unsigned long long) addr);
	    fflush(stdout);
	    rv = bsl_write_data_s(&bsl_host, addr, buf, len);
	    if (rv) {
		fprintf(stderr, "Could not write at: %8.8llx: %x\n",
			(unsigned long long) addr, rv);
		return 1;
	    }
	}
	printf("\r");
	fflush(stdout);
    }

    rv = bsl_start_app_s(&bsl_host);
    if (rv)
	fprintf(stderr, "Error starting app: %x\n", rv);

    elfc_free(e);
    close(fd);
    return 0;
}
