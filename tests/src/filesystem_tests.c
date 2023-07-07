/*
 * filesystem_tests.c
 *
 *  Created on: Jul 6, 2023
 *      Author: Corey Minyard
 */

#include <stdarg.h>
#include "pacsat.h"
#include "redposix.h"
#include "filesystem_tests.h"

static uint32_t start_blocks, max_files, blocksize;

static int
fs_data_init(void)
{
    REDSTATFS stats;

    if (red_statvfs("/", &stats) == 1) {
        printf("Unable to stat filesystem: %s\n", red_strerror(red_errno));
        return -1;
    }
    blocksize = stats.f_bsize;
    start_blocks = stats.f_bfree;
    max_files = stats.f_files - 1;
    return 0;
}

static int
get_fs_free_blocks(uint32_t *rblocks)
{
    REDSTATFS stats;

    if (red_statvfs("/", &stats) == 1) {
        printf("Unable to stat filesystem: %s\n", red_strerror(red_errno));
        return -1;
    }
    *rblocks = stats.f_bfree;
    return 0;
}

static int
get_fs_free_files(uint32_t *rfiles)
{
    REDSTATFS stats;

    if (red_statvfs("/", &stats) == 1) {
        printf("Unable to stat filesystem: %s\n", red_strerror(red_errno));
        return -1;
    }
    *rfiles = stats.f_ffree;
    return 0;
}

/*
 * Make sure the filesystem free space has been returned to its
 * original value.
 */
static int
test_end_check(void)
{
    uint32_t end_blocks, end_files;

    if (get_fs_free_blocks(&end_blocks) == -1)
        return -1;
    if (get_fs_free_files(&end_files) == -1)
        return -1;
    if (start_blocks != end_blocks) {
        printf("Filesystem blocks not same after file creation test.\n");
        printf("  started at %u, ended at %d\n", start_blocks, end_blocks);
        return -1;
    }
    if (max_files != end_files) {
        printf("Filesystem files not same after file creation test.\n");
        printf("  started at %u, ended at %d\n", max_files, end_files);
        return -1;
    }
    return 0;
}

/*
 * The routines to manipulate files can be told to expect certain errors.
 */
#define EXPECT_OPEN_ERR (1 << 0)
#define EXPECT_WRITE_ERR (1 << 1)
#define EXPECT_DELETE_ERR (1 << 2)
#define EXPECT_READ_ERR (1 << 3)
#define EXPECT_NOSPACE_ERR (1 << 4)

/*
 * Create a file using the fmt as the file name.  Write data to the file
 * consisting of the filename over and over again until you write size
 * bytes.
 */
static int32_t
createfile(int expect_err, uint32_t size, const char *fmt, ...)
{
    va_list ap;
    char fname[50];
    int32_t fd;
    uint32_t i, fname_size;
    uint32_t blks;

    va_start(ap, fmt);
    fname_size = vsnprintf(fname, sizeof(fname), fmt, ap);
    if (fname_size >= sizeof(fname)) {
        printf("createfile: Filename is too long, format is '%s'\n", fmt);
        return -1;
    }
    va_end(ap);

    fd = red_open(fname, RED_O_RDWR | RED_O_CREAT | RED_O_EXCL);
    if (fd == -1) {
        if (!(expect_err & EXPECT_OPEN_ERR)) {
            printf("createfile: Unable to create file '%s': %s\n",
                   fname, red_strerror(red_errno));
            return -1;
        }
        return 0;
    }

    for (i = size; i > 0; ) {
        uint32_t wr_count = fname_size;
        int32_t rc;

        if (wr_count > i)
            wr_count = i;
        get_fs_free_blocks(&blks);
        rc = red_write(fd, fname, wr_count);
        if (rc == -1) {
            if (expect_err & EXPECT_NOSPACE_ERR && red_errno == RED_ENOSPC) {
                red_close(fd);
                return size - i;
            }
            if (!(expect_err & EXPECT_WRITE_ERR))
                printf("createfile: Unable to write file '%s': %s\n",
                       fname, red_strerror(red_errno));
            red_close(fd);
            red_unlink(fname);
            if (!(expect_err & EXPECT_WRITE_ERR))
                return -1;
            return 0;
        }
        if (rc > wr_count) {
            printf("createfile: "
                   "Write return too much on file '%s': got %d, expected %u\n",
                   fname, rc, wr_count);
            return -1;
        }
        i -= rc;
    }
    red_close(fd);
    if (expect_err) {
        printf("createfile: Did not get expected error %u on file %s\n",
               expect_err, fname);
        return -1;
    }
    return 0;
}

/*
 * Check that the file matches what was created by createfile()
 */
static int
checkfile(int expect_err, uint32_t size, const char *fmt, ...)
{
    va_list ap;
    char fname[50];
    char data[50];
    int32_t fd, rc;
    uint32_t i, fname_size;

    va_start(ap, fmt);
    fname_size = vsnprintf(fname, sizeof(fname), fmt, ap);
    if (fname_size >= sizeof(fname)) {
        printf("checkfile: Filename is too long, format is '%s'\n", fmt);
        return -1;
    }
    va_end(ap);

    fd = red_open(fname, RED_O_RDONLY);
    if (fd == -1) {
        if (!(expect_err & EXPECT_OPEN_ERR)) {
            printf("checkfile: Unable to create file '%s': %s\n",
                   fname, red_strerror(red_errno));
            return -1;
        }
        return 0;
    }

    for (i = size; i > 0; ) {
        uint32_t rd_count = fname_size;

        if (rd_count > i)
            rd_count = i;
        rc = red_read(fd, data, rd_count);
        if (rc == -1) {
            if (!(expect_err & EXPECT_READ_ERR))
                printf("checkfile: Unable to write file '%s': %s\n",
                       fname, red_strerror(red_errno));
            red_close(fd);
            red_unlink(fname);
            if (!(expect_err & EXPECT_READ_ERR))
                return -1;
            return 0;
        }
        if (rc != rd_count) {
            red_close(fd);
            printf("checkfile: read mismatch on file %s, read %d, expected %u\n",
                   fname, rc, rd_count);
            return -1;
        }
        if (memcmp(fname, data, rd_count) != 0) {
            red_close(fd);
            printf("checkfile: data mismatch on file %s\n", fname);
            return -1;
        }
        i -= rc;
    }
    rc = red_read(fd, data, sizeof(data));
    if (rc != 0) {
        red_close(fd);
        printf("checkfile: extra data at end of file %s: %d\n", fname, rc);
        return -1;
    }
    red_close(fd);
    if (expect_err) {
        printf("checkfile: Did not get expected error %u on file %s\n",
               expect_err, fname);
        return -1;
    }
    return 0;
}


static int
deletefile(int expect_err, const char *fmt, ...)
{
    va_list ap;
    char fname[50];

    va_start(ap, fmt);
    if (vsnprintf(fname, sizeof(fname), fmt, ap) >= sizeof(fname)) {
        printf("deletefile: Filename is too long, format is '%s'\n", fmt);
        return -1;
    }
    va_end(ap);
    if (red_unlink(fname) == -1) {
        if (!(expect_err & EXPECT_DELETE_ERR)) {
            printf("deletefile: Unable to remove file '%s': %s\n", fname, red_strerror(red_errno));
            return -1;
        }
        return 0;
    }
    if (expect_err) {
        printf("deletefile: Did not get expected error %u on file %s\n", expect_err, fname);
        return -1;
    }
    return 0;
}

/*
 * Create files up to the limit of the maximum number of files.
 */
static int
test_makefiles(void)
{
    uint32_t i, j;

    for (i = 0; i < max_files; i++) {
        if (createfile(0, 0, "//%u", i) == -1)
            return -1;
        if (createfile(EXPECT_OPEN_ERR, 0, "//%u", i) == -1)
            return -1;
    }
    if (createfile(EXPECT_OPEN_ERR, 0, "//%u", i) == -1)
        return -1;
    printf("Created %u files\n", i);

    for (j = 0; j < i; j++) {
        if (deletefile(0, "//%u", j) != 0)
            return -1;
        if (deletefile(EXPECT_DELETE_ERR, "//%u", j) != 0)
            return -1;
    }
    printf("Deleted %u files\n", j);

    if (test_end_check() == -1)
        return -1;
    return 0;
}

/*
 * Create files up to the limit and put some data in them this time.
 */
static int
test_filedata(void)
{
    uint32_t filesize;
    uint32_t i, j;

    /*
     * Try to use up as much filesystem space as possible.  This assumes
     * there are a plenty of blocks for each possible file.
     */
    filesize = start_blocks / max_files;
    if (start_blocks % max_files < max_files / 2)
        filesize--;
    filesize *= blocksize;

    for (i = 0; i < max_files; i++) {
        if (createfile(0, filesize, "//%u", i) == -1)
            return -1;
        if (createfile(EXPECT_OPEN_ERR, 0, "//%u", i) == -1)
            return -1;
    }
    if (createfile(EXPECT_OPEN_ERR, 0, "//%u", i) == -1)
        return -1;
    printf("Created %u files\n", i);

    for (j = 0; j < i; j++) {
        if (checkfile(0, filesize, "//%u", j) != 0)
            return -1;
    }

    for (j = 0; j < i; j++) {
        if (deletefile(0, "//%u", j) != 0)
            return -1;
        if (deletefile(EXPECT_DELETE_ERR, "//%u", j) != 0)
            return -1;
    }
    printf("Deleted %u files\n", j);

    if (test_end_check() == -1)
        return -1;
    return 0;
}

/*
 * Create a file and write as much data as possible into it.
 *
 * You can't use all the blocks for data, when a file exceeds a
 * certain size it has to use some blocks as pointer tables to other
 * blocks, so you have to take into account some overhead.
 */
static int
test_maxfilesize(void)
{
    uint32_t maxsize = blocksize * start_blocks;
    uint32_t max_overhead = maxsize * 5 / 100; /* ~ 5% */
    int32_t size;

    size = createfile(EXPECT_NOSPACE_ERR, maxsize, "//testfile1");
    if (size == -1)
        return -1;
    printf("Wrote %d bytes to file of %u max\n", size, maxsize);
    if (size > maxsize || size + max_overhead < maxsize) {
        printf("Invalid file sizes, either size > max or overhead of %d exceeded\n",
               max_overhead);
        return -1;
    }
    if (checkfile(0, size, "//testfile1") == -1)
        return -1;
    if (deletefile(0, "//testfile1") == -1)
        return -1;

    if (test_end_check() == -1)
        return -1;
    return 0;
}

void
filesystemTest(void)
{
    if (red_format("/") == -1) {
        printf("Unable to format filesystem: %s\n",
               red_strerror(red_errno));
        return;
    }
    printf("Filesystem formatted\n");
    if (red_mount("/") == -1) {
        printf("Unable to mount filesystem: %s\n",
               red_strerror(red_errno));
        return;
    }
    if (fs_data_init() == -1)
        return;
    printf("%u blocks free\n", start_blocks);
    printf("%u available files\n", max_files);

    printf("\nTest making up to the limit of files.\n");
    if (test_makefiles() == -1)
        goto out;

    printf("\nTest files with data.\n");
    if (test_filedata() == -1)
        goto out;

    printf("\nTesting maximum file size.\n");
    if (test_maxfilesize() == -1)
        goto out;

    printf("\n***SUCCESS!**\n");
 out:
    /* Clean up the filesystem by reformatting again. */
    if (red_umount("/") == -1) {
        printf("Unable to unmount filesystem: %s\n",
               red_strerror(red_errno));
        return;
    }
    if (red_format("/") == -1) {
        printf("Unable to format filesystem: %s\n",
               red_strerror(red_errno));
        return;
    }
    printf("Filesystem reformatted for use\n");
}
