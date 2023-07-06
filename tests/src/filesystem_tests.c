/*
 * filesystem_tests.c
 *
 *  Created on: Jul 6, 2023
 *      Author: cminyard
 */

#include <stdarg.h>
#include "pacsat.h"
#include "redposix.h"
#include "filesystem_tests.h"

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
get_fs_max_files(uint32_t *rfiles)
{
    REDSTATFS stats;

    if (red_statvfs("/", &stats) == 1) {
        printf("Unable to stat filesystem: %s\n", red_strerror(red_errno));
        return -1;
    }
    *rfiles = stats.f_files - 1; /* The '/' always takes a single file */
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

#define EXPECT_OPEN_ERR (1 << 0)
#define EXPECT_WRITE_ERR (1 << 1)
#define EXPECT_DELETE_ERR (1 << 2)

static int
createfile(int expect_err, uint32_t size, const char *fmt, ...)
{
    va_list ap;
    char fname[50];
    int32_t fd;
    uint32_t i;

    va_start(ap, fmt);
    if (vsnprintf(fname, sizeof(fname), fmt, ap) >= sizeof(fname)) {
        printf("createfile: Filename is too long, format is '%s'\n", fmt);
        return -1;
    }
    va_end(ap);

    fd = red_open(fname, RED_O_RDWR | RED_O_CREAT | RED_O_EXCL);
    if (fd == -1) {
        if (!(expect_err & EXPECT_OPEN_ERR)) {
            printf("createfile: Unable to create file '%s': %s\n", fname, red_strerror(red_errno));
            return -1;
        }
        return 0;
    }

    for (i = size; i > 0; ) {
        uint32_t wr_count = sizeof(fname);
        int32_t rc;

        if (wr_count > size)
            wr_count = size;
        rc = red_write(fd, fname, wr_count);
        if (rc == -1) {
            if (!(expect_err & EXPECT_WRITE_ERR))
                printf("createfile: Unable to write file '%s': %s\n", fname, red_strerror(red_errno));
            red_close(fd);
            red_unlink(fname);
            if (!(expect_err & EXPECT_WRITE_ERR))
                return -1;
            return 0;
        }
        i -= rc;
    }
    red_close(fd);
    if (expect_err) {
        printf("createfile: Did not get expected error %u on file %s\n", expect_err, fname);
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

static uint32_t start_blocks, max_files;

static int
test_makefiles(void)
{
    uint32_t end_blocks, end_files, i, j;

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
    if (get_fs_free_blocks(&start_blocks) == -1)
        return;
    if (get_fs_max_files(&max_files) == -1)
        return;
    printf("%u blocks free\n", start_blocks);
    printf("%u available files\n", max_files);

    if (test_makefiles() == -1)
        goto out;

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
