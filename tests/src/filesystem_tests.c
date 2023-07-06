/*
 * filesystem_tests.c
 *
 *  Created on: Jul 6, 2023
 *      Author: cminyard
 */

#include "pacsat.h"
#include "redposix.h"
#include "filesystem_tests.h"

void
filesystemTest(void)
{

    if (red_format("/") == -1) {
        printf("Unable to format filesystem: %s\n",
               red_strerror(red_errno));
        return;
    }
    printf("Filesystem formatted\n");


}
