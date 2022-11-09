#ifndef KEYFILE_H
#define KEYFILE_H
/*
 * Do not include the secret key in the git repo.  It can be created with the AuthGen program, but only exists
 * on the main build system.
 */

#define ENCRYPTION_KEY_MAGIC_VALUE 0x71539172 /* Random value unlikely to be there by default */

/*
 * This key is for "the LTMTest" satellite.
 */

#define DEFAULT_KEY {0x62,0x17,0xe2,0x14,0x1a,0x5d,0xa0,0xf2,0x05,0xc3,0x25,0xd7,0x59,0x91,0x49,0xbb};
#endif // KEYFILE_H
