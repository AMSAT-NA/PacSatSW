#ifndef KEYFILE_H
#define KEYFILE_H
/*
 * Do not include the secret key in the git repo.  It can be created with the AuthGen program, but only exists
 * on the main build system.
 */

#include <stdint.h>

#define ENCRYPTION_KEY_MAGIC_VALUE 0x71539172 /* Random value unlikely to be there by default */

#define AUTH_KEY_SIZE 32

extern uint8_t hmac_sha_key[AUTH_KEY_SIZE];

uint32_t key_checksum(uint8_t *key);

void LoadMRAMKey(void);

#endif // KEYFILE_H
