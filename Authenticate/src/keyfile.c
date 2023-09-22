#include <string.h>
#include "keyfile.h"
#include "nonvol.h"
#include "MRAMmap.h"

/* The default key, overwritten from MRAM at startup. */

uint8_t hmac_sha_key[AUTH_KEY_SIZE] = {
    0x49, 0xc2, 0x90, 0x2e, 0x9d, 0x99, 0x32,
    0xf0, 0x9a, 0x09, 0x32, 0xb9, 0x8c, 0x09,
    0x8e, 0x98, 0xa9, 0x80, 0xd0, 0x98, 0x92,
    0xc8, 0x9e, 0x98, 0xd7, 0x9f, 0x98, 0x7e
};

uint32_t key_checksum(uint8_t *key)
{
    unsigned int i;
    uint32_t checksum = 0;

    for(i=0; i<AUTH_KEY_SIZE; i++)
        checksum += key[i];
    return checksum;
}

void LoadMRAMKey(void)
{
    uint8_t key[AUTH_KEY_SIZE];
    uint32_t magic, checksum;
    const MRAMmap_t *LocalFlash = 0;

    if (!readNV(key, sizeof(key), NVConfigData, (int)&LocalFlash->AuthenticateKey.key)) {
        printf("Unable to load key from nvram\n");
        return;
    }
    if (!readNV(&checksum, sizeof(checksum), NVConfigData,
                (int)&LocalFlash->AuthenticateKey.keyChecksum)) {
        printf("Unable to load key checksum from nvram\n");
        return;
    }
    if (!readNV(&magic, sizeof(magic), NVConfigData,
                (int)&LocalFlash->AuthenticateKey.magic)) {
        printf("Unable to load key magic from nvram\n");
        return;
    }
    if (ENCRYPTION_KEY_MAGIC_VALUE != magic) {
        printf("Key magic mismatch\n");
        return;
    }
    if (checksum != key_checksum(key)) {
        printf("Key checksum mismatch\n");
        return;
    }
    memcpy(hmac_sha_key, key, AUTH_KEY_SIZE);
}
