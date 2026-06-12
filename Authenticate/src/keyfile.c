#include <string.h>
#include "keyfile.h"
#include "nonvol.h"
#include "MRAMmap.h"

/* The default key, overwritten from MRAM at startup. */

uint8_t hmac_sha_key[AUTH_KEY_SIZE] = {
    0x74, 0xad, 0x32, 0x0c, 0x1e, 0x07, 0x92, 0xd5,
    0x2e, 0xcc, 0x37, 0xb9, 0xe6, 0xf9, 0xf0, 0x13,
    0x39, 0xbd, 0xdd, 0x1b, 0xb2, 0x99, 0x0a, 0xee,
    0x96, 0x4e, 0x39, 0x62, 0x62, 0x77, 0xd1, 0x68
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
