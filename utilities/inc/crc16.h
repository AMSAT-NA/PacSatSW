/*
 * crc16.h
 *
 *  Created on: Mar 25, 2023
 *      Author: g0kla
 */

#ifndef UTILITIES_INC_CRC16_H_
#define UTILITIES_INC_CRC16_H_

/*
    crc.h

    CCITT CRC routines

    John Melton
    G0ORX, N6LYT

    4 Charlwoods Close
    Copthorne
    West Sussex
    RH10 3QZ
    England

    INTERNET:   g0orx@amsat.org
            n6lyt@amsat.org
            john@images.demon.co.uk
            J.D.Melton@slh0613.icl.wins.co.uk

*/

#define CRCLENGTH 2

int check_crc(unsigned char *buf, int length);
unsigned short crc16(unsigned char *buf, int length);



#endif /* UTILITIES_INC_CRC16_H_ */
