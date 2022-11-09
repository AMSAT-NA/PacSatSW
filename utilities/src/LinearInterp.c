/*
 * LinearInterp.c
 *
 *  Created on: May 21, 2020
 *      Author: bfisher
 */
#include "LinearInterp.h"
uint16_t LinearInterpolate(uint16_t inVal,uint16_t inMax,uint16_t outMax){
	uint32_t inVal32=inVal,outMax32=outMax,inMax32=inMax,retVal;
	retVal = (inVal32*outMax32)/inMax32;
	return (uint16_t)retVal;
}


