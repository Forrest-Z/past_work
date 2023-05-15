#include "EQYC_UCCAN.h"
#include <inttypes.h>

#include "rtwtypes.h"

#include "math.h"

#include "float.h"

#define UNUSED(X) ((void)(X))

#define UNUSED(X) ((void)(X))

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
boolean_T rtIsInf(real_T value)
{
	return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}
boolean_T rtIsNaN(real_T value)
{
	return (boolean_T)((value!=value) ? 1U : 0U);
}
boolean_T rtIsNaNF(real32_T value)
{
	return (boolean_T)(((value!=value) ? 1U : 0U));
}
real_T rt_roundd_snf(real_T u)
{
	real_T y;
	if (fabs(u) < 4.5035996273705E+15) {
		y = u >= 0.5 ? floor(u + 0.5) : u > -0.5 ? u * 0.0 : ceil(u - 0.5);
	} else {
		y = u;
	}
	return y;
}
real_T rt_modd_snf(real_T u0, real_T u1)
{
	real_T y;
	boolean_T y_0;
	real_T u;
	boolean_T y_1;
	if (u1 == 0.0) {
		y = u0;
	} else {
		u = u0;
		y_0 = ((!rtIsNaN(u)) && (!rtIsInf(u)));
		u = u1;
		y_1 = ((!rtIsNaN(u)) && (!rtIsInf(u)));
		if (!(y_0 && y_1)) {
			y = (rtNaN);
		} else {
			u = u0 / u1;
			if (u1 <= floor(u1)) {
				y = u0 - floor(u) * u1;
			} else if (fabs(u - rt_roundd_snf(u)) <= DBL_EPSILON * fabs(u)) {
				y = 0.0;
			} else {
				y = (u - floor(u)) * u1;
			}
		}
	}
	return y;
}
static inline uint64_t reverse_byte_order(uint64_t x)
{
	//x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
	//x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
	//x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
	return x;
}


int unpack_message(unsigned id, uint64_t data, uint8_t dlc)
{
	switch(id) {
	case 0x501: return unpack_can_0x501_UCCAN_501(&can_0x501_UCCAN_501_data, data, dlc);
	case 0x504: return unpack_can_0x504_IMCU_504(&can_0x504_IMCU_504_data, data, dlc);
	case 0x505: return unpack_can_0x505_IMCU_505(&can_0x505_IMCU_505_data, data, dlc);
	default: break; 
	}
	return -1; 
}

int pack_message(unsigned id, uint64_t *data)
{
	switch(id) {
	case 0x501: return pack_can_0x501_UCCAN_501(&can_0x501_UCCAN_501_data, data);
	case 0x504: return pack_can_0x504_IMCU_504(&can_0x504_IMCU_504_data, data);
	case 0x505: return pack_can_0x505_IMCU_505(&can_0x505_IMCU_505_data, data);
	default: break; 
	}
	return -1; 
}

int print_message(unsigned id, FILE* data)
{
	switch(id) {
	case 0x501: return print_can_0x501_UCCAN_501(&can_0x501_UCCAN_501_data, data);
	case 0x504: return print_can_0x504_IMCU_504(&can_0x504_IMCU_504_data, data);
	case 0x505: return print_can_0x505_IMCU_505(&can_0x505_IMCU_505_data, data);
	default: break; 
	}
	return -1; 
}

can_0x501_UCCAN_501_t can_0x501_UCCAN_501_data;

int pack_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *pack, uint64_t *data)
{
	register uint64_t x;
	register uint64_t m = 0;
	/* UC501_Reqsteeringangle: start-bit 15, length 16, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->UC501_Reqsteeringangle + 0.000000) / 0.100000)) & 0xffff;
	x <<= 40; 
	m |= x;
	/* UC501_Empressure: start-bit 47, length 8, endianess motorola, scaling 2.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_Empressure + 0.000000) / 2.000000)) & 0xff;
	x <<= 16; 
	m |= x;
	/* UC501_Reqvehiclespeed: start-bit 31, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_Reqvehiclespeed + 0.000000) / 1.000000)) & 0xff;
	x <<= 32; 
	m |= x;
	/* UC501_Lightingcontrol: start-bit 33, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_Lightingcontrol + 0.000000) / 1.000000)) & 0x3;
	x <<= 24; 
	m |= x;
	/* UC501_Controlmode: start-bit 3, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_Controlmode + 0.000000) / 1.000000)) & 0x3;
	x <<= 58; 
	m |= x;
	/* UC501_OperatingmodeB: start-bit 1, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_OperatingmodeB + 0.000000) / 1.000000)) & 0x1;
	x <<= 57; 
	m |= x;
	/* UC501_OperatingmodeA: start-bit 0, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_OperatingmodeA + 0.000000) / 1.000000)) & 0x1;
	x <<= 56; 
	m |= x;
	/* UC501_EMflag: start-bit 4, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->UC501_EMflag + 0.000000) / 1.000000)) & 0x1;
	x <<= 60; 
	m |= x;
	*data = reverse_byte_order(m);
	return 0;
}

int unpack_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *unpack, uint64_t data, uint8_t dlc)
{
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if(dlc < 8)
		return -1;
	/* UC501_Reqsteeringangle: start-bit 15, length 16, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = (m >> 40) & 0xffff;
	/* UC501_Empressure: start-bit 47, length 8, endianess motorola, scaling 2.000000, offset 0.000000 */
	x = (m >> 16) & 0xff;
	unpack->UC501_Empressure = x * 2.000000 + 0.000000;
	/* UC501_Reqvehiclespeed: start-bit 31, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 32) & 0xff;
	unpack->UC501_Reqvehiclespeed = x * 1.000000 + 0.000000;
	/* UC501_Lightingcontrol: start-bit 33, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 24) & 0x3;
	unpack->UC501_Lightingcontrol = x * 1.000000 + 0.000000;
	/* UC501_Controlmode: start-bit 3, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 58) & 0x3;
	unpack->UC501_Controlmode = x * 1.000000 + 0.000000;
	/* UC501_OperatingmodeB: start-bit 1, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 57) & 0x1;
	unpack->UC501_OperatingmodeB = x * 1.000000 + 0.000000;
	/* UC501_OperatingmodeA: start-bit 0, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 56) & 0x1;
	unpack->UC501_OperatingmodeA = x * 1.000000 + 0.000000;
	/* UC501_EMflag: start-bit 4, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 60) & 0x1;
	unpack->UC501_EMflag = x * 1.000000 + 0.000000;
	return 0;
}

int print_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *print, FILE *data)
{
	double scaled;
	int r = 0;
	scaled = 0.100;
	r = printf("UC501_Reqsteeringangle = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_Reqsteeringangle));
	scaled = 2.000;
	r = printf("UC501_Empressure = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_Empressure));
	scaled = 1.000;
	r = printf("UC501_Reqvehiclespeed = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_Reqvehiclespeed));
	scaled = 1.000;
	r = printf("UC501_Lightingcontrol = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_Lightingcontrol));
	scaled = 1.000;
	r = printf("UC501_Controlmode = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_Controlmode));
	scaled = 1.000;
	r = printf("UC501_OperatingmodeB = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_OperatingmodeB));
	scaled = 1.000;
	r = printf("UC501_OperatingmodeA = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_OperatingmodeA));
	scaled = 1.000;
	r = printf("UC501_EMflag = %.3f (wire: %.0f)\n", scaled, (double)(print->UC501_EMflag));
	return r;
}

can_0x504_IMCU_504_t can_0x504_IMCU_504_data;

int pack_can_0x504_IMCU_504(can_0x504_IMCU_504_t *pack, uint64_t *data)
{
	register uint64_t x;
	register uint64_t m = 0;
	/* IMCU504_Reqsteeringangle: start-bit 15, length 16, endianess motorola, scaling 1.000000, offset -720.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU504_Reqsteeringangle + -720.000000) / 1.000000)) & 0xffff;
	x <<= 40; 
	m |= x;
	/* IMCU504_Fbsteeringangle: start-bit 39, length 16, endianess motorola, scaling 0.100000, offset -720.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU504_Fbsteeringangle + -720.000000) / 0.100000)) & 0xffff;
	x <<= 16; 
	m |= x;
	/* IMCU504_Reqvehiclespeed: start-bit 31, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Reqvehiclespeed + 0.000000) / 1.000000)) & 0xff;
	x <<= 32; 
	m |= x;
	/* IMCU504_Fbvehiclespeed: start-bit 55, length 8, endianess motorola, scaling 0.200000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Fbvehiclespeed + 0.000000) / 0.200000)) & 0xff;
	x <<= 8; 
	m |= x;
	/* IMCU504_Fbsteerwheeltorque: start-bit 63, length 8, endianess motorola, scaling 0.100000, offset -10.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Fbsteerwheeltorque + -10.000000) / 0.100000)) & 0xff;
	m |= x;
	/* IMCU504_Lightingstatus: start-bit 7, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Lightingstatus + 0.000000) / 1.000000)) & 0x3;
	x <<= 62; 
	m |= x;
	/* IMCU504_Gearstatus: start-bit 5, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Gearstatus + 0.000000) / 1.000000)) & 0x3;
	x <<= 60; 
	m |= x;
	/* IMCU504_Controlmodestatus: start-bit 3, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_Controlmodestatus + 0.000000) / 1.000000)) & 0x3;
	x <<= 58; 
	m |= x;
	/* IMCU504_OperatingstatusB: start-bit 1, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_OperatingstatusB + 0.000000) / 1.000000)) & 0x1;
	x <<= 57; 
	m |= x;
	/* IMCU504_OperatingstatusA: start-bit 0, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU504_OperatingstatusA + 0.000000) / 1.000000)) & 0x1;
	x <<= 56; 
	m |= x;
	*data = reverse_byte_order(m);
	return 0;
}

int unpack_can_0x504_IMCU_504(can_0x504_IMCU_504_t *unpack, uint64_t data, uint8_t dlc)
{
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if(dlc < 8)
		return -1;
	/* IMCU504_Reqsteeringangle: start-bit 15, length 16, endianess motorola, scaling 1.000000, offset -720.000000 */
	x = (m >> 40) & 0xffff;
	unpack->IMCU504_Reqsteeringangle = x * 1.000000 + -720.000000;
	/* IMCU504_Fbsteeringangle: start-bit 39, length 16, endianess motorola, scaling 0.100000, offset -720.000000 */
	x = (m >> 16) & 0xffff;
	unpack->IMCU504_Fbsteeringangle = x * 0.100000 + -720.000000;
	/* IMCU504_Reqvehiclespeed: start-bit 31, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 32) & 0xff;
	unpack->IMCU504_Reqvehiclespeed = x * 1.000000 + 0.000000;
	/* IMCU504_Fbvehiclespeed: start-bit 55, length 8, endianess motorola, scaling 0.200000, offset 0.000000 */
	x = (m >> 8) & 0xff;
	unpack->IMCU504_Fbvehiclespeed = x * 0.200000 + 0.000000;
	/* IMCU504_Fbsteerwheeltorque: start-bit 63, length 8, endianess motorola, scaling 0.100000, offset -10.000000 */
	x = m & 0xff;
	unpack->IMCU504_Fbsteerwheeltorque = x * 0.100000 + -10.000000;
	/* IMCU504_Lightingstatus: start-bit 7, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 62) & 0x3;
	unpack->IMCU504_Lightingstatus = x * 1.000000 + 0.000000;
	/* IMCU504_Gearstatus: start-bit 5, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 60) & 0x3;
	unpack->IMCU504_Gearstatus = x * 1.000000 + 0.000000;
	/* IMCU504_Controlmodestatus: start-bit 3, length 2, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 58) & 0x3;
	unpack->IMCU504_Controlmodestatus = x * 1.000000 + 0.000000;
	/* IMCU504_OperatingstatusB: start-bit 1, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 57) & 0x1;
	unpack->IMCU504_OperatingstatusB = x * 1.000000 + 0.000000;
	/* IMCU504_OperatingstatusA: start-bit 0, length 1, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 56) & 0x1;
	unpack->IMCU504_OperatingstatusA = x * 1.000000 + 0.000000;
	return 0;
}

int print_can_0x504_IMCU_504(can_0x504_IMCU_504_t *print, FILE *data)
{
	double scaled;
	int r = 0;
	scaled = 1.000;
	r = printf("IMCU504_Reqsteeringangle = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Reqsteeringangle));
	scaled = 0.100;
	r = printf("IMCU504_Fbsteeringangle = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Fbsteeringangle));
	scaled = 1.000;
	r = printf("IMCU504_Reqvehiclespeed = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Reqvehiclespeed));
	scaled = 0.200;
	r = printf("IMCU504_Fbvehiclespeed = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Fbvehiclespeed));
	scaled = 0.100;
	r = printf("IMCU504_Fbsteerwheeltorque = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Fbsteerwheeltorque));
	scaled = 1.000;
	r = printf("IMCU504_Lightingstatus = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Lightingstatus));
	scaled = 1.000;
	r = printf("IMCU504_Gearstatus = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Gearstatus));
	scaled = 1.000;
	r = printf("IMCU504_Controlmodestatus = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_Controlmodestatus));
	scaled = 1.000;
	r = printf("IMCU504_OperatingstatusB = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_OperatingstatusB));
	scaled = 1.000;
	r = printf("IMCU504_OperatingstatusA = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU504_OperatingstatusA));
	return r;
}

can_0x505_IMCU_505_t can_0x505_IMCU_505_data;

int pack_can_0x505_IMCU_505(can_0x505_IMCU_505_t *pack, uint64_t *data)
{
	register uint64_t x;
	register uint64_t m = 0;
	/* IMCU505_FrequencyR2: start-bit 35, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU505_FrequencyR2 + 0.000000) / 0.100000)) & 0xfff;
	x <<= 16; 
	m |= x;
	/* IMCU505_FrequencyR1: start-bit 31, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU505_FrequencyR1 + 0.000000) / 0.100000)) & 0xfff;
	x <<= 28; 
	m |= x;
	/* IMCU505_FrequencyL2: start-bit 11, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU505_FrequencyL2 + 0.000000) / 0.100000)) & 0xfff;
	x <<= 40; 
	m |= x;
	/* IMCU505_FrequencyL1: start-bit 7, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = ((uint16_t)rt_roundd_snf((pack->IMCU505_FrequencyL1 + 0.000000) / 0.100000)) & 0xfff;
	x <<= 52; 
	m |= x;
	/* IMCU505_BrakePedal: start-bit 63, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU505_BrakePedal + 0.000000) / 1.000000)) & 0xff;
	m |= x;
	/* IMCU505_AcceleratorPedal: start-bit 55, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = ((uint8_t)rt_roundd_snf((pack->IMCU505_AcceleratorPedal + 0.000000) / 1.000000)) & 0xff;
	x <<= 8; 
	m |= x;
	*data = reverse_byte_order(m);
	return 0;
}

int unpack_can_0x505_IMCU_505(can_0x505_IMCU_505_t *unpack, uint64_t data, uint8_t dlc)
{
	register uint64_t x;
	register uint64_t m = reverse_byte_order(data);
	if(dlc < 8)
		return -1;
	/* IMCU505_FrequencyR2: start-bit 35, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = (m >> 16) & 0xfff;
	unpack->IMCU505_FrequencyR2 = x * 0.100000 + 0.000000;
	/* IMCU505_FrequencyR1: start-bit 31, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = (m >> 28) & 0xfff;
	unpack->IMCU505_FrequencyR1 = x * 0.100000 + 0.000000;
	/* IMCU505_FrequencyL2: start-bit 11, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = (m >> 40) & 0xfff;
	unpack->IMCU505_FrequencyL2 = x * 0.100000 + 0.000000;
	/* IMCU505_FrequencyL1: start-bit 7, length 12, endianess motorola, scaling 0.100000, offset 0.000000 */
	x = (m >> 52) & 0xfff;
	unpack->IMCU505_FrequencyL1 = x * 0.100000 + 0.000000;
	/* IMCU505_BrakePedal: start-bit 63, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = m & 0xff;
	unpack->IMCU505_BrakePedal = x * 1.000000 + 0.000000;
	/* IMCU505_AcceleratorPedal: start-bit 55, length 8, endianess motorola, scaling 1.000000, offset 0.000000 */
	x = (m >> 8) & 0xff;
	unpack->IMCU505_AcceleratorPedal = x * 1.000000 + 0.000000;
	return 0;
}

int print_can_0x505_IMCU_505(can_0x505_IMCU_505_t *print, FILE *data)
{
	double scaled;
	int r = 0;
	scaled = 0.100;
	r = printf("IMCU505_FrequencyR2 = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_FrequencyR2));
	scaled = 0.100;
	r = printf("IMCU505_FrequencyR1 = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_FrequencyR1));
	scaled = 0.100;
	r = printf("IMCU505_FrequencyL2 = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_FrequencyL2));
	scaled = 0.100;
	r = printf("IMCU505_FrequencyL1 = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_FrequencyL1));
	scaled = 1.000;
	r = printf("IMCU505_BrakePedal = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_BrakePedal));
	scaled = 1.000;
	r = printf("IMCU505_AcceleratorPedal = %.3f (wire: %.0f)\n", scaled, (double)(print->IMCU505_AcceleratorPedal));
	return r;
}
