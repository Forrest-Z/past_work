/** @brief CAN message encoder/decoder: automatically generated - do not edit
  * @note  Generated by dbcc: See https://github.com/howerj/dbcc
  */

#ifndef EQYC_UCCAN_H
#define EQYC_UCCAN_H

#include <stdint.h>
#include <stdio.h>

typedef double real_T;

#ifdef __cplusplus
extern "C" { 
#endif

int unpack_message(unsigned id, uint64_t data, uint8_t dlc);
int pack_message(unsigned id, uint64_t *data);
int print_message(unsigned id, FILE* data);

typedef struct {
	real_T UC501_Reqsteeringangle; /*scaling 0.1, offset 0.0, units deg*/
	real_T UC501_Empressure; /*scaling 2.0, offset 0.0, units %*/
	uint8_t UC501_Reqvehiclespeed; /*scaling 1.0, offset 0.0, units km/h*/
	uint8_t UC501_Lightingcontrol; /*scaling 1.0, offset 0.0, units none*/
	uint8_t UC501_Controlmode; /*scaling 1.0, offset 0.0, units none*/
	uint8_t UC501_OperatingmodeB; /*scaling 1.0, offset 0.0, units none*/
	uint8_t UC501_OperatingmodeA; /*scaling 1.0, offset 0.0, units none*/
	uint8_t UC501_EMflag; /*scaling 1.0, offset 0.0, units none*/
} can_0x501_UCCAN_501_t;

extern can_0x501_UCCAN_501_t can_0x501_UCCAN_501_data;
int pack_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *pack, uint64_t *data);
int unpack_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *unpack, uint64_t data, uint8_t dlc);

double decode_can_0x501_UC501_Reqsteeringangle(can_0x501_UCCAN_501_t *record);
double encode_can_0x501_UC501_Reqsteeringangle(can_0x501_UCCAN_501_t *record);
double decode_can_0x501_UC501_Empressure(can_0x501_UCCAN_501_t *record);
double encode_can_0x501_UC501_Empressure(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_Reqvehiclespeed(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_Reqvehiclespeed(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_Lightingcontrol(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_Lightingcontrol(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_Controlmode(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_Controlmode(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_OperatingmodeB(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_OperatingmodeB(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_OperatingmodeA(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_OperatingmodeA(can_0x501_UCCAN_501_t *record);
uint8_t decode_can_0x501_UC501_EMflag(can_0x501_UCCAN_501_t *record);
uint8_t encode_can_0x501_UC501_EMflag(can_0x501_UCCAN_501_t *record);
int print_can_0x501_UCCAN_501(can_0x501_UCCAN_501_t *print, FILE *data);


typedef struct {
	int16_t IMCU504_Reqsteeringangle; /*scaling 1.0, offset -720.0, units deg*/
	real_T IMCU504_Fbsteeringangle; /*scaling 0.1, offset -720.0, units deg*/
	uint8_t IMCU504_Reqvehiclespeed; /*scaling 1.0, offset 0.0, units km/h*/
	real_T IMCU504_Fbvehiclespeed; /*scaling 0.2, offset 0.0, units km/h*/
	real_T IMCU504_Fbsteerwheeltorque; /*scaling 0.1, offset -10.0, units Nm*/
	uint8_t IMCU504_Lightingstatus; /*scaling 1.0, offset 0.0, units none*/
	uint8_t IMCU504_Gearstatus; /*scaling 1.0, offset 0.0, units none*/
	uint8_t IMCU504_Controlmodestatus; /*scaling 1.0, offset 0.0, units none*/
	uint8_t IMCU504_OperatingstatusB; /*scaling 1.0, offset 0.0, units none*/
	uint8_t IMCU504_OperatingstatusA; /*scaling 1.0, offset 0.0, units none*/
} can_0x504_IMCU_504_t;

extern can_0x504_IMCU_504_t can_0x504_IMCU_504_data;
int pack_can_0x504_IMCU_504(can_0x504_IMCU_504_t *pack, uint64_t *data);
int unpack_can_0x504_IMCU_504(can_0x504_IMCU_504_t *unpack, uint64_t data, uint8_t dlc);

double decode_can_0x504_IMCU504_Reqsteeringangle(can_0x504_IMCU_504_t *record);
double encode_can_0x504_IMCU504_Reqsteeringangle(can_0x504_IMCU_504_t *record);
double decode_can_0x504_IMCU504_Fbsteeringangle(can_0x504_IMCU_504_t *record);
double encode_can_0x504_IMCU504_Fbsteeringangle(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_Reqvehiclespeed(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_Reqvehiclespeed(can_0x504_IMCU_504_t *record);
double decode_can_0x504_IMCU504_Fbvehiclespeed(can_0x504_IMCU_504_t *record);
double encode_can_0x504_IMCU504_Fbvehiclespeed(can_0x504_IMCU_504_t *record);
double decode_can_0x504_IMCU504_Fbsteerwheeltorque(can_0x504_IMCU_504_t *record);
double encode_can_0x504_IMCU504_Fbsteerwheeltorque(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_Lightingstatus(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_Lightingstatus(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_Gearstatus(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_Gearstatus(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_Controlmodestatus(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_Controlmodestatus(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_OperatingstatusB(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_OperatingstatusB(can_0x504_IMCU_504_t *record);
uint8_t decode_can_0x504_IMCU504_OperatingstatusA(can_0x504_IMCU_504_t *record);
uint8_t encode_can_0x504_IMCU504_OperatingstatusA(can_0x504_IMCU_504_t *record);
int print_can_0x504_IMCU_504(can_0x504_IMCU_504_t *print, FILE *data);


typedef struct {
	real_T IMCU505_FrequencyR2; /*scaling 0.1, offset 0.0, units Hz*/
	real_T IMCU505_FrequencyR1; /*scaling 0.1, offset 0.0, units Hz*/
	real_T IMCU505_FrequencyL2; /*scaling 0.1, offset 0.0, units Hz*/
	real_T IMCU505_FrequencyL1; /*scaling 0.1, offset 0.0, units Hz*/
	uint8_t IMCU505_BrakePedal; /*scaling 1.0, offset 0.0, units %*/
	uint8_t IMCU505_AcceleratorPedal; /*scaling 1.0, offset 0.0, units %*/
} can_0x505_IMCU_505_t;

extern can_0x505_IMCU_505_t can_0x505_IMCU_505_data;
int pack_can_0x505_IMCU_505(can_0x505_IMCU_505_t *pack, uint64_t *data);
int unpack_can_0x505_IMCU_505(can_0x505_IMCU_505_t *unpack, uint64_t data, uint8_t dlc);

double decode_can_0x505_IMCU505_FrequencyR2(can_0x505_IMCU_505_t *record);
double encode_can_0x505_IMCU505_FrequencyR2(can_0x505_IMCU_505_t *record);
double decode_can_0x505_IMCU505_FrequencyR1(can_0x505_IMCU_505_t *record);
double encode_can_0x505_IMCU505_FrequencyR1(can_0x505_IMCU_505_t *record);
double decode_can_0x505_IMCU505_FrequencyL2(can_0x505_IMCU_505_t *record);
double encode_can_0x505_IMCU505_FrequencyL2(can_0x505_IMCU_505_t *record);
double decode_can_0x505_IMCU505_FrequencyL1(can_0x505_IMCU_505_t *record);
double encode_can_0x505_IMCU505_FrequencyL1(can_0x505_IMCU_505_t *record);
uint8_t decode_can_0x505_IMCU505_BrakePedal(can_0x505_IMCU_505_t *record);
uint8_t encode_can_0x505_IMCU505_BrakePedal(can_0x505_IMCU_505_t *record);
uint8_t decode_can_0x505_IMCU505_AcceleratorPedal(can_0x505_IMCU_505_t *record);
uint8_t encode_can_0x505_IMCU505_AcceleratorPedal(can_0x505_IMCU_505_t *record);
int print_can_0x505_IMCU_505(can_0x505_IMCU_505_t *print, FILE *data);


#ifdef __cplusplus
} 
#endif

#endif