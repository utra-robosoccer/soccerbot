/*
 * packet.h
 *
 *  Created on: 2018-01-11
 *      Author: vuwij
 */

#ifndef PACKET_H
#define PACKET_H

#include <serial.h>

#define PACKET_SIZE 800

typedef struct packet {
	u_int8_t start_pattern;
	u_int8_t byte_count;
	u_int8_t data[PACKET_SIZE];
} Packet;

#endif /* PACKET_H */
