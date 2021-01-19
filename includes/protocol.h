#ifndef PROTOCOL_H
#define PROTOCOL_H

enum msg_type {
	UNKNWON = 0,
	// regular directional control message for RC car
	DIRECTIONAL_CONTROL = 1 ,
	// Over-the-air firmware update
	OTA_UPDATE_HEADER = 2 ,
	OTA_UPDATE_PAYLOAD = 3,	
	OTA_UPDATE_RESPONSE = 4
};

enum msg_ack_nack {
	RECEIVED_OK  = 0xaa,
	RECEIVED_NOT_OK = 0x55
};

// msg sequence ID, first message of a group starts with sequence ID 0
typedef msg_type uint8_t;
typedef sequence_id uint16_t;

// first message to be sent when transmitting an OTA update
typedef struct msg_ota_hdr {
	sequence_id sequence_id;	
	uint16_t total_payload_byte_cnt;
} msg_ota_hdr;

// payload message when sending an OTA update
typedef struct msg_ota_payload {
	sequence_id sequence_id;
	uint8_t payload_size;
	uint8_t payload[];
} msg_ota_payload;

typedef struct msg_ota_response {
	sequence_id sequence_id;
	msg_ack_nack response;
} msg_ota_payload;

// directional update for RC car
typedef struct msg_dir_update {	
	// X-axis speed ( -100 ... 100 ) in percent
	int8_t xDir;
	// Y-axis speed ( -100 ... 100 ) in percent	
	int8_t yDir;
} msg_dir_update;

// Message 
typedef struct msg {
	// make sure to update PROTOCOL_MAX_MESSAGE_SIZE when adding
	// new fields to this struct
	msg_type type;
	union {
		msg_ota_hdr ota_hdr,
		msg_ota_payload ota_payload,
		msg_dir_update dir_update
	}
	uint8_t crc;		
} msg;

// Maximum size of message payload ; note that the size needs to be rather
// small as data is only protected by a CRC-8 and using
// too large payloads will result in transmission errors 
// not getting recognized
#define PROTOCOL_MAX_PAYLOAD_SIZE 32

#define member_size(type, member) sizeof(((type *)0)->member)

#define PROTOCOL_MAX_MESSAGE_SIZE ( PROTOCOL_MAX_PAYLOAD_SIZE + member_size(msg,type) + member_size(msg,crc) )

void proto_send_dir_update(uint8_t xDir, uint8_t yUpdate);
msg_ack_nack proto_send_ota_header(uint16_t total_payload_byte_cnt);
msg_ack_nack proto_send_ota_payload_block(void *payload,uint8_t length);
#endif