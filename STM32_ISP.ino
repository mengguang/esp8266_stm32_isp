#include <SoftwareSerial.h>
#include "FS.h"

SoftwareSerial debugSerial(13, 12, false, 256); //rx tx inverse buffer

#define ispSerial Serial
#define BLOCK_SIZE 128
#define START_KEY 0
#define BOOT0_PIN 14
#define RESET_PIN 16
#define BEGIN_ADDRESS 0x08000000

uint8_t memory_buffer[BLOCK_SIZE];

uint8_t start_key_pressed() {
	if(digitalRead(START_KEY) == LOW) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t reset_stm32_to_isp_mode() {
	pinMode(RESET_PIN,OUTPUT);
	pinMode(BOOT0_PIN,OUTPUT);
	digitalWrite(BOOT0_PIN,HIGH);
	delay(50);
	digitalWrite(RESET_PIN,LOW);
	delay(50);
	digitalWrite(RESET_PIN,HIGH);
	delay(50);
	digitalWrite(BOOT0_PIN,LOW);
	pinMode(BOOT0_PIN,INPUT);
	pinMode(RESET_PIN,INPUT);
}

uint8_t reset_stm32_to_app_mode() {
	pinMode(RESET_PIN,OUTPUT);
	pinMode(BOOT0_PIN,OUTPUT);
	digitalWrite(BOOT0_PIN,LOW);
	delay(50);
	digitalWrite(RESET_PIN,LOW);
	delay(50);
	digitalWrite(RESET_PIN,HIGH);
	pinMode(BOOT0_PIN,INPUT);
	pinMode(RESET_PIN,INPUT);
}

void hardware_init() {
	pinMode(RESET_PIN,INPUT);
	pinMode(BOOT0_PIN,INPUT);
	pinMode(START_KEY,INPUT);
	ispSerial.begin(57600,SERIAL_8E1);
	ispSerial.setTimeout(5000);
	debugSerial.begin(57600);
}

char log_buffer[128];
void debug_log() {
	debugSerial.print(log_buffer);
}

uint8_t isp_serial_write(uint8_t *buffer,uint8_t length) {
	return ispSerial.write(buffer,length);
}

uint8_t isp_serial_read(uint8_t *buffer,uint8_t length) {
	return ispSerial.readBytes(buffer,length);
}

uint8_t isp_serial_flush() {
	while(ispSerial.available() > 0) {
		ispSerial.read();
	}
	return 1;
}

uint8_t wait_for_ack(char *when) {
	uint8_t cmd[1] = {0};
	uint8_t nread = isp_serial_read(cmd,1);
	if(nread == 1) {
		if(cmd[0] == 0x79) { // ack
			return 1;
		}
		if(cmd[0] == 0x1F) { // nack
			sprintf(log_buffer,"got nack when: %s\n",when);
			debug_log();
			return 2;
		}
		sprintf(log_buffer,"got unknown response: %d when: %s.\n",cmd[0],when);
		debug_log();
	}
	sprintf(log_buffer,"no response when: %s.\n",when);
	debug_log();
	return 0;
}

uint8_t init_chip() {
	uint8_t cmd[1];
	cmd[0] = 0x7F;
	for(int i=0;i<100;i++){
		sprintf(log_buffer,"trying to init chip...\n");
		debug_log();
		isp_serial_write(cmd,1);
		if(wait_for_ack("init_chip_write_cmd") > 0) { // ack or nack 
			sprintf(log_buffer,"init chip successed.\n");
			debug_log();
			isp_serial_flush();
			return 1;
		}
		delay(50);
	}
	sprintf(log_buffer,"init chip failed.\n");
	debug_log();
	return 0;
}

uint8_t cmd_generic(uint8_t command) {
	isp_serial_flush();
	uint8_t cmd[2];
	cmd[0] = command;
	cmd[1] = command ^ 0xFF;
	isp_serial_write(cmd,2);
	return wait_for_ack("cmd_generic");
}

uint8_t cmd_get() {
	if(cmd_generic(0x00)) {
		uint8_t buffer[8];
		isp_serial_read(buffer,2);
		uint8_t len = buffer[0];
		uint8_t ver = buffer[1];
		sprintf(log_buffer,"version: %d\n",ver);
		debug_log();
		isp_serial_read(buffer,len);
		return wait_for_ack("cmd_get");
	}
	return 0;
}

void encode_address(uint32_t address, uint8_t *result) {
	uint8_t b3 = (uint8_t)((address >> 0) & 0xFF);
	uint8_t b2 = (uint8_t)((address >> 8) & 0xFF);
	uint8_t b1 = (uint8_t)((address >> 16) & 0xFF);
	uint8_t b0 = (uint8_t)((address >> 24) & 0xFF);
	
	uint8_t crc = (uint8_t)(b0 ^ b1 ^ b2 ^ b3);
	result[0] = b0;
	result[1] = b1;
	result[2] = b2;
	result[3] = b3;
	result[4] = crc;
}

uint8_t cmd_read_memory(uint32_t address, uint8_t length) {
	if(cmd_generic(0x11) == 1) {
		uint8_t enc_address[5] = {0};
		encode_address(address,enc_address);
		isp_serial_write(enc_address,5);
		if(wait_for_ack("write_address") != 1) {
			return 0;
		}
		uint8_t nread = length - 1;
		uint8_t crc = nread ^ 0xFF;
		uint8_t buffer[2];
		buffer[0] = nread;
		buffer[1] = crc;
		isp_serial_write(buffer,2);
		if(wait_for_ack("write_address") != 1) {
			return 0;
		}
		uint8_t nreaded = 0;
		while(nreaded < length) {
			nreaded += isp_serial_read(memory_buffer+nreaded, length - nreaded);
		}
		return 1;
	}
	return 0;
}

uint8_t cmd_write_memory(uint32_t address, uint8_t length) {
	if(cmd_generic(0x31) == 1) {
		uint8_t enc_address[5] = {0};
		encode_address(address,enc_address);
		isp_serial_write(enc_address,5);
		if(wait_for_ack("write_address") != 1) {
			return 0;
		}
		uint8_t nwrite = length - 1;
		uint8_t buffer[1];
		buffer[0] = nwrite;
		isp_serial_write(buffer,1);
		uint8_t crc = nwrite;
		for(int i=0;i<length;i++){
			crc = crc ^ memory_buffer[i];
		}
		isp_serial_write(memory_buffer,length);
		buffer[0] = crc;
		isp_serial_write(buffer,1);
		return wait_for_ack("write_data");
		
	}
	return 0;
}

uint8_t cmd_erase_all_memory() {
	if(cmd_generic(0x44) == 1) {
		uint8_t cmd[3];
		cmd[0] = 0xFF;
		cmd[1] = 0xFF;
		cmd[2] = 0x00;
		isp_serial_write(cmd,3);
		return wait_for_ack("erase_all_memory");
	}
	return 0;
}

uint8_t cmd_go(uint32_t address) {
	if(cmd_generic(0x21) == 1) {
		uint8_t enc_address[5] = {0};
		encode_address(address,enc_address);
		isp_serial_write(enc_address,5);
		return wait_for_ack("cmd_go");
	}
	return 0;
}

uint8_t esp8266_spifs_write_file(char *filename) {
	reset_stm32_to_isp_mode();
	if(init_chip() != 1) {
		return 0;
	}
	cmd_get();
	sprintf(log_buffer,"erasing all memory...\n");
	debug_log();
	
	if(cmd_erase_all_memory() != 1) {	
		return 0;
	}
	
	SPIFFS.begin();
	if(!SPIFFS.exists(filename)) {
		sprintf(log_buffer,"file: %s not exists.\n",filename);
		debug_log();
		return 0;
	}
	File fp = SPIFFS.open(filename,"r");
	uint32_t filesize = fp.size();
	sprintf(log_buffer,"filesize: %d\n",filesize);
	debug_log();
	sprintf(log_buffer,"begin to write file.\n");
	debug_log();
	while(fp.position() < filesize) {
		uint8_t nread = BLOCK_SIZE;
		if(filesize - fp.position() < BLOCK_SIZE) {
			nread = filesize - fp.position();
		}
		nread = fp.readBytes((char *)memory_buffer,nread);
		sprintf(log_buffer,"read bytes: %d, file position: %d\n",nread, fp.position());
		debug_log();
		uint8_t result = cmd_write_memory(BEGIN_ADDRESS+fp.position() - nread,nread);
		if(result != 1) {
			sprintf(log_buffer,"file write failed.\n");
			debug_log();
			return 0;
		}
	}
	sprintf(log_buffer,"file write successed.\n");
	debug_log();
	uint8_t file_buffer[BLOCK_SIZE];
	fp.seek(0,SeekSet);
	sprintf(log_buffer,"begin to verify file.\n");
	debug_log();
	while(fp.position() < filesize) {
		uint8_t nread = BLOCK_SIZE;
		if(filesize - fp.position() < BLOCK_SIZE) {
			nread = filesize - fp.position();
		}
		nread = fp.readBytes((char *)file_buffer,nread);
		sprintf(log_buffer,"read bytes: %d, file position: %d\n",nread, fp.position());
		debug_log();
		uint8_t result = cmd_read_memory(BEGIN_ADDRESS+fp.position() - nread, nread);
		if(result != 1) {
			sprintf(log_buffer,"read memory failed: %d\n",fp.position());
			debug_log();
			return 0;
		}
		result = memcmp(file_buffer,memory_buffer,nread);
		if(result != 0) {
			sprintf(log_buffer,"verify failed.\n");
			debug_log();
			return 0;
		}
	}
	sprintf(log_buffer,"verify file successed.\n");
	debug_log();
	sprintf(log_buffer,"start application.\n");
	debug_log();
	//reset_stm32_to_app_mode();
	cmd_go(BEGIN_ADDRESS);
	return 1;
}

void setup() {
	hardware_init();
	delay(1000);
	while(! start_key_pressed()) {
		delay(1000);
		sprintf(log_buffer,"press key to start.\n");
		debug_log();
	}
	esp8266_spifs_write_file("/stm32f030c8_db_led.bin");
}

void loop() {
	delay(100);
}
