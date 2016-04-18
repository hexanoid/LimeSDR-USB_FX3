/**
-- ----------------------------------------------------------------------------	
-- FILE:	lime_sdr-usb_brd.h
-- DESCRIPTION:	LimeSDR-USB
-- DATE:	2016.04.14
-- AUTHOR(s):	Lime Microsystems
-- REVISION: v0r2
-- ----------------------------------------------------------------------------	

*/
#ifndef _LIMESDR_USB_BRD_H_
#define _LIMESDR_USB_BRD_H_

#include "LMS64C_protocol.h"

//get info
#define DEV_TYPE			LMS_DEV_LIMESDR_USB
#define HW_VER				2 //LimeSDR-USB 1.2
#define EXP_BOARD			EXP_BOARD_UNSUPPORTED

//I2C devices
#define SI5351_I2C_ADDR		0xC0
#define MAX7322_I2C_ADDR	0xDA
#define LM75_I2C_ADDR		0x90
#define SC18IS602B_I2C_ADDR	0x50
#define EEPROM_I2C_ADDR		0xA0

//SC18IS602B SPI SS
#define BRDG_SPI_FPGA_SS	0x01
#define BRDG_SPI_ADF_SS		0x02
#define BRDG_SPI_DAC_SS		0x04
#define BRDG_SPI_DUMMY_SS	0x08

#define FX3_FLASH1_SS		54

// RG LED
#define FX3_LED_R	27
#define FX3_LED_G	26

#define SC18B20_INT_GPIO	45


//BRD_SPI map

#define FPGA_SPI_REG_LMS1_LMS2_CTRL  0x13

#define LMS1_SS			0
#define LMS1_RESET		1

#define FPGA_SPI_REG_SS_CTRL  0x12

#define TCXO_ADF_SS		0 //SS0
#define TCXO_DAC_SS		1 //SS1

//FX3 Flash
#define FX3_SIZE 					(512*1024)
#define FX3_FLASH_PAGE_SIZE 		0x100 //256 bytes, SPI Page size to be used for transfers
#define FX3_FLASH_SECTOR_SIZE 		0x10000 //256 pages * 256 page size = 65536 bytes
#define FX3_FLASH_CMD_SECTOR_ERASE 	0xD8

//FPGA Cyclone IV (EP4CE40F23C7N) bitstream (RBF) size in bytes 
#define FPGA_SIZE 			1191788

//FLash memory (M25P16, 16M-bit))
#define FLASH_PAGE_SIZE 	0x100 //256 bytes, SPI Page size to be used for transfers
#define FLASH_SECTOR_SIZE 	0x10000 //256 pages * 256 page size = 65536 bytes
//#define FLASH_BLOCK_SIZE	(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE) //in pages

//FLASH memory layout
//#define FLASH_LAYOUT_FPGA_METADATA	12//FPGA autoload metadata (start sector)
#define FLASH_LAYOUT_FPGA_BITSTREAM	0//FPGA bitstream (start sector) till end

#define FLASH_CMD_SECTOR_ERASE 0xD8 //depends on flash: 0xD8 or 0x20

typedef struct{
	uint32_t Bitream_size;
	uint8_t Autoload;
}tBoard_Config_FPGA; //tBoard_Config_FPGA or tBoard_cfg_FPGA

#endif
