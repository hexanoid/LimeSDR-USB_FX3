/*
 * Copyright (c) 2013 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "cyu3gpio.h"
#include "pib_regs.h"
#include "cyu3spi.h"
#include "spi_flash_lib.h"
#include "lime_sdr-usb_brd.h"
#include "cyu3i2c.h"

#define CHANGE_SPI_TO_LSB

 uint8_t   I2C_Addr, data, cmd_errors, sc_brdg_data[255];
 CyU3PI2cPreamble_t preamble;

/* SPI initialization for application. */
CyU3PReturnStatus_t FlashSpiInit(void)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS) {
        return status;
    }

    /* Start the SPI master block. Run the SPI clock at 8MHz
     * and configure the word length to 8 bits. Also configure
     * the slave select using FW. */
    CyU3PMemSet((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 8000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig(&spiConfig, NULL);

    return status;
}

CyU3PReturnStatus_t FlashSpiDeInit() {
	return CyU3PSpiDeInit();
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t CyFxSpiWaitForStatus(void)
{
    uint8_t buf[2], rd_buf[2];
    //CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do {
        buf[0] = 0x06;  /* Write enable command. */

        /*CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }*/

    	/*//write byte
        I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
        preamble.length    = 2;
        preamble.buffer[0] = I2C_Addr; //write h70;
        preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
        preamble.ctrlMask  = 0x0000;
        sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

        ///CyU3PThreadSleep (1);

		//write-read SPI bytes using using I2C-SPI bridge
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, &buf[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		CyU3PThreadSleep (1);

		/*//write byte
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		///CyU3PThreadSleep (1);

        buf[0] = 0x05;  /* Read status command */

        /*CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
        status = CyU3PSpiTransmitWords(buf, 1);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI READ_STATUS command failed\n\r");
            CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords(rd_buf, 2);
        CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        if(status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI status read failed\n\r");
            return status;
        }*/


    	//write byte
        /*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
        preamble.length    = 2;
        preamble.buffer[0] = I2C_Addr; //write h70;
        preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
        preamble.ctrlMask  = 0x0000;
        sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

        ///CyU3PThreadSleep (1);


		//write/read SPI bytes using using I2C-SPI bridge
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
		preamble.ctrlMask  = 0x0000;

		sc_brdg_data[0] = buf[0];
		sc_brdg_data[1] = 0; //dummy byte for read
		sc_brdg_data[2] = 0; //dummy byte for read

		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1 + 2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		CyU3PThreadSleep (1);

		//read received SPI bytes from I2C-SPI bridge buffer
		I2C_Addr |= 1 << 0;	//read addr
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 1 + 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

		CyU3PThreadSleep (1);

		rd_buf[0] = sc_brdg_data[1];
		rd_buf[1] = sc_brdg_data[2];


		/*//write byte
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		///CyU3PThreadSleep (1);


    } while ((rd_buf[0] & 1) || (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

CyBool_t spiFastRead = CyFalse;
void FlashSpiFastRead(CyBool_t v) {
    spiFastRead = v;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead)
{
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / FLASH_PAGE_SIZE);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    spiFastRead = CyTrue;// force fast read

    if (byteCount == 0) {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % FLASH_PAGE_SIZE) != 0) {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }

    byteAddress = pageAddress * FLASH_PAGE_SIZE;

    while (pageCount != 0) {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead) {
            location[0] = 0x03; /* Read command. */

            if (!spiFastRead) {
                status = CyFxSpiWaitForStatus();
                if (status != CY_U3P_SUCCESS)
                    return status;
            }

            /*
            CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);

            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint (2, "SPI READ command failed\r\n");
                CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            status = CyU3PSpiReceiveWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
            */

        	//write byte
            /*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
            preamble.length    = 2;
            preamble.buffer[0] = I2C_Addr; //write h70;
            preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
            preamble.ctrlMask  = 0x0000;
            sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
            if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
			preamble.ctrlMask  = 0x0000;
			sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
			if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

            ///CyU3PThreadSleep (1);


        	//write-read SPI bytes using using I2C-SPI bridge
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

        	preamble.length    = 2;
        	preamble.buffer[0] = I2C_Addr; //write h70;
        	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
        	preamble.ctrlMask  = 0x0000;

        	if( CyU3PI2cTransmitBytes (&preamble, &location[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
        	CyU3PThreadSleep (1);


    		memset (sc_brdg_data, 0, sizeof(sc_brdg_data)); //dummy byte for read


    		//write/read SPI bytes using using I2C-SPI bridge
    		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

    		preamble.length    = 2;
    		preamble.buffer[0] = I2C_Addr; //write h70;
    		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
    		preamble.ctrlMask  = 0x0000;

    		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], FLASH_PAGE_SIZE/2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    		CyU3PThreadSleep (10);

    		//read received SPI bytes from I2C-SPI bridge buffer
    		I2C_Addr |= 1 << 0;	//read addr
    		preamble.length    = 1;
    		preamble.buffer[0] = I2C_Addr;
    		preamble.ctrlMask  = 0x0000;

    		if( CyU3PI2cReceiveBytes (&preamble, &buffer[0 + 0], FLASH_PAGE_SIZE/2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

    		CyU3PThreadSleep (1);



    		//write/read SPI bytes using using I2C-SPI bridge
    		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

    		preamble.length    = 2;
    		preamble.buffer[0] = I2C_Addr; //write h70;
    		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
    		preamble.ctrlMask  = 0x0000;

    		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], FLASH_PAGE_SIZE/2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    		CyU3PThreadSleep (10);

    		//read received SPI bytes from I2C-SPI bridge buffer
    		I2C_Addr |= 1 << 0;	//read addr
    		preamble.length    = 1;
    		preamble.buffer[0] = I2C_Addr;
    		preamble.ctrlMask  = 0x0000;

    		if( CyU3PI2cReceiveBytes (&preamble, &buffer[0 + FLASH_PAGE_SIZE/2], FLASH_PAGE_SIZE/2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

    		CyU3PThreadSleep (1);


    		//write byte
    		/*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    		preamble.length    = 2;
    		preamble.buffer[0] = I2C_Addr; //write h70;
    		preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
    		preamble.ctrlMask  = 0x0000;
    		sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
    		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
			preamble.ctrlMask  = 0x0000;
			sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
			if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    		///CyU3PThreadSleep (1);


        } else { /* Write */
            location[0] = 0x02; /* Write command */

            status = CyFxSpiWaitForStatus();
            if (status != CY_U3P_SUCCESS)
                return status;

            /*
            CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(2, "SPI WRITE command failed\r\n");
                CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            status = CyU3PSpiTransmitWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
             */


        	//write byte
            /*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
            preamble.length    = 2;
            preamble.buffer[0] = I2C_Addr; //write h70;
            preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
            preamble.ctrlMask  = 0x0000;
            sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
            if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
			preamble.ctrlMask  = 0x0000;
			sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
			if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

            ///CyU3PThreadSleep (1);


        	//write-read SPI bytes using using I2C-SPI bridge
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

        	preamble.length    = 2;
        	preamble.buffer[0] = I2C_Addr; //write h70;
        	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
        	preamble.ctrlMask  = 0x0000;

        	if( CyU3PI2cTransmitBytes (&preamble, &location[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
        	CyU3PThreadSleep (1);


			#ifdef CHANGE_SPI_TO_LSB
        	//write byte
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
            preamble.length    = 2;
            preamble.buffer[0] = I2C_Addr; //write h70;
            preamble.buffer[1] = 0xF0; //reg to write = configure spi
            preamble.buffer[2] = 0x00;
            preamble.ctrlMask  = 0x0000;
            data = 0x20; // LSB word first, Mode 0 (CPOL = 0, CPHA = 0), 1.843 MHz
            if( CyU3PI2cTransmitBytes (&preamble, &data, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

            CyU3PThreadSleep (1);
 			#endif



        	//write-read SPI bytes using using I2C-SPI bridge
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

        	preamble.length    = 2;
        	preamble.buffer[0] = I2C_Addr; //write h70;
        	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
        	preamble.ctrlMask  = 0x0000;

        	if( CyU3PI2cTransmitBytes (&preamble, &buffer[0 + 0], FLASH_PAGE_SIZE/2, 0) != CY_U3P_SUCCESS)  cmd_errors++;
        	CyU3PThreadSleep (4);


        	//write-read SPI bytes using using I2C-SPI bridge
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

        	preamble.length    = 2;
        	preamble.buffer[0] = I2C_Addr; //write h70;
        	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
        	preamble.ctrlMask  = 0x0000;

        	if( CyU3PI2cTransmitBytes (&preamble, &buffer[0 + FLASH_PAGE_SIZE/2], FLASH_PAGE_SIZE/2, 0) != CY_U3P_SUCCESS)  cmd_errors++;
        	CyU3PThreadSleep (4);


			#ifdef CHANGE_SPI_TO_LSB
        	//write byte
        	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
            preamble.length    = 2;
            preamble.buffer[0] = I2C_Addr; //write h70;
            preamble.buffer[1] = 0xF0; //reg to write = configure spi
            preamble.buffer[2] = 0x00;
            preamble.ctrlMask  = 0x0000;
            data = 0x00; // MSB word first, Mode 0 (CPOL = 0, CPHA = 0), 1.843 MHz
            if( CyU3PI2cTransmitBytes (&preamble, &data, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

            CyU3PThreadSleep (1);
			#endif


    		/*//write byte
    		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    		preamble.length    = 2;
    		preamble.buffer[0] = I2C_Addr; //write h70;
    		preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
    		preamble.ctrlMask  = 0x0000;
    		sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
    		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
			preamble.ctrlMask  = 0x0000;
			sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
			if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    		///CyU3PThreadSleep (1);

        }

        byteAddress += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        pageCount--;

        if (!spiFastRead)
            CyU3PThreadSleep(15);
    }

    return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
CyU3PReturnStatus_t FlashSpiEraseSector(CyBool_t isErase, uint8_t sector)
{
    uint32_t temp = 0;
    uint8_t  location[4], rdBuf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x06;  /* Write enable. */

    /*CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;*/


	/*//write byte
    I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    preamble.length    = 2;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
    preamble.ctrlMask  = 0x0000;
    sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    ///CyU3PThreadSleep (1);


	//write-read SPI bytes using using I2C-SPI bridge
	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr; //write h70;
	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cTransmitBytes (&preamble, &location[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
	CyU3PThreadSleep (1);

	/*//write byte
	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr; //write h70;
	preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

	//write byte
	preamble.length    = 1;
	preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	///CyU3PThreadSleep (1);

    if (isErase)
    {
        location[0] = FLASH_CMD_SECTOR_ERASE; /* Sector erase. */
        temp        = sector * FLASH_SECTOR_SIZE;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        /*CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 4);
        CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);*/

    	//write byte
        /*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
        preamble.length    = 2;
        preamble.buffer[0] = I2C_Addr; //write h70;
        preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
        preamble.ctrlMask  = 0x0000;
        sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

        ///CyU3PThreadSleep (1);


    	//write-read SPI bytes using using I2C-SPI bridge
    	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

    	preamble.length    = 2;
    	preamble.buffer[0] = I2C_Addr; //write h70;
    	preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
    	preamble.ctrlMask  = 0x0000;

    	if( CyU3PI2cTransmitBytes (&preamble, &location[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
    	CyU3PThreadSleep (1);


    	//write byte
    	/*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    	preamble.length    = 2;
    	preamble.buffer[0] = I2C_Addr; //write h70;
    	preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
    	preamble.ctrlMask  = 0x0000;
    	sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
    	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    	///CyU3PThreadSleep (1);
    }

    location[0] = 0x05; /* RDSTATUS */
    do {
    	/*CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 1);
        status = CyU3PSpiReceiveWords(rdBuf, 1);
        CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);*/

    	//write byte
        /*I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
        preamble.length    = 2;
        preamble.buffer[0] = I2C_Addr; //write h70;
        preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
        preamble.ctrlMask  = 0x0000;
        sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x40; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 0,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

        ///CyU3PThreadSleep (1);


		//write/read SPI bytes using using I2C-SPI bridge
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
		preamble.ctrlMask  = 0x0000;

		sc_brdg_data[0] = location[0];
		sc_brdg_data[1] = 0; //dummy byte for read

		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1 + 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
		CyU3PThreadSleep (1);

		//read received SPI bytes from I2C-SPI bridge buffer
		I2C_Addr |= 1 << 0;	//read addr
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 1 + 1, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

		CyU3PThreadSleep (1);

		rdBuf[0] = sc_brdg_data[1];

		/*//write byte
		I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr; //write h70;
		preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;*/

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = 0xDA; //MAX7322_I2C_ADDR
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0x41; //NCONFIG = 1, AS_SW = 0,.. EXP_AS_SS = 1,
		if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		///CyU3PThreadSleep (1);

    } while(rdBuf[0] & 1);

    return status;
}


///test function
void Flash_ID()
{
	uint8_t res1, res2, res3, spi_data_wr;

	//CyU3PGpioSetValue (FX3_FLASH2_SNN, CyFalse);//digitalWrite(_pinCE, LOW);

	CyU3PSpiTransmitWords (&spi_data_wr, 1);//_writeByte(0x9F);

	CyU3PSpiReceiveWords (&res1, 1); //res1=_readByte();
	CyU3PSpiReceiveWords (&res2, 1); //res2=_readByte();
	CyU3PSpiReceiveWords (&res3, 1); //res3=_readByte();
	//CyU3PGpioSetValue (FX3_FLASH2_SNN, CyTrue);////digitalWrite(_pinCE, HIGH);

	spi_data_wr = 0x9F;

	//write byte
    I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    preamble.length    = 2;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
    preamble.ctrlMask  = 0x0000;
    sc_brdg_data[0] = 0x00; //FLASH_SS gpio = 0
    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    CyU3PThreadSleep (1);


    //write/read SPI bytes using using I2C-SPI bridge
	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
    preamble.length    = 2;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = BRDG_SPI_DUMMY_SS;// ADF_SS dummy //0x01; //FLASH SS
    preamble.ctrlMask  = 0x0000;

    sc_brdg_data[0] = spi_data_wr;
    sc_brdg_data[1] = 0; //dummy byte for read
    sc_brdg_data[2] = 0; //dummy byte for read
    sc_brdg_data[3] = 0; //dummy byte for read

    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1 + 3, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	CyU3PThreadSleep (1);

	//read received SPI bytes from I2C-SPI bridge buffer
	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR

	I2C_Addr |= 1 << 0;	//read addr

	preamble.length    = 1;
	preamble.buffer[0] = I2C_Addr;
	preamble.ctrlMask  = 0x0000;

	if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 1 + 3, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

	res1 = sc_brdg_data[1]; //for testimng only
	res2 = sc_brdg_data[2]; //for testimng only
	res3 = sc_brdg_data[3]; //for testimng only
	res3 = sc_brdg_data[3]+1;

	CyU3PThreadSleep (1);

	//write byte
	I2C_Addr = 0x50;// SC18IS602B_I2C_ADDR
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr; //write h70;
	preamble.buffer[1] = 0xF4; //reg to write = GPIO Write
	preamble.ctrlMask  = 0x0000;
	sc_brdg_data[0] = 0x01; //FLASH_SS gpio = 1
	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	CyU3PThreadSleep (1);



	/*ID_manufacturer=res1;
	ID_type=res2;
	ID_device=res3;

	Text_manufacturer="xxx";
	for(int i = 0; i<(sizeof(_device)/sizeof(struct _s_device)); i++)
	{
		if ((_device[i]._manufacturer_id==res1) & (_device[i]._type_id==res2) & (_device[i]._device_id==res3))
		{
			Text_manufacturer	= _device[i]._manufacturer_name;
			Text_type			= _device[i]._device_type;
			Text_device			= _device[i]._device_name;
			Capacity			= _device[i]._size;
			Pages				= _device[i]._pages+1;
			_max_pages			= _device[i]._pages;
			_cmd_write			= _device[i]._cmd_write;
			_flags				= _device[i]._flags;
		}
	}
	if (Text_manufacturer=="xxx")
	{
		Text_manufacturer	= "(Unknown manufacturer)";
		Text_type			= "(Unknown type)";
		Text_device			= "(Unknown device)";
		Capacity			= 0;
		ID_manufacturer		= 0;
		ID_type				= 0;
		ID_device			= 0;
		_max_pages			= 0;
		_cmd_write			= 0;
		_flags				= 0;
	}*/

}


////////////////functions for fx3 flash

/* Wait for the status response from the FX3 SPI flash. */
CyU3PReturnStatus_t FX3_FlashSpiWaitForStatus(void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do {
        buf[0] = 0x06;  /* Write enable command. */

        CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
        status = CyU3PSpiTransmitWords(buf, 1);
        if (status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI READ_STATUS command failed\n\r");
            CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords(rd_buf, 2);
        CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        if(status != CY_U3P_SUCCESS) {
            CyU3PDebugPrint(2, "SPI status read failed\n\r");
            return status;
        }

    } while ((rd_buf[0] & 1) || (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

/* FX3 SPI flash read / write for programmer application. */
CyU3PReturnStatus_t FX3_FlashSpiTransfer(uint16_t pageAddress, uint16_t byteCount, uint8_t *buffer, CyBool_t isRead)
{
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / FLASH_PAGE_SIZE);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    spiFastRead = CyTrue;// force fast read

    if (byteCount == 0) {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % FLASH_PAGE_SIZE) != 0) {
        return CY_U3P_ERROR_NOT_SUPPORTED;
    }

    byteAddress = pageAddress * FLASH_PAGE_SIZE;

    while (pageCount != 0) {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead) {
            location[0] = 0x03; /* Read command. */

            if (!spiFastRead) {
                status = FX3_FlashSpiWaitForStatus();
                if (status != CY_U3P_SUCCESS)
                    return status;
            }

            CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);

            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint (2, "SPI READ command failed\r\n");
                CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            status = CyU3PSpiReceiveWords(buffer, FLASH_PAGE_SIZE);
            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        } else { /* Write */
            location[0] = 0x02; /* Write command */

            status = FX3_FlashSpiWaitForStatus();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine(CyFalse);
            status = CyU3PSpiTransmitWords(location, 4);
            if (status != CY_U3P_SUCCESS) {
                CyU3PDebugPrint(2, "SPI WRITE command failed\r\n");
                CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            status = CyU3PSpiTransmitWords(buffer, FLASH_PAGE_SIZE);

            if (status != CY_U3P_SUCCESS) {
            	CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
                return status;
            }

            CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine(CyTrue);
        }

        byteAddress += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
        pageCount--;

        if (!spiFastRead)
            CyU3PThreadSleep(15);
    }

    return CY_U3P_SUCCESS;
}

/* Function to erase FX3 SPI flash sectors. */
CyU3PReturnStatus_t FX3_FlashSpiEraseSector(CyBool_t isErase, uint8_t sector)
{
    uint32_t temp = 0;
    uint8_t  location[4], rdBuf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    location[0] = 0x06;  /* Write enable. */

    CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    if (isErase)
    {
        location[0] = FX3_FLASH_CMD_SECTOR_ERASE; /* Sector erase. */
        temp        = sector * FLASH_SECTOR_SIZE;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 4);
        CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
    }

    location[0] = 0x05; /* RDSTATUS */
    do {
    	CyU3PGpioSetValue (FX3_FLASH1_SS, CyFalse);//CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 1);
        status = CyU3PSpiReceiveWords(rdBuf, 1);
        CyU3PGpioSetValue (FX3_FLASH1_SS, CyTrue);//CyU3PSpiSetSsnLine (CyTrue);
    } while(rdBuf[0] & 1);

    return status;
}
