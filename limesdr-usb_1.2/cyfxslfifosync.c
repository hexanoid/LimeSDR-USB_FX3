/*
 ## Cypress USB 3.0 Platform source file (cyfxslfifosync.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2011,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the Slave FIFO Synchronous mode example */

/*
   This example comprises of two USB bulk endpoints. A bulk OUT endpoint acts as the
   producer of data from the host. A bulk IN endpoint acts as the consumer of data to
   the host. Appropriate vendor class USB enumeration descriptors with these two bulk
   endpoints are implemented.

   The GPIF configuration data for the Synchronous Slave FIFO operation is loaded onto
   the appropriate GPIF registers. The p-port data transfers are done via the producer
   p-port socket and the consumer p-port socket.

   This example implements two DMA Channels in MANUAL mode one for P to U data transfer
   and one for U to P data transfer.

   The U to P DMA channel connects the USB producer (OUT) endpoint to the consumer p-port
   socket. And the P to U DMA channel connects the producer p-port socket to the USB 
   consumer (IN) endpoint.

   Upon every reception of data in the DMA buffer from the host or from the p-port, the
   CPU is signalled using DMA callbacks. There are two DMA callback functions implemented
   each for U to P and P to U data paths. The CPU then commits the DMA buffer received so
   that the data is transferred to the consumer.

   The DMA buffer size for each channel is defined based on the USB speed. 64 for full
   speed, 512 for high speed and 1024 for super speed. CY_FX_SLFIFO_DMA_BUF_COUNT in the
   header file defines the number of DMA buffers per channel.

   The constant CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT in the header file is used to
   select 16bit or 32bit GPIF data bus configuration.
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
//#include "cyu3uart.h"
#include "cyfxslfifosync.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include <cyu3gpio.h>
#include "stdint.h"

/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
//#include "cyfxgpif_syncsf.h"
#include "cyfxgpif2config.h"
#include "cyfxgpif2config_16b.h"

#include "cyu3gpio.h"
#include "cyu3i2c.h"
#include "cyu3spi.h"
#include "LMS64C_protocol.h"
#include "lime_sdr-usb_brd.h"
#include "spi_flash_lib.h"

#include <stdio.h>

//get info
#define FW_VER				0

#define sbi(p,n) ((p) |= (1UL << (n)))
#define cbi(p,n) ((p) &= ~(1 << (n)))

#define sadd16(a, b)  (uint16_t)( ((uint32_t)(a)+(uint32_t)(b)) > 0xffff ? 0xffff : ((a)+(b)))
#define sadd32(a, b)  (uint32_t)( ((uint64_t)(a)+(uint64_t)(b)) > 0xffffffff ? 0xffffffff : ((a)+(b)))

#define TRUE			CyTrue
#define FALSE			CyFalse


#define LED_WINK_PERIOD		18
#define LED_BLINK1_PERIOD	20
#define LED_BLINK2_PERIOD	10

//CMD_PROG_MCU
#define PROG_EEPROM 1
#define PROG_SRAM	2
#define BOOT_MCU	3

///CMD_PROG_MCU
#define MCU_CONTROL_REG	0x02
#define MCU_STATUS_REG	0x03
#define MCU_FIFO_WR_REG	0x04

#define MAX_MCU_RETRIES	30

enum {LED_MODE_OFF, LED_MODE_ON, LED_MODE_WINK};

uint8_t test, block, cmd_errors, glEp0Buffer[64], glEp0Buffer_Rx[64], glEp0Buffer_Tx[64] __attribute__ ((aligned (32))); //4096

tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet*)glEp0Buffer_Tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet*)glEp0Buffer_Rx;

int flash_page = 0, flash_page_data_cnt = 0, flash_data_cnt_free = 0, flash_data_counter_to_copy = 0;
unsigned char flash_page_data[FLASH_PAGE_SIZE], LED_mode = {LED_MODE_OFF}, LED_timeout, temp_byte, dac_val, need_fx3_reset = CyFalse;
uint8_t temp_status, MCU_retries;


//FPGA conf
unsigned long int last_portion, current_portion, fpga_data;
unsigned char data_cnt, sc_brdg_data[255];
tBoard_Config_FPGA *Board_Config_FPGA = (tBoard_Config_FPGA*) flash_page_data;

//MYRIAD_FW
unsigned char finish_flash_page;
unsigned short flash_word, flash_byte;

uint16_t phy_err_cnt, lnk_err_cnt;
uint32_t phy_err_cnt_total = 0, lnk_err_cnt_total = 0;

CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;   /* DMA Channel handle for P2U transfer. */

uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */
CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */
uint8_t burstLength =0;

unsigned long int fpga_byte, config_size = 256, fpga_bitstream_size;

extern CyU3PReturnStatus_t CyU3PUsbSetTxSwing (uint32_t swing);

//functions prototypes
void Modify_BRDSPI16_Reg_bits (unsigned short int SPI_reg_addr, unsigned char MSB_bit, unsigned char LSB_bit, unsigned short int new_bits_data);
void Delay_us (unsigned int us);
void Set_LED_mode (unsigned char mode);
unsigned char Check_many_blocks (unsigned char block_size);
void Configure_LM75 (void);
void Control_TCXO_DAC (unsigned char oe, unsigned char *data);
void Control_TCXO_ADF (unsigned char oe, unsigned char *data);
CyU3PReturnStatus_t Reconfigure_GPIF_16b (void);
void Wait_till_SC18B20_busy (void);
void Reconfigure_SPI_for_LMS (void);
void Reconfigure_SPI_for_Flash (void);



/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}


/* I2c initialization . */
CyU3PReturnStatus_t
CyFxI2cInit ()
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = 400000;//400 kHz
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    return status;
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxSlFifoApplnStart (
        void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            burstLength=1;

            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            burstLength=16;
            break;

        default:
            //CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
#ifdef STREAM_IN_OUT
    epCfg.burstLen = burstLength;
#else
    epCfg.burstLen = BURST_LEN;
#endif
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Create a DMA AUTO channel for U2P transfer.
        * DMA size is set based on the USB speed. */

	dmaCfg.size  = DMA_BUF_SIZE* size;
	dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT_U_2_P;
	dmaCfg.prodSckId = CY_FX_PRODUCER_USB_SOCKET;
	dmaCfg.consSckId = CY_FX_CONSUMER_PPORT_SOCKET;
	dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
	/* Enabling the callback for produce event. */
	dmaCfg.notification = 0;
	dmaCfg.cb = NULL;
	dmaCfg.prodHeader = 0;
	dmaCfg.prodFooter = 0;
	dmaCfg.consHeader = 0;
	dmaCfg.prodAvailCount = 0;

	apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoUtoP,
		   CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
	if (apiRetStatus != CY_U3P_SUCCESS)
	{
	   //CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
	   CyFxAppErrorHandler(apiRetStatus);
	}

	/* Create a DMA AUTO channel for P2U transfer. */
	dmaCfg.size  = DMA_BUF_SIZE*size; //increase buffer size for higher performance
	dmaCfg.count = CY_FX_SLFIFO_DMA_BUF_COUNT_P_2_U; // increase buffer count for higher performance
	dmaCfg.prodSckId = CY_FX_PRODUCER_PPORT_SOCKET;
	dmaCfg.consSckId = CY_FX_CONSUMER_USB_SOCKET;
	dmaCfg.cb = NULL;
	apiRetStatus = CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,
		   CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Set DMA channel transfer size. */
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Update the status flag. */
    glIsApplnActive = CyTrue;
    CyU3PGpioSetValue (59, CyFalse);
}

/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void
CyFxSlFifoApplnStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    glIsApplnActive = CyFalse;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleSlFifoUtoP);
    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t
CyFxSlFifoApplnUSBSetupCB (
        uint32_t setupdat0,
        uint32_t setupdat1
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex;
    CyBool_t isHandled = CyFalse;

    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == CY_FX_EP_PRODUCER)
                {
                    CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
                }

                if (wIndex == CY_FX_EP_CONSUMER)
                {
                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
                }

                CyU3PUsbStall (wIndex, CyFalse, CyTrue);
                isHandled = CyTrue;
            }
        }
    }



    /* Handle supported vendor requests. */
     if (bType == CY_U3P_USB_VENDOR_RQT)
     {
         isHandled = CyTrue;
         uint8_t   I2C_Addr;
     	CyU3PI2cPreamble_t preamble;

         switch (bRequest)
         {

         case 0xC0: //read
        	Set_LED_mode (LED_MODE_WINK);
         	CyU3PUsbSendEP0Data (64, glEp0Buffer_Tx);
         	if(need_fx3_reset) CyU3PDeviceReset(CyFalse); //hard fx3 reset
         	break;

         case 0xC1: //write

        	Set_LED_mode (LED_MODE_ON);
         	CyU3PUsbGetEP0Data (64, glEp0Buffer_Rx, NULL);

         	////LMS64C protocol
         	memset (glEp0Buffer_Tx, 0, sizeof(glEp0Buffer_Tx)); //fill whole tx buffer with zeros
         	cmd_errors = 0;

     		LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
     		LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
     		LMS_Ctrl_Packet_Tx->Header.Periph_ID = LMS_Ctrl_Packet_Rx->Header.Periph_ID;
     		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;

     		switch(LMS_Ctrl_Packet_Rx->Header.Command)
     		{
 				case CMD_GET_INFO:

 					LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
 					LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
 					LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

 					//read BSN from eeprom
 					I2C_Addr = EEPROM_I2C_ADDR;

 					//read byte
					preamble.length = 4;

					I2C_Addr &= ~(1 << 0);//write addr
					preamble.buffer[0] = I2C_Addr;

					preamble.buffer[1] = 0; //addr hi
					preamble.buffer[2] = 0; //addr lo

					I2C_Addr |= 1 << 0;	//read addr

					preamble.buffer[3] = I2C_Addr;
					preamble.ctrlMask  = 0x0004;

					if( CyU3PI2cReceiveBytes (&preamble, &LMS_Ctrl_Packet_Tx->Data_field[5], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_WR_INFO:

 					if (LMS_Ctrl_Packet_Rx->Header.Data_blocks != 1)
 					{
 						LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
 						break;
 					}

 					//write BSN to eeprom
 					I2C_Addr = EEPROM_I2C_ADDR;

 					preamble.length = 3;

					I2C_Addr &= ~(1 << 0);//write addr
					preamble.buffer[0] = I2C_Addr;

					preamble.buffer[1] = 0; //addr hi
					preamble.buffer[2] = 0; //addr lo

					preamble.ctrlMask  = 0x0000;

					if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[5], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

					CyU3PThreadSleep (5); //wait till EEPROM finish writing

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_LMS_RST:

 					switch (LMS_Ctrl_Packet_Rx->Data_field[0])
 					{
 						case LMS_RST_DEACTIVATE:
 							Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); //high level
 							break;

 						case LMS_RST_ACTIVATE:
 							Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); //low level
 							break;

 						case LMS_RST_PULSE:
 							Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 0); //low level
 							Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_RESET, LMS1_RESET, 1); //high level
 							break;

 						default:
 							cmd_errors++;
 							break;
 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 	 				case CMD_LMS7002_WR:
 	 					if(Check_many_blocks (4)) break;
 	 					Reconfigure_SPI_for_LMS ();

 	 					I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

 	 					Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

 	        			//write byte
 	        	        preamble.length    = 1;
 	        	        preamble.buffer[0] = I2C_Addr; //write h70;
 	        	        preamble.buffer[1] = 0xF1; //Clear Interrupt
 	        	        preamble.ctrlMask  = 0x0000;

 	        	        sc_brdg_data[0] = 0xF1; //Clear Interrupt

 	        	        CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

 	 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 	 					{
 	 						//write reg addr
 	 						sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
 	 					}

 	        			//write byte
 	        	        preamble.length    = 2;
 	        	        preamble.buffer[0] = I2C_Addr; //write h70;
 	        	        preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
 	        	        preamble.ctrlMask  = 0x0000;

 	        	        if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

 	        	        //CyU3PThreadSleep (3);
 	        	        Wait_till_SC18B20_busy ();

 	        	        Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

 	 					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
 	 					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 	 					break;

 	 				case CMD_LMS7002_RD:
 	 					if(Check_many_blocks (4)) break;
 	 					Reconfigure_SPI_for_LMS ();

 	 					I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

 	 					Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

 	        			//write byte
 	        	        preamble.length    = 1;
 	        	        preamble.buffer[0] = I2C_Addr; //write h70;
 	        	        preamble.buffer[1] = 0xF1; //Clear Interrupt
 	        	        preamble.ctrlMask  = 0x0000;
 	        	        sc_brdg_data[0] = 0xF1; //Clear Interrupt
 	        	        CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

 	        			//write byte
 	        	        preamble.length    = 2;
 	        	        preamble.buffer[0] = I2C_Addr; //write h70;
 	        	        preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
 	        	        preamble.ctrlMask  = 0x0000;

 	 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 	 					{
 	 						//write reg addr
 	 						cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit

 	 						sc_brdg_data[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr MSB
 	 						sc_brdg_data[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr LSB

 	 						sc_brdg_data[2 + (block * 4)] = 0x00; //dummy byte for spi reading
 	 						sc_brdg_data[3 + (block * 4)] = 0x00; //dummy byte for spi reading
 	 					}

 	        	        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

 	  				    //CyU3PThreadSleep (3);
 	        	        Wait_till_SC18B20_busy ();


 	     		        //read bytes from I2C buffer

 	     		        I2C_Addr |= 1 << 0;	//read addr

 	     		        preamble.length    = 1;
 	     		        preamble.buffer[0] = I2C_Addr;
 	     		        preamble.ctrlMask  = 0x0000;


 	     		        if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

 	 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 	 					{
 	 						LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr
 	 						LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr

 	 						//read reg data
 	 						LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = sc_brdg_data[(block * 4) + 2]; //reg data MSB
 	 						LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = sc_brdg_data[(block * 4) + 3]; //reg data LSB
 	 					}

 	        	        CyU3PThreadSleep (1); ///????
 	 					//Wait_till_SC18B20_busy ();

 	        	        Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

 	 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 	 					break;

 				case CMD_BRDSPI16_WR:
 					if(Check_many_blocks (4)) break;

					Reconfigure_SPI_for_LMS ();

 					I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

        			//write byte
        	        preamble.length    = 1;
        	        preamble.buffer[0] = I2C_Addr; //write h70;
        	        preamble.buffer[1] = 0xF1; //Clear Interrupt
        	        preamble.ctrlMask  = 0x0000;

        	        sc_brdg_data[0] = 0xF1; //Clear Interrupt

        	        CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

 					//Modify_BRDSPI16_Reg_bits (0x14, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
 						//write reg addr
 						sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); //set write bit
 					}

        		  	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

        			//write byte
        	        preamble.length    = 2;
        	        preamble.buffer[0] = I2C_Addr; //write h70;
        	        preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
        	        preamble.ctrlMask  = 0x0000;

        	        if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

        	        //CyU3PThreadSleep (3);
        	        Wait_till_SC18B20_busy ();

 					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
 					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

 					break;

 				case CMD_BRDSPI16_RD:

 					if(Check_many_blocks (4)) break;

 					Reconfigure_SPI_for_LMS ();

 					I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

        			//write byte
        	        preamble.length    = 1;
        	        preamble.buffer[0] = I2C_Addr; //write h70;
        	        preamble.buffer[1] = 0xF1; //Clear Interrupt
        	        preamble.ctrlMask  = 0x0000;

        	        sc_brdg_data[0] = 0xF1; //Clear Interrupt

        	        CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


 					I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

        			//write byte
        	        preamble.length    = 2;
        	        preamble.buffer[0] = I2C_Addr; //write h70;
        	        preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
        	        preamble.ctrlMask  = 0x0000;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
 						//write reg addr
 						cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7);  //clear write bit

 						sc_brdg_data[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr MSB
 						sc_brdg_data[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr LSB

 						sc_brdg_data[2 + (block * 4)] = 0x00; //dummy byte for spi reading
 						sc_brdg_data[3 + (block * 4)] = 0x00; //dummy byte for spi reading
 					}

 					if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0) != CY_U3P_SUCCESS)  cmd_errors++;

 					Wait_till_SC18B20_busy ();

     		        //read byte

     		        I2C_Addr |= 1 << 0;	//read addr

     		        preamble.length    = 1;
     		        preamble.buffer[0] = I2C_Addr;
     		        preamble.ctrlMask  = 0x0000;

     		        if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4*LMS_Ctrl_Packet_Rx->Header.Data_blocks, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
 						LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]; //reg addr
 						LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]; //reg addr

 						//read reg data
 						LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = sc_brdg_data[(block * 4) + 2]; //reg data MSB
 						LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = sc_brdg_data[(block * 4) + 3]; //reg data LSB
 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;


 				case CMD_ADF4002_WR:
 					if(Check_many_blocks (3)) break;

 					Control_TCXO_DAC (0, NULL); //set DAC out to three-state

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
 						Control_TCXO_ADF (1, &LMS_Ctrl_Packet_Rx->Data_field[0 + (block*3)]); //write data to ADF
 					}

 					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_INVALID_PERIPH_ID_CMD;
 						else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_SI5351_WR:
 					if(Check_many_blocks (2)) break;

        		  	I2C_Addr = SI5351_I2C_ADDR;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
            			//write byte
            	        preamble.length    = 2;
            	        preamble.buffer[0] = I2C_Addr; //write h70;
            	        preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[block * 2]; //reg to write
            	        preamble.ctrlMask  = 0x0000;

            	        if( CyU3PI2cTransmitBytes (&preamble, &LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
 					}

 					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
 					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_SI5351_RD:
 					if(Check_many_blocks (2)) break;

 					I2C_Addr = SI5351_I2C_ADDR;

 					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
 					{
        		        //read byte
        		        preamble.length = 3;

        		        I2C_Addr &= ~(1 << 0);//write addr
        		        preamble.buffer[0] = I2C_Addr;//0xE0; //write h70;

        		        preamble.buffer[1] = LMS_Ctrl_Packet_Rx->Data_field[block]; //reg to read

        		        I2C_Addr |= 1 << 0;	//read addr

        		        preamble.buffer[2] = I2C_Addr;//0xE1; //read h70
        		        preamble.ctrlMask  = 0x0002;

        		        if( CyU3PI2cReceiveBytes (&preamble, &LMS_Ctrl_Packet_Tx->Data_field[1 + block * 2], 1, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

 						LMS_Ctrl_Packet_Tx->Data_field[block * 2] = LMS_Ctrl_Packet_Rx->Data_field[block];
 					}

 					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
 					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_SSTREAM_RST: //fifo_rst

 					/*
 					 * 0 - FX3 reset
 					 * 1 - FPGA reset active
 					 * 2 - FPGA reset inactive
 					 * 3 - FPGA stream start
 					 * 4 - FPGA stream stop
 					 */
 					switch(LMS_Ctrl_Packet_Rx->Data_field[0])
 					{
 						case 0: //FX3 reset
 		 					CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
 		                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
 		                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
 		                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);

 		                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
 		                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
 		                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
 		                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
 							break;

 					}

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
 					break;

 				case CMD_MEMORY_WR:
					current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
					data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

 					if((LMS_Ctrl_Packet_Rx->Data_field[10] == 0) && (LMS_Ctrl_Packet_Rx->Data_field[11] == 1)) ///TARGET = 1 (FX33)
 					{
 						switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
 						{
							case 0:
								need_fx3_reset = CyTrue;
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								break;

 							case 1: //PROG_MODE = 1 (destroy current firmware in flash)

 								if(current_portion == 0)//beginning
								{
									Reconfigure_GPIF_16b ();
									Reconfigure_SPI_for_Flash ();

									FX3_FlashSpiEraseSector(CyTrue, 0);
									LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								}
								else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								break;

							case 2: //PROG_MODE = 2 (write FW to flash)

								if(current_portion == 0)//beginning
								{
									//check CY signature
									if (!(LMS_Ctrl_Packet_Rx->Data_field[24] == 'C' && LMS_Ctrl_Packet_Rx->Data_field[25] == 'Y')) //CY Signature ok?
									{
										LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
										break;
									}

									flash_page = 0;
									flash_page_data_cnt = 0;
									flash_data_counter_to_copy = 0;
									fpga_byte = 0;

									Reconfigure_GPIF_16b ();
									Reconfigure_SPI_for_Flash ();
								}

								flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;

								if (flash_data_cnt_free > 0)
								{
									if (flash_data_cnt_free > data_cnt)
										flash_data_counter_to_copy = data_cnt; //copy all data if fits to free page space
									else
										flash_data_counter_to_copy = flash_data_cnt_free; //copy only amount of data that fits in to free page size

									memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], flash_data_counter_to_copy);

									flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
									flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;

									if (data_cnt == 0)//all bytes transmitted, end of programming
									{
										if (flash_page_data_cnt > 0)
											flash_page_data_cnt = FX3_FLASH_PAGE_SIZE; //finish page
									}

									flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								if (flash_page_data_cnt >= FX3_FLASH_PAGE_SIZE) //write data to flash
								{
									if ((flash_page % (FX3_FLASH_SECTOR_SIZE/FX3_FLASH_PAGE_SIZE)) == 0) //need to erase sector? reached number of pages in block?
										status = FX3_FlashSpiEraseSector(CyTrue, flash_page/(FX3_FLASH_SECTOR_SIZE/FX3_FLASH_PAGE_SIZE));

									status = FX3_FlashSpiTransfer(flash_page, FX3_FLASH_PAGE_SIZE, flash_page_data, CyFalse);//write to flash

									flash_page++;
									flash_page_data_cnt = 0;
									flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								//if not all bytes written to flash page
								if (data_cnt > flash_data_counter_to_copy)
								{
									flash_data_counter_to_copy = data_cnt - flash_data_counter_to_copy;

									memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt);

									flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
									flash_data_cnt_free = FX3_FLASH_PAGE_SIZE - flash_page_data_cnt;
								}

								fpga_byte = fpga_byte + data_cnt;

								if (fpga_byte <= FX3_SIZE) //correct bitream size?
								{
									if (data_cnt == 0)//end of programming
									{
									}

									LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
								}
								else //not correct firmware size
									LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

								break;

							default:
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
								break;
						}
					}
 					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

 					break;

				case CMD_ALTERA_FPGA_GW_WR: //FPGA active serial

					current_portion = (LMS_Ctrl_Packet_Rx->Data_field[1] << 24) | (LMS_Ctrl_Packet_Rx->Data_field[2] << 16) | (LMS_Ctrl_Packet_Rx->Data_field[3] << 8) | (LMS_Ctrl_Packet_Rx->Data_field[4]);
					data_cnt = LMS_Ctrl_Packet_Rx->Data_field[5];

					switch(LMS_Ctrl_Packet_Rx->Data_field[0])//prog_mode
					{
						/*
						Programming mode:

						0 - Bitstream to FPGA
						1 - Bitstream to Flash
						2 - Bitstream from Flash
						*/

						case 1: //write data to Flash from PC
							//Flash_ID();
							/*Reconfigure_SPI_for_Flash ();*/

							if(current_portion == 0)//beginning
							{
								flash_page = 0;
								flash_page_data_cnt = 0;
								flash_data_counter_to_copy = 0;
								fpga_byte = 0;

								//write byte
			 				    preamble.length    = 1;
			 				    preamble.buffer[0] = MAX7322_I2C_ADDR; //MAX7322_I2C_ADDR
			 				    preamble.ctrlMask  = 0x0000;
			 				    sc_brdg_data[0] = 0x41; //AS_SW = 0, NCONFIG = 1 .. AS_SS = 1
			 				    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							    CyU3PThreadSleep (1);
							}

							flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

							if (flash_data_cnt_free > 0)
							{
								if (flash_data_cnt_free > data_cnt)
									flash_data_counter_to_copy = data_cnt; //copy all data if fits to free page space
								else
									flash_data_counter_to_copy = flash_data_cnt_free; //copy only amount of data that fits in to free page size

								memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], flash_data_counter_to_copy);

								flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
								flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;

								if (data_cnt == 0)//all bytes transmitted, end of programming
								{
									if (flash_page_data_cnt > 0)
										flash_page_data_cnt = FLASH_PAGE_SIZE; //finish page
								}

								flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
							}

							if (flash_page_data_cnt >= FLASH_PAGE_SIZE)
							{
								if ((flash_page % (FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE)) == 0) //need to erase sector? reached number of pages in block?
									status = FlashSpiEraseSector(CyTrue, FLASH_LAYOUT_FPGA_BITSTREAM + flash_page/(FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE));

								status = FlashSpiTransfer((FLASH_LAYOUT_FPGA_BITSTREAM * FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE) + flash_page, FLASH_PAGE_SIZE, flash_page_data, CyFalse);//write to flash

								flash_page++;
								flash_page_data_cnt = 0;
								flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
							}

							//if not all bytes written to flash page
							if (data_cnt > flash_data_counter_to_copy)
							{
								flash_data_counter_to_copy = data_cnt - flash_data_counter_to_copy;

								memcpy(&flash_page_data[flash_page_data_cnt], &LMS_Ctrl_Packet_Rx->Data_field[24], data_cnt);

								flash_page_data_cnt = flash_page_data_cnt + flash_data_counter_to_copy;
								flash_data_cnt_free = FLASH_PAGE_SIZE - flash_page_data_cnt;
							}

							fpga_byte = fpga_byte + data_cnt;

							if (fpga_byte <= FPGA_SIZE) //correct bitream size?
							{
								if (data_cnt == 0)//end of programming
								{
									/*Board_Config_FPGA->Bitream_size = fpga_byte; //bitsream ok
									Board_Config_FPGA->Autoload = 1; //autoload
									status = FlashSpiEraseSector(CyTrue, FLASH_LAYOUT_FPGA_METADATA); //erase sector for FPGA autoload metadata
									FlashSpiTransfer((FLASH_LAYOUT_FPGA_METADATA * FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE), FLASH_PAGE_SIZE, flash_page_data, CyFalse);//write to flash*/

									//write byte
								    preamble.length    = 1;
								    preamble.buffer[0] = MAX7322_I2C_ADDR; //MAX7322_I2C_ADDR
								    preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
								    preamble.ctrlMask  = 0x0000;
								    sc_brdg_data[0] = 0xC1; //AS_SW = 1, NCONFIG = 1 .. AS_SS = 1
								    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;
								}

								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
							}
							else //not correct bitsream size
								LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;

							break;

						case 2: //configure FPGA from flash

							//write byte
						    preamble.length    = 1;
						    preamble.buffer[0] = MAX7322_I2C_ADDR; //MAX7322_I2C_ADDR
						    preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
						    preamble.ctrlMask  = 0x0000;
						    sc_brdg_data[0] = 0x81; //AS_SW = 1, NCONFIG = 0 .. AS_SS = 1
						    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						    CyU3PThreadSleep (5);

							//write byte
						    preamble.length    = 1;
						    preamble.buffer[0] = MAX7322_I2C_ADDR; //MAX7322_I2C_ADDR
						    preamble.buffer[1] = 0xF7; //reg to write = GPIO Configuration
						    preamble.ctrlMask  = 0x0000;
						    sc_brdg_data[0] = 0xC1; //AS_SW = 1, NCONFIG = 1 .. AS_SS = 1
						    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;


						    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

							break;

						default:
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
							break;

					}

					break;

				case CMD_ANALOG_VAL_RD:

					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
					{
						signed short int converted_val;

						switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block)])//ch
						{
							case 0://TCXO DAC val

								LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
								LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x00; //RAW //unit, power

								LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = 0; //signed val, MSB byte
								LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = dac_val; //signed val, LSB byte
								break;

							case 1: //temperature
								I2C_Addr = LM75_I2C_ADDR; //LM75 I2C_ADDR

								//read byte
								preamble.length = 3;

								I2C_Addr &= ~(1 << 0);//write addr
								preamble.buffer[0] = I2C_Addr;

								preamble.buffer[1] = 0x00; //temperature

								I2C_Addr |= 1 << 0;	//read addr

								preamble.buffer[2] = I2C_Addr;
								preamble.ctrlMask  = 0x0002;

								if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

								converted_val = (((signed short int)sc_brdg_data[0]) << 8) + 0;//sc_brdg_data[1];
								converted_val = (converted_val/256)*10;

								if(sc_brdg_data[1]&0x80) converted_val = converted_val + 5;

								LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[block]; //ch
								LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = 0x50; //mC //unit, power

								LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = (converted_val >> 8); //signed val, MSB byte
								LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = converted_val; //signed val, LSB byte

								break;

							default:
								cmd_errors++;
								break;
						}
					}

					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

					break;

				case CMD_ANALOG_VAL_WR:
					if(Check_many_blocks (4)) break;

					for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
					{
						switch (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]) //do something according to channel
						{
							case 0: //TCXO DAC
								if (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0) //RAW units?
								{
									if(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)] == 0) //MSB byte empty?
									{
										Control_TCXO_ADF (0, NULL); //set ADF4002 CP to three-state

										//write data to DAC
										dac_val = LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)];
										Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val
									}
									else cmd_errors++;
								}
								else cmd_errors++;

								break;
							default:
								cmd_errors++;
								break;
						}
					}

					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
					break;

				case CMD_LMS_MCU_FW_WR:

					current_portion = LMS_Ctrl_Packet_Rx->Data_field[1];

					//check if portions are send in correct order
					if(current_portion != 0) //not first portion?
					{
						if(last_portion != (current_portion - 1)) //portion number increments?
						{
							LMS_Ctrl_Packet_Tx->Header.Status = STATUS_WRONG_ORDER_CMD;
							break;
						}
					}

	 				I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	 				Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 0); //Enable LMS's SPI

					if (current_portion == 0) //PORTION_NR = first fifo
					{

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

			        	//write byte
	        	        preamble.length    = 2;
	        	        preamble.buffer[0] = I2C_Addr; //write h70;
	        	        preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
	        	        preamble.ctrlMask  = 0x0000;

						//reset mcu
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB
						sc_brdg_data[3] = (0x00); //reg data LSB //8

	        	        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

			        	//write byte
	        	        preamble.length    = 2;
	        	        preamble.buffer[0] = I2C_Addr; //write h70;
	        	        preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
	        	        preamble.ctrlMask  = 0x0000;

						//set mode
						//write reg addr - mSPI_REG2 (Controls MCU input pins)
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_CONTROL_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB

						//reg data LSB
						switch (LMS_Ctrl_Packet_Rx->Data_field[0]) //PROG_MODE
						{
							case PROG_EEPROM:
								sc_brdg_data[3] = (0x01); //Programming both EEPROM and SRAM  //8
								if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
								Wait_till_SC18B20_busy ();
								break;

							case PROG_SRAM:
								sc_brdg_data[3] =(0x02); //Programming only SRAM  //8
								if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
								Wait_till_SC18B20_busy ();
								break;

							case BOOT_MCU:
								sc_brdg_data[3] = (0x03); //Programming both EEPROM and SRAM  //8
								if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;
								Wait_till_SC18B20_busy ();

								//write byte
								preamble.length    = 1;
								preamble.buffer[0] = I2C_Addr; //write h70;
								preamble.buffer[1] = 0xF1; //Clear Interrupt
								preamble.ctrlMask  = 0x0000;
								sc_brdg_data[0] = 0xF1; //Clear Interrupt
								CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


								//write byte
								preamble.length    = 2;
								preamble.buffer[0] = I2C_Addr; //write h70;
								preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
								preamble.ctrlMask  = 0x0000;

								sc_brdg_data[0] = (0x00); //reg addr MSB
								sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
								sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
								sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

								if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

								Wait_till_SC18B20_busy ();


								//read bytes from I2C buffer
								I2C_Addr |= 1 << 0;	//read addr

								preamble.length    = 1;
								preamble.buffer[0] = I2C_Addr;
								preamble.ctrlMask  = 0x0000;

								if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
								temp_status = sc_brdg_data[3]; //reg data LSB
								CyU3PThreadSleep (1); ///????

								goto BOOTING;

								break;
						}
					}

					MCU_retries = 0;

					//wait till EMPTY_WRITE_BUFF = 1
					while (MCU_retries < MAX_MCU_RETRIES)
					{
						//read status reg

						//spi read
						//write reg addr

						I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
						preamble.ctrlMask  = 0x0000;

						sc_brdg_data[0] = (0x00); //reg addr MSB
						sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
						sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
						sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

						if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();


						//read bytes from I2C buffer
						I2C_Addr |= 1 << 0;	//read addr

						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.ctrlMask  = 0x0000;

						if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
						temp_status = sc_brdg_data[3]; //reg data LSB
						CyU3PThreadSleep (1); ///????


						if (temp_status &0x01) break;

						MCU_retries++;
						Delay_us (30);
					}

					//write 32 bytes to FIFO
					for(block = 0; block < 32; block++)
					{

						I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

			        	//write byte
	        	        preamble.length    = 2;
	        	        preamble.buffer[0] = I2C_Addr; //write h70;
	        	        preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
	        	        preamble.ctrlMask  = 0x0000;

						//reset mcu
						sc_brdg_data[0] = (0x80); //reg addr MSB with write bit
						sc_brdg_data[1] = (MCU_FIFO_WR_REG); //reg addr LSB

						sc_brdg_data[2] = (0x00); //reg data MSB
						sc_brdg_data[3] = (LMS_Ctrl_Packet_Rx->Data_field[2 + block]); //reg data LSB //8

	        	        if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();

						temp_status=0;
						MCU_retries = 0;
					}

					MCU_retries = 0;

					//wait till EMPTY_WRITE_BUFF = 1
					while (MCU_retries < 500)
					{
						//read status reg

						//spi read
						//write reg addr

						I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

						//write byte
						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = 0xF1; //Clear Interrupt
						preamble.ctrlMask  = 0x0000;
						sc_brdg_data[0] = 0xF1; //Clear Interrupt
						CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


						//write byte
						preamble.length    = 2;
						preamble.buffer[0] = I2C_Addr; //write h70;
						preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
						preamble.ctrlMask  = 0x0000;

						sc_brdg_data[0] = (0x00); //reg addr MSB
						sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
						sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
						sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

						if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

						Wait_till_SC18B20_busy ();


						//read bytes from I2C buffer
						I2C_Addr |= 1 << 0;	//read addr

						preamble.length    = 1;
						preamble.buffer[0] = I2C_Addr;
						preamble.ctrlMask  = 0x0000;

						if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
						temp_status = sc_brdg_data[3]; //reg data LSB
						CyU3PThreadSleep (1); ///????

						if (temp_status &0x01) break;

						MCU_retries++;
						Delay_us (30);
					}


					if (current_portion  == 255) //PORTION_NR = last fifo
					{
						//chek programmed bit

						MCU_retries = 0;

						//wait till PROGRAMMED = 1
						while (MCU_retries < MAX_MCU_RETRIES)
						{
							//read status reg

							//spi read
							//write reg addr

							I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

							//write byte
							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr; //write h70;
							preamble.buffer[1] = 0xF1; //Clear Interrupt
							preamble.ctrlMask  = 0x0000;
							sc_brdg_data[0] = 0xF1; //Clear Interrupt
							CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


							//write byte
							preamble.length    = 2;
							preamble.buffer[0] = I2C_Addr; //write h70;
							preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
							preamble.ctrlMask  = 0x0000;

							sc_brdg_data[0] = (0x00); //reg addr MSB
							sc_brdg_data[1] = (MCU_STATUS_REG); //reg addr LSB
							sc_brdg_data[2] = 0x00;  //dummy byte for spi reading
							sc_brdg_data[3] = 0x00;  //dummy byte for spi reading

							if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

							Wait_till_SC18B20_busy ();


							//read bytes from I2C buffer
							I2C_Addr |= 1 << 0;	//read addr

							preamble.length    = 1;
							preamble.buffer[0] = I2C_Addr;
							preamble.ctrlMask  = 0x0000;

							if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
							temp_status = sc_brdg_data[3]; //reg data LSB
							CyU3PThreadSleep (1); ///????

							if (temp_status &0x40) break; //PROGRAMMED = 1

							MCU_retries++;
							Delay_us (30);
						}

						if (MCU_retries == MAX_MCU_RETRIES) cmd_errors++;
					}

					last_portion = current_portion; //save last portion number

					BOOTING:

					if(cmd_errors) LMS_Ctrl_Packet_Tx->Header.Status = STATUS_ERROR_CMD;
					else LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

					Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_LMS1_LMS2_CTRL, LMS1_SS, LMS1_SS, 1); //Disable LMS's SPI

					break;


 				default:

 					LMS_Ctrl_Packet_Tx->Header.Status = STATUS_UNKNOWN_CMD;
 					break;
     		}

     		Set_LED_mode (LED_MODE_WINK);
         	break;

             default:
                 /* This is unknown request. */
                 isHandled = CyFalse;
                 break;
         }

         /* If there was any error, return not handled so that the library will
          * stall the request. Alternatively EP0 can be stalled here and return
          * CyTrue. */
         if (status != CY_U3P_SUCCESS)
         {
             isHandled = CyFalse;
         }
     }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
CyFxSlFifoApplnUSBEventCB (
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
    )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            CyU3PUsbLPMDisable();
            /* Start the loop back function. */
            CyFxSlFifoApplnStart ();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the loop back function. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            break;

        default:
            break;
    }
    Set_LED_mode (LED_MODE_WINK);
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

void
gpif_error_cb(CyU3PPibIntrType cbType, uint16_t cbArg)
{

if(cbType==CYU3P_PIB_INTR_ERROR)
{
    switch (CYU3P_GET_PIB_ERROR_TYPE(cbArg))
    {
        case CYU3P_PIB_ERR_THR0_WR_OVERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR0_WR_OVERRUN");
        break;
        case CYU3P_PIB_ERR_THR1_WR_OVERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR1_WR_OVERRUN");
        break;
        case CYU3P_PIB_ERR_THR2_WR_OVERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR2_WR_OVERRUN");
        break;
        case CYU3P_PIB_ERR_THR3_WR_OVERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR3_WR_OVERRUN");
        break;

        case CYU3P_PIB_ERR_THR0_RD_UNDERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR0_RD_UNDERRUN");
        break;
        case CYU3P_PIB_ERR_THR1_RD_UNDERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR1_RD_UNDERRUN");
        break;
        case CYU3P_PIB_ERR_THR2_RD_UNDERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR2_RD_UNDERRUN");
        break;
        case CYU3P_PIB_ERR_THR3_RD_UNDERRUN:
        //CyU3PDebugPrint (4, "CYU3P_PIB_ERR_THR3_RD_UNDERRUN");
        break;

        default:
        //CyU3PDebugPrint (4, "No Error :%d\n ",CYU3P_GET_PIB_ERROR_TYPE(cbArg));
            break;
    }
}

}



/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void
CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "P-port Initialization failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    apiRetStatus = CyU3PGpifLoad (&CyFxGpifConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PGpifLoad failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /*CyU3PGpifSocketConfigure (0,CY_U3P_PIB_SOCKET_0,6,CyFalse,1);
    CyU3PGpifSocketConfigure (3,CY_U3P_PIB_SOCKET_3,6,CyFalse,1);*/

    CyU3PGpifSocketConfigure (0,CY_U3P_PIB_SOCKET_0,6,CyFalse,1); //n=6, watermark= n * 32/32 - 4
    CyU3PGpifSocketConfigure (3,CY_U3P_PIB_SOCKET_3,4,CyFalse,1); //n=4, watermark= n * 32/32 - 4

    /* Start the state machine. */
    apiRetStatus = CyU3PGpifSMStart (RESET,ALPHA_RESET);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PGpifSMStart failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Init the GPIO module */
	gpioClock.fastClkDiv = 2;
	gpioClock.slowClkDiv = 0;
	gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	gpioClock.clkSrc = CY_U3P_SYS_CLK;
	gpioClock.halfDiv = 0;

	apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
	if (apiRetStatus != 0)
	{
		/* Error Handling */
		//CyU3PDebugPrint (4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}

	gpioConfig.outValue = CyFalse;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_LED_R, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_LED_R, &gpioConfig);

	gpioConfig.outValue = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_LED_G, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_LED_G, &gpioConfig);

	gpioConfig.outValue = CyFalse;
	gpioConfig.inputEn = CyTrue;
	gpioConfig.driveLowEn = CyFalse;
	gpioConfig.driveHighEn = CyFalse;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (SC18B20_INT_GPIO, CyTrue);
	CyU3PGpioSetSimpleConfig(SC18B20_INT_GPIO, &gpioConfig);


    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    CyU3PUsbSetTxSwing(127); //TX Amplitude swing of the electrical signalling on the USB superspeed lines in 10 mV units. Should be less than 1.28V.

    /* callback to see if there is any overflow of data on the GPIF II side*/
    CyU3PPibRegisterCallback(gpif_error_cb,0xffff);

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);    

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        //CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

/* Entry function for the slFifoAppThread. */
void
SlFifoAppThread_Entry (
        uint32_t input)
{
	CyU3PDmaState_t state;
	uint32_t consXferCount, prodXferUtoPCount, prodXferCountUtoP_old, prodXferPtoUCount, prodXferCountPtoU_old;

    uint8_t data;
	CyU3PI2cPreamble_t preamble;

	//CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyFxI2cInit ();
    //CyU3PSpiInit(); //impossible to use spi in 32 bit GPIF mode

    /* Initialize the slave FIFO application */
    CyFxSlFifoApplnInit();

    Set_LED_mode (LED_MODE_WINK);

	//write byte
    preamble.length    = 1;
    preamble.buffer[0] = MAX7322_I2C_ADDR; //MAX7322_I2C_ADDR
    preamble.ctrlMask  = 0x0000;
    data = 0xC1; //AS_SW = 1, NCONFIG = 1 .. AS_SS = 1
    if( CyU3PI2cTransmitBytes (&preamble, &data, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//write default TCXO DAC value
	Control_TCXO_ADF (0, NULL); //set ADF4002 CP to three-state
	dac_val = 125; //default DAC value
	Control_TCXO_DAC (1, &dac_val); //enable DAC output, set new val

	Configure_LM75 (); //set LM75 configuration

    for (;;)
    {
        CyU3PThreadSleep (10);

		if (LED_mode != LED_MODE_OFF)
		{
			if (LED_timeout) LED_timeout--;

			if(LED_timeout == 0)
			{
				switch(LED_mode)
				{
					case LED_MODE_WINK:
						//CyU3PGpioSimpleSetValue (LED_GPIO, CyFalse); //turn off led
						CyU3PGpioSimpleSetValue (FX3_LED_R, CyFalse);
						CyU3PGpioSimpleSetValue (FX3_LED_G, CyTrue);

						LED_mode= LED_MODE_OFF;
						break;

					case LED_MODE_ON:
						break;

					default:
					case LED_MODE_OFF:
						break;
				}
			}
		}

		//detecting streams in both directions and indicate this
		prodXferCountUtoP_old = prodXferUtoPCount;
		prodXferCountPtoU_old = prodXferPtoUCount;
        CyU3PDmaChannelGetStatus(&glChHandleSlFifoUtoP, &state, &prodXferUtoPCount, &consXferCount);
        CyU3PDmaChannelGetStatus(&glChHandleSlFifoPtoU, &state, &prodXferPtoUCount, &consXferCount);

        if ((prodXferUtoPCount != prodXferCountUtoP_old) || (prodXferPtoUCount != prodXferCountPtoU_old)) if (LED_mode != LED_MODE_ON) Set_LED_mode (LED_MODE_WINK);
    }
}

/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);

    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
                          "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
                          SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

    /* Check the return code */
    if (retThrdCreate != 0)
    {
        /* Thread Creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

        /* setSysClk400 clock configurations */
        clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
        clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
        clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
        clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
        clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
        clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

    /* Initialize the device */
    status = CyU3PDeviceInit (&clkCfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large 
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
     * isDQ32Bit for 32-bit slave FIFO configuration. */
    io_cfg.useUart   = CyFalse;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyFalse;

#if (CY_FX_SLFIFO_GPIF_16_32BIT_CONF_SELECT == 0)
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_SPI_ONLY;
#else
    io_cfg.isDQ32Bit = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
#endif

    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;

    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

void Delay_us (unsigned int us)
{
	//CyU3PThreadSleep (1);
	unsigned int i;

	for (i = 0; i< (us*4); i++) {}
}

/**	Function to control LED mode. */
void Set_LED_mode (unsigned char mode)
{
	LED_mode = mode;//save new LED mode

	switch (LED_mode)
	{
		case LED_MODE_OFF:
			//CyU3PGpioSimpleSetValue (LED_GPIO, CyFalse); //turn off led
			CyU3PGpioSimpleSetValue (FX3_LED_R, CyFalse);
			CyU3PGpioSimpleSetValue (FX3_LED_G, CyTrue);
			break;

		case LED_MODE_ON:
			//CyU3PGpioSimpleSetValue (LED_GPIO, CyTrue); //turn on led
			CyU3PGpioSimpleSetValue (FX3_LED_R, CyTrue);
			CyU3PGpioSimpleSetValue (FX3_LED_G, CyFalse);
			break;

		case LED_MODE_WINK:
			//CyU3PGpioSimpleSetValue (LED_GPIO, CyTrue); //turn on led
			CyU3PGpioSimpleSetValue (FX3_LED_R, CyTrue);
			CyU3PGpioSimpleSetValue (FX3_LED_G, CyFalse);
			LED_timeout = LED_WINK_PERIOD; //set LED timeout
			break;
	}
}

/**	This function checks if all blocks could fit in data field.
*	If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks (unsigned char block_size)
{
	if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field)/block_size))
	{
		LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
		return TRUE;
	}
	else return FALSE;
	return FALSE;
}

/**
 *	@brief Function to configure LM75 temperature sensor (OS polarity, THYST, TOS)
 */
void Configure_LM75 (void)
{
    uint8_t   I2C_Addr, LM75_data[2];
	CyU3PI2cPreamble_t preamble;

	//configure LM75

	I2C_Addr = LM75_I2C_ADDR;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x01; //Configuration addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 0x04;//Configuration = OS polarity = 1, Comparator/int = 0, Shutdown = 0

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 1, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x02; //THYST addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 45;//THYST H
	LM75_data[1] = 0;//THYST L

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;

	//write byte
	preamble.length = 2;
	preamble.buffer[0] = I2C_Addr;
	preamble.buffer[1] = 0x03; //TOS addr
	preamble.ctrlMask  = 0x0000;

	LM75_data[0] = 55;//TOS H
	LM75_data[1] = 0;//TOS L

	if( CyU3PI2cTransmitBytes (&preamble, &LM75_data[0], 2, 0)  != CY_U3P_SUCCESS)  cmd_errors++;
}

/**
 *	@brief Function to control DAC for TCXO frequency control
 *	@param oe output enable control: 0 - output disabled, 1 - output enabled
 *	@param data pointer to DAC value (1 byte)
 */
void Control_TCXO_DAC (unsigned char oe, unsigned char *data) //controls DAC (AD5601)
{
	CyU3PI2cPreamble_t preamble;
	uint8_t   I2C_Addr;
	unsigned char DAC_data[2];

	Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_DAC_SS, TCXO_DAC_SS, 0); //select

	if (oe == 0) //set DAC out to three-state
	{
		DAC_data[0] = 0xC0; //POWER-DOWN MODE = THREE-STATE (MSB bits = 11) + MSB data
		DAC_data[1] = 0x00; //LSB data

		//Reconfigure_SPI_for_AD5601 ();

		//write-read SPI bytes using using I2C-SPI bridge
		I2C_Addr = SC18IS602B_I2C_ADDR;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0xF1; //Clear Interrupt
		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, &DAC_data[0], 2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();
	}
	else //enable DAC output, set new val
	{
		DAC_data[0] = (*data) >>2; //POWER-DOWN MODE = NORMAL OPERATION (MSB bits =00) + MSB data
		DAC_data[1] = (*data) <<6; //LSB data

		//Reconfigure_SPI_for_AD5601 ();

		//write-read SPI bytes using using I2C-SPI bridge
		I2C_Addr = SC18IS602B_I2C_ADDR;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;
		sc_brdg_data[0] = 0xF1; //Clear Interrupt
		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, &DAC_data[0], 2, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();
	}

	Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_DAC_SS, TCXO_DAC_SS, 1); //deselect
}

/**
 *	@brief Function to control ADF for TCXO frequency control
 *	@param oe output enable control: 0 - output disabled, 1 - output enabled
 *	@param data pointer to ADF data block (3 bytes)
 */
void Control_TCXO_ADF (unsigned char oe, unsigned char *data) //controls ADF4002
{
	CyU3PI2cPreamble_t preamble;
	uint8_t   I2C_Addr;
	unsigned char ADF_data[12], ADF_block;

	if (oe == 0) //set ADF4002 CP to three-state and MUX_OUT to DGND
	{
		ADF_data[0] = 0x1f;
		ADF_data[1] = 0x81;
		ADF_data[2] = 0xf3;
		ADF_data[3] = 0x1f;
		ADF_data[4] = 0x81;
		ADF_data[5] = 0xf2;
		ADF_data[6] = 0x00;
		ADF_data[7] = 0x01;
		ADF_data[8] = 0xf4;
		ADF_data[9] = 0x01;
		ADF_data[10] = 0x80;
		ADF_data[11] = 0x01;

		Reconfigure_SPI_for_LMS();

		//write data to ADF
		for(ADF_block = 0; ADF_block < 4; ADF_block++)
		{
			Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 0); //Select ADF

			I2C_Addr = SC18IS602B_I2C_ADDR;

			//write byte
			preamble.length    = 1;
			preamble.buffer[0] = I2C_Addr;
			preamble.buffer[1] = 0xF1; //Clear Interrupt
			preamble.ctrlMask  = 0x0000;

			sc_brdg_data[0] = 0xF1; //Clear Interrupt

			CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

			I2C_Addr = SC18IS602B_I2C_ADDR;

			//write byte
			preamble.length    = 2;
			preamble.buffer[0] = I2C_Addr;
			preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
			preamble.ctrlMask  = 0x0000;

			if( CyU3PI2cTransmitBytes (&preamble, &ADF_data[ADF_block*3], 3, 0) != CY_U3P_SUCCESS)  cmd_errors++;

			Wait_till_SC18B20_busy ();

			Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 1); //Deselect ADF
		}
	}
	else //set PLL parameters, 4 blocks must be written
	{
		Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 0); //Select ADF

		Reconfigure_SPI_for_LMS();

		I2C_Addr = SC18IS602B_I2C_ADDR;

		//write byte
		preamble.length    = 1;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = 0xF1; //Clear Interrupt
		preamble.ctrlMask  = 0x0000;

		sc_brdg_data[0] = 0xF1; //Clear Interrupt

		CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);

		I2C_Addr = SC18IS602B_I2C_ADDR;

		//write byte
		preamble.length    = 2;
		preamble.buffer[0] = I2C_Addr;
		preamble.buffer[1] = BRDG_SPI_DUMMY_SS;
		preamble.ctrlMask  = 0x0000;

		if( CyU3PI2cTransmitBytes (&preamble, data, 3, 0) != CY_U3P_SUCCESS)  cmd_errors++;

		Wait_till_SC18B20_busy ();

		Modify_BRDSPI16_Reg_bits (FPGA_SPI_REG_SS_CTRL, TCXO_ADF_SS, TCXO_ADF_SS, 1); //Deselect ADF
	}
}

CyU3PReturnStatus_t Reconfigure_GPIF_16b (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	CyU3PGpifDisable(CyTrue);
	//CyU3PDmaChannelReset(&glDmaChHandle);
	CyU3PPibDeInit();
	CyU3PLppDeInit(2); // Deinit SPI block

    io_cfg.useUart   = CyFalse;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_SPI_ONLY;

	/* No GPIOs are enabled. */
	io_cfg.gpioSimpleEn[0]  = 0;
	io_cfg.gpioSimpleEn[1]  = 0;
	io_cfg.gpioComplexEn[0] = 0;
	io_cfg.gpioComplexEn[1] = 0;

	status = CyU3PDeviceConfigureIOMatrix (&io_cfg);

    if (status != CY_U3P_SUCCESS)
    {
    	CyU3PDebugPrint (4, "IO Matrix config failed, Error code = %d\n", status);
    	CyFxAppErrorHandler(status);
    }

	//CyU3PPibInit(1, &pibClk);
	CyU3PGpifLoad(&CyFxGpifConfig_16b);
	//CyU3PDmaChannelSetXfer(&glDmaChHandle, 0);
	CyU3PGpifSMStart(0,0);

	gpioConfig.outValue = CyFalse;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_LED_R, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_LED_R, &gpioConfig);

	gpioConfig.outValue = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_LED_G, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_LED_G, &gpioConfig);

	gpioConfig.outValue = CyTrue;
	gpioConfig.inputEn = CyFalse;
	gpioConfig.driveLowEn = CyTrue;
	gpioConfig.driveHighEn = CyTrue;
	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
	CyU3PDeviceGpioOverride (FX3_FLASH1_SS, CyTrue);
	CyU3PGpioSetSimpleConfig(FX3_FLASH1_SS, &gpioConfig);

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

	return status;
}

void Modify_BRDSPI16_Reg_bits (unsigned short int SPI_reg_addr, unsigned char MSB_bit, unsigned char LSB_bit, unsigned short int new_bits_data)
{
	unsigned short int mask, SPI_reg_data;
	unsigned char bits_number;
	uint8_t MSB_byte, LSB_byte;

    uint8_t   I2C_Addr;
	CyU3PI2cPreamble_t preamble;

	//Reconfigure_SPI_for_LMS ();

	bits_number = MSB_bit - LSB_bit + 1;

	mask = 0xFFFF;

	//removing unnecessary bits from mask
	mask = mask << (16 - bits_number);
	mask = mask >> (16 - bits_number);

	new_bits_data &= mask; //mask new data

	new_bits_data = new_bits_data << LSB_bit; //shift new data

	mask = mask << LSB_bit; //shift mask
	mask =~ mask;//invert mask

	MSB_byte = (SPI_reg_addr >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_addr & 0xFF;


	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	//write byte
    preamble.length    = 1;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = 0xF1; //Clear Interrupt
    preamble.ctrlMask  = 0x0000;

    sc_brdg_data[0] = 0xF1; //Clear Interrupt

    CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);



	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	//write byte
	preamble.length    = 2;
	preamble.buffer[0] = I2C_Addr; //write h70;
	preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
	preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = MSB_byte;
	sc_brdg_data[1] = LSB_byte;
	sc_brdg_data[2] = 0x00; //dummy byte for spi reading
	sc_brdg_data[3] = 0x00; //dummy byte for spi reading

	if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

	//CyU3PThreadSleep (2);
	Wait_till_SC18B20_busy ();


	 //read byte
	 I2C_Addr |= 1 << 0;	//read addr

	 preamble.length    = 1;
	 preamble.buffer[0] = I2C_Addr;
	 preamble.ctrlMask  = 0x0000;

	 if( CyU3PI2cReceiveBytes (&preamble, &sc_brdg_data[0], 4, 0)  != CY_U3P_SUCCESS)  cmd_errors++;;

	//read reg data
	MSB_byte = sc_brdg_data[2]; //reg data MSB
	LSB_byte = sc_brdg_data[3]; //reg data LSB


	SPI_reg_data = (MSB_byte << 8) + LSB_byte; //read current SPI reg data

	//modify reg data
	SPI_reg_data &= mask;//clear bits
	SPI_reg_data |= new_bits_data; //set bits with new data

	//write reg addr
	MSB_byte = (SPI_reg_addr >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_addr & 0xFF;

	sbi(MSB_byte, 7); //set write bit


	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	//write byte
    preamble.length    = 1;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = 0xF1; //Clear Interrupt
    preamble.ctrlMask  = 0x0000;

    sc_brdg_data[0] = 0xF1; //Clear Interrupt

    CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 1, 0);


  	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	//write byte
    preamble.length    = 2;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = BRDG_SPI_FPGA_SS; //FPGA SS
    preamble.ctrlMask  = 0x0000;

	sc_brdg_data[0] = MSB_byte;
	sc_brdg_data[1] = LSB_byte;

	////write modified data back to SPI reg
	MSB_byte = (SPI_reg_data >> 8 ) & 0xFF;
	LSB_byte = SPI_reg_data & 0xFF;

	sc_brdg_data[2] = MSB_byte;
	sc_brdg_data[3] = LSB_byte;

    if( CyU3PI2cTransmitBytes (&preamble, &sc_brdg_data[0], 4, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    //CyU3PThreadSleep (2);
    Wait_till_SC18B20_busy ();
}

void Wait_till_SC18B20_busy (void)
{
	CyBool_t int_val;
	int a=0;

	CyU3PGpioSimpleGetValue (SC18B20_INT_GPIO, &int_val);

	a++;

	while (int_val == CyTrue) //wait till SPI transmission has been completed
	{
		Delay_us(10); //wait 10 us
		a++;

		if(a > 300) break; //wait up to 3 ms

		CyU3PGpioSimpleGetValue (SC18B20_INT_GPIO, &int_val);

	}
}

/** Reconfigures SPI to match the current serial port settings issued by the host. */
void Reconfigure_SPI_for_LMS(void)
{
    uint8_t   I2C_Addr, data;
	CyU3PI2cPreamble_t preamble;

  	I2C_Addr = SC18IS602B_I2C_ADDR;// SC18IS602B_I2C_ADDR

	//write byte
    preamble.length    = 2;
    preamble.buffer[0] = I2C_Addr; //write h70;
    preamble.buffer[1] = 0xF0; //reg to write = configure spi
    preamble.buffer[2] = 0x00;
    preamble.ctrlMask  = 0x0000;
    data = 0x00; // MSB word first, Mode 0 (CPOL = 0, CPHA = 0), 1.843 MHz
    if( CyU3PI2cTransmitBytes (&preamble, &data, 1, 0) != CY_U3P_SUCCESS)  cmd_errors++;

    ////CyU3PThreadSleep (1);
}

void Reconfigure_SPI_for_Flash(void)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyFalse;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyFalse;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_NONE;
    spiConfig.clock      = 50 *1000000; //50 MHz
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
}

/* [ ] */

