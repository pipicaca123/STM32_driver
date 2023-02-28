/*
 * stm32f303xx_SPI.c
 *
 *  Created on: 2023年2月11日
 *      Author: Lou Yi Cheng
 */

#include "stm32f303xx_SPI.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle){
	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	// configure the SPI_CR1 register
	uint32_t TempReg = 0x00;

	// 1. configure the device mode
	TempReg |= (1 << SPI_CR1_MSTR);

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		TempReg &= ~(1 << SPI_CR1_BIDIMODE); //BIDI mode should be cleared
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		TempReg |= (1 << SPI_CR1_BIDIMODE);		//BIDI mode should be set
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		TempReg &= ~(1 << SPI_CR1_BIDIMODE);	//BIDI mode should be cleared
		//RXONLY bit must be set, or SCLK won't create CLK signal.
		TempReg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	TempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. Configure the DFF (stm32f3: CRCL bit)
	TempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL);

	// 5. configure the CPOL
	TempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6. configure the CPHA
	TempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 7. configure the SSM
	TempReg |= (pSPIHandle->SPIConfig.SPI_SSM <<SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 |= TempReg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}

/*
 * SPI_GetFlagStatus: test SPI current bit is set(1) or reset(0)
 * */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL )){
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
