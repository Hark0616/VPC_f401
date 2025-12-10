// #include "stm32f10x.h"                  // Device header // <-- Comentado: Reemplazado por main.h para el entorno HAL
#include "main.h" // <-- Añadido: Cabecera principal para STM32CubeIDE
#include "vpc3.h"
// --- Nueva implementación DelayUs usando DWT (precisa y compatible con HAL) ---
static void DelayUs(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = (SystemCoreClock / 1000000U) * us;
	while ((DWT->CYCCNT - start) < ticks) { }
}

// Declaración externa para el handle de SPI1 generado por CubeIDE
extern SPI_HandleTypeDef hspi1;


/*---------------------------------------------------------------------------*/
/* function: Vpc3Write                                                       */
/*---------------------------------------------------------------------------*/
/**
 * @brief Write a byte to VPC3+.
 *
 * @attention This function is only necessary with VPC3+S in SPI- or IIC-Mode!
 *
 * @param[in]wAddress Address in VPC3+
 * @param[in]bData Data
 */

void Vpc3Write(VPC3_ADR wAddress, uint8_t bData )
{

	/* ********************************************************************************** */
	/* (HAL - Hardware Abstraction Layer)                                    */
	/* ********************************************************************************** */
	uint8_t upperByte = (uint8_t)(wAddress >> 8);
	uint8_t lowerByte = (uint8_t)wAddress;
	uint8_t tx_buffer[4];

	// Las funciones DpAppl_... son stubs vacíos por ahora, se mantienen.
	DpAppl_DisableInterruptVPC3Channel1();

	// Bajar el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
	DelayUs(10); // Pequeña demora como en el original

	// Preparar el buffer de transmisión
	tx_buffer[0] = WRITE_BYTE;
	tx_buffer[1] = upperByte;
	tx_buffer[2] = lowerByte;
	tx_buffer[3] = bData;

	// Transmitir el comando, la dirección y los datos en una sola llamada
	HAL_SPI_Transmit(&hspi1, tx_buffer, 4, HAL_MAX_DELAY);

	// Subir el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
	DelayUs(10); // Pequeña demora como en el original

	DpAppl_EnableInterruptVPC3Channel1();
	
}//void Vpc3Write( VPC3_ADR wAddress, uint8_t bData )



/*---------------------------------------------------------------------------*/
/* function: Vpc3Read                                                        */
/*---------------------------------------------------------------------------*/
/**
 * @brief Read one byte from VPC3+.
 *
 * @attention This function is only necessary with VPC3+S in SPI- or IIC-Mode!
 *
 * @param[in]wAddress Address in VPC3+
 * @return value of wAddress
 */

uint8_t Vpc3Read( VPC3_ADR wAddress )
{
	

	/* ********************************************************************************** */
	/* (HAL - Hardware Abstraction Layer)                                    */
	/* ********************************************************************************** */
	uint8_t upperByte = (uint8_t)(wAddress >> 8);
	uint8_t lowerByte = (uint8_t)wAddress;
	uint8_t tx_buffer[4] = {READ_BYTE, upperByte, lowerByte, 0x00}; // Comando, Addr High, Addr Low, Dummy para recibir
	uint8_t rx_buffer[4];

	DpAppl_DisableInterruptVPC3Channel1();

	// Bajar el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
	DelayUs(10);

	// Transmitir comando y dirección, y recibir la respuesta al mismo tiempo
	HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 4, HAL_MAX_DELAY);

	// Subir el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
	DelayUs(10);

	DpAppl_EnableInterruptVPC3Channel1();

	// El dato útil es el último byte recibido, que corresponde a la transmisión del byte dummy
	return rx_buffer[3];
	
}//uint8_t Vpc3Read( VPC3_ADR wAddress )




/*---------------------------------------------------------------------------*/
/* function: Vpc3MemSet                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Fill block of VPC3+ memory.
 *
 * @param[in]wAddress Address of the block of memory to fill.
 * @param[in]bValue  Value to be set.
 * @param[in]wLength Number of bytes to be set to the value.
 */

void Vpc3MemSet( VPC3_ADR wAddress, uint8_t bValue, uint16_t wLength )
{
	/* ********************************************************************************** */
	/* (HAL - Hardware Abstraction Layer)                                    */
	/* ********************************************************************************** */
	uint8_t upperByte = (uint8_t)(wAddress >> 8);
	uint8_t lowerByte = (uint8_t)wAddress;
	uint8_t cmd_addr_buffer[3] = {WRITE_ARRAY, upperByte, lowerByte};

	DpAppl_DisableInterruptVPC3Channel1();

	// Bajar el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
	DelayUs(10);

	// 1. Enviar el comando y la dirección
	HAL_SPI_Transmit(&hspi1, cmd_addr_buffer, 3, HAL_MAX_DELAY);

	// 2. Enviar el valor repetidamente
	for(uint16_t i = 0; i < wLength; i++){
		HAL_SPI_Transmit(&hspi1, &bValue, 1, HAL_MAX_DELAY);
	}

	// Subir el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
	DelayUs(10);

	DpAppl_EnableInterruptVPC3Channel1();

}//void Vpc3MemSet( VPC3_ADR wAddress, uint8_t bValue, uint16_t wLength )


/*---------------------------------------------------------------------------*/
/* function: Vpc3MemCmp                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Compare two blocks of VPC3+ memory.
 *
 * @param[in]pToVpc3Memory1 Pointer to block of memory.
 * @param[in]pToVpc3Memory2 Pointer to block of memory.
 * @param[in]wLength Number of bytes to compare.
 * @return 0 Indicates that the contents of both memory blocks are equal.
 * @return 1 Indicates that the contents of both memory blocks are not equal.
 */

uint8_t Vpc3MemCmp( VPC3_UNSIGNED8_PTR pToVpc3Memory1, VPC3_UNSIGNED8_PTR pToVpc3Memory2, uint16_t wLength )
{
   /** @todo Add your own code here! */

uint8_t bRetValue;
uint16_t i;

   bRetValue = 0;
   for( i = 0; i < wLength; i++ )
   {
      if( Vpc3Read( (VPC3_ADR)(uintptr_t)pToVpc3Memory1++ ) != Vpc3Read( (VPC3_ADR)(uintptr_t)pToVpc3Memory2++ ) )
      {
         bRetValue = 1;
         break;
      }//if( Vpc3Read( (VPC3_ADR)pToVpc3Memory1++ ) != Vpc3Read( (VPC3_ADR)pToVpc3Memory2++ ) )
   }//for( i = 0; i < wLength; i++ )

   return bRetValue;
}//uint8_t Vpc3MemCmp( VPC3_UNSIGNED8_PTR pToVpc3Memory1, VPC3_UNSIGNED8_PTR pToVpc3Memory2, uint16_t wLength )






/*---------------------------------------------------------------------------*/
/* function: CopyFromVpc3                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Copy block of memory from VPC3+.
 *
 * @param[in]pLocalMemory Pointer to the destination array where the content is to be copied.
 * @param[in]pToVpc3Memory Pointer to the source of data to be copied.
 * @param[in]wLength Number of bytes to copy.
 */
void CopyFromVpc3( MEM_UNSIGNED8_PTR pLocalMemory, VPC3_UNSIGNED8_PTR pToVpc3Memory, uint16_t wLength)
{
	

	/* ********************************************************************************** */
	/* Nuevo codigo (HAL - Hardware Abstraction Layer)                                    */
	/* ********************************************************************************** */
	uint16_t wAddress = (VPC3_ADR)(uintptr_t)pToVpc3Memory;
	uint8_t upperByte = (uint8_t)(wAddress >> 8);
	uint8_t lowerByte = (uint8_t)wAddress;
	uint8_t tx_buffer[3] = {READ_ARRAY, upperByte, lowerByte};

	DpAppl_DisableInterruptVPC3Channel1();

	// Bajar el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
	DelayUs(10);

	// 1. Enviar el comando de lectura de array y la dirección
	HAL_SPI_Transmit(&hspi1, tx_buffer, 3, HAL_MAX_DELAY);

	// 2. Recibir los datos del VPC3+
	HAL_SPI_Receive(&hspi1, pLocalMemory, wLength, HAL_MAX_DELAY);

	// Subir el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
	DelayUs(10);

	DpAppl_EnableInterruptVPC3Channel1();

}//void CopyFromVpc3( MEM_UNSIGNED8_PTR pLocalMemory, VPC3_UNSIGNED8_PTR pToVpc3Memory, uint16_t wLength )






/*---------------------------------------------------------------------------*/
/* function: CopyToVpc3                                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Copy block of memory to VPC3+.
 *
 * @param[in]pToVpc3Memory Pointer to the destination array where the content is to be copied.
 * @param[in]pLocalMemory Pointer to the source of data to be copied.
 * @param[in]wLength Number of bytes to copy.
 */

void CopyToVpc3(VPC3_UNSIGNED8_PTR pToVpc3Memory, MEM_UNSIGNED8_PTR pLocalMemory, uint16_t wLength)
{
	uint16_t wAddress = (VPC3_ADR)(uintptr_t)pToVpc3Memory;
	uint8_t upperByte = (uint8_t)(wAddress >> 8);
	uint8_t lowerByte = (uint8_t)wAddress;
	uint8_t cmd_addr_buffer[3] = {WRITE_ARRAY, upperByte, lowerByte};

	DpAppl_DisableInterruptVPC3Channel1();

	// Bajar el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
	DelayUs(10);

	// 1. Enviar el comando de escritura de array y la dirección
	HAL_SPI_Transmit(&hspi1, cmd_addr_buffer, 3, HAL_MAX_DELAY);

	// 2. Enviar el bloque de datos desde la memoria local
	HAL_SPI_Transmit(&hspi1, pLocalMemory, wLength, HAL_MAX_DELAY);

	// Subir el pin Chip Select (NSS)
	HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
	DelayUs(10);

	DpAppl_EnableInterruptVPC3Channel1();

}//void CopyToVpc3( VPC3_UNSIGNED8_PTR pToVpc3Memory, MEM_UNSIGNED8_PTR pLocalMemory, uint16_t wLength )


/*---------------------------------------------------------------------------*/
/* function: DpAppl_SetResetVPC3Channel1                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Set VPC3+ reset (activate reset).
 *
 * @attention The VPC3+ reset is HIGH-ACTIVE!
 * To put VPC3+S in reset state, this pin must be HIGH.
 *
 */
void DpAppl_SetResetVPC3Channel1( void )
{
   // VPC3+S reset is HIGH-ACTIVE: HIGH = reset active, LOW = normal operation
   HAL_GPIO_WritePin(VPC3_RESET_GPIO_Port, VPC3_RESET_Pin, GPIO_PIN_SET);

}//void DpAppl_SetResetVPC3Channel1( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_ClrResetVPC3Channel1                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Clear VPC3+ reset (release from reset).
 *
 * @attention The VPC3+ reset is HIGH-ACTIVE!
 * To release VPC3+S from reset, this pin must be LOW.
 *
 */
void DpAppl_ClrResetVPC3Channel1( void )
{
   // VPC3+S reset is HIGH-ACTIVE: HIGH = reset active, LOW = normal operation
   HAL_GPIO_WritePin(VPC3_RESET_GPIO_Port, VPC3_RESET_Pin, GPIO_PIN_RESET);

}//void DpAppl_ClrResetVPC3Channel1( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableInterruptVPC3Channel1                              */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable VPC3+ interrupt.
 */
void DpAppl_EnableInterruptVPC3Channel1( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_EnableInterruptVPC3Channel1( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableInterruptVPC3Channel1                             */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable VPC3+ interrupt.
 *
 */
void DpAppl_DisableInterruptVPC3Channel1( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_DisableInterruptVPC3Channel1( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableInterruptVPC3Sync                                  */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable VPC3+ isochronous interrupt.
 *
 * @attention Is only supported from VPC3+S!
 *
 */
void DpAppl_EnableInterruptVPC3Sync( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_EnableInterruptVPC3Sync( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableInterruptVPC3Sync                                 */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable VPC3+ isochronous interrupt.
 *
 * @attention Is only supported from VPC3+S!
 *
 */
void DpAppl_DisableInterruptVPC3Sync( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_DisableInterruptVPC3Sync( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_EnableAllInterrupts                                      */
/*---------------------------------------------------------------------------*/
/**
 * @brief Enable all microcontroller interrupts.
 *
 */
void DpAppl_EnableAllInterrupts( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_EnableAllInterrupts( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_DisableAllInterrupts                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Disable all microcontroller interrupts.
 *
 */
void DpAppl_DisableAllInterrupts( void )
{
   /** @todo Add your own code here! */
}//void DpAppl_DisableAllInterrupts( void )

/*---------------------------------------------------------------------------*/
/* function: Vpc3Wait_1ms                                                    */
/*---------------------------------------------------------------------------*/
/**
 * @brief Wait some time.
 *
 */
void Vpc3Wait_1ms( void )
{
   // Original: DelayMs(1);
   // Nuevo: Usa la función de la HAL que ya está configurada por CubeIDE.
   HAL_Delay(1);

}//void Vpc3Wait_1ms( void )

