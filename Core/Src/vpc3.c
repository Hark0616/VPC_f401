// #include "stm32f10x.h"                  // Device header // <-- Comentado: Reemplazado por main.h para el entorno HAL
#include "main.h" // <-- Añadido: Cabecera principal para STM32CubeIDE
#include "vpc3.h"
#include <stdio.h>

// External UART handle for diagnostics
extern UART_HandleTypeDef huart1;

// --- Nueva implementación DelayUs usando DWT (precisa y compatible con HAL) ---
static void DelayUs(uint32_t us)
{
	// Verificar que DWT esté habilitado
	if((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
		// Si DWT no está habilitado, usar un delay aproximado basado en loops
		volatile uint32_t loops = (SystemCoreClock / 1000000U) * us / 4;
		while(loops--) { __NOP(); }
		return;
	}
	
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = (SystemCoreClock / 1000000U) * us;
	while ((DWT->CYCCNT - start) < ticks) { }
}

// Declaración externa para el handle de SPI1 generado por CubeIDE
extern SPI_HandleTypeDef hspi1;

/*---------------------------------------------------------------------------*/
/* function: Vpc3_SpiDiagnostic                                              */
/*---------------------------------------------------------------------------*/
/**
 * @brief Diagnóstico del SPI para verificar comunicación con VPC3+S
 * @return 0 si todo OK, código de error si hay problemas
 */
uint8_t Vpc3_SpiDiagnostic(void)
{
    char buf[80];
    int n;
    uint8_t tx_data[4];
    uint8_t rx_data[4];
    HAL_StatusTypeDef status;
    
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] Iniciando diagnostico SPI...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    // Verificar estado del SPI
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] SPI1 State: %d, ErrorCode: 0x%lX\r\n", 
                 (int)hspi1.State, hspi1.ErrorCode);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    // Probar envío simple
    tx_data[0] = READ_BYTE;  // Comando de lectura
    tx_data[1] = 0x00;       // Dirección alta
    tx_data[2] = 0x3F;       // Dirección baja (Status Register High)
    tx_data[3] = 0x00;       // Dummy byte para recibir respuesta
    
    HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_RESET);
    DelayUs(10);
    
    status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 4, 100);
    
    HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);
    DelayUs(10);
    
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] HAL_SPI Status: %d\r\n", (int)status);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] TX: [%02X %02X %02X %02X]\r\n", 
                 tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] RX: [%02X %02X %02X %02X]\r\n", 
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    // Analizar respuesta
    if(status != HAL_OK) {
        n = snprintf(buf, sizeof(buf), "[SPI_DIAG] ERROR: HAL_SPI fallo!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
        return 1;
    }
    
    // El byte recibido en rx_data[3] es el Status High del VPC3
    // Debería contener el ASIC Type (AT) en los bits superiores
    // AT_VPC3S = 0x19 para VPC3+S
    uint8_t status_h = rx_data[3];
    
    if(status_h == 0xFF) {
        n = snprintf(buf, sizeof(buf), "[SPI_DIAG] WARN: RX=0xFF - Posible MISO no conectado o VPC3 en reset\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
        return 2;
    }
    
    if(status_h == 0x00) {
        n = snprintf(buf, sizeof(buf), "[SPI_DIAG] WARN: RX=0x00 - VPC3 no responde, verificar conexiones\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
        return 3;
    }
    
    // Verificar ASIC Type (bits 7:3 del Status High)
    uint8_t asic_type = (status_h >> 3) & 0x1F;
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] ASIC Type detectado: 0x%02X\r\n", asic_type);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    
    // VPC3+S ASIC Type es 0x03 (AT_VPC3S = 0x19 = 0b00011001)
    // Los bits [7:3] = 0x03
    if(asic_type == 0x03) {
        n = snprintf(buf, sizeof(buf), "[SPI_DIAG] OK: VPC3+S detectado correctamente!\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
        return 0;
    }
    
    n = snprintf(buf, sizeof(buf), "[SPI_DIAG] WARN: ASIC Type inesperado, continuando...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    return 0;  // Continuar de todas formas
}


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
	/* CORREGIDO: wAddress es ahora dirección absoluta en shadow RAM (0x2000XXXX) */
	/* Necesitamos convertirla a offset relativo (0x0000-0x0FFF) para el VPC3 */
	extern uint8_t vpc3_shadow_ram[];
	uint16_t vpc3Offset = (uint16_t)(wAddress - (VPC3_ADR)(uintptr_t)vpc3_shadow_ram);
	uint8_t upperByte = (uint8_t)(vpc3Offset >> 8);
	uint8_t lowerByte = (uint8_t)vpc3Offset;
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
	/* CORREGIDO: wAddress es ahora dirección absoluta en shadow RAM (0x2000XXXX) */
	/* Necesitamos convertirla a offset relativo (0x0000-0x0FFF) para el VPC3 */
	extern uint8_t vpc3_shadow_ram[];
	uint16_t vpc3Offset = (uint16_t)(wAddress - (VPC3_ADR)(uintptr_t)vpc3_shadow_ram);
	uint8_t upperByte = (uint8_t)(vpc3Offset >> 8);
	uint8_t lowerByte = (uint8_t)vpc3Offset;
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
	/* CORREGIDO: wAddress es ahora dirección absoluta en shadow RAM (0x2000XXXX) */
	/* Necesitamos convertirla a offset relativo (0x0000-0x0FFF) para el VPC3 */
	extern uint8_t vpc3_shadow_ram[];
	uint16_t vpc3Offset = (uint16_t)(wAddress - (VPC3_ADR)(uintptr_t)vpc3_shadow_ram);
	uint8_t upperByte = (uint8_t)(vpc3Offset >> 8);
	uint8_t lowerByte = (uint8_t)vpc3Offset;
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
   /* CORREGIDO: Calcular offset relativo al shadow RAM para direcciones VPC3 */
   extern uint8_t vpc3_shadow_ram[];
   uint8_t bRetValue;
   uint16_t i;
   VPC3_ADR addr1, addr2;

   bRetValue = 0;
   for( i = 0; i < wLength; i++ )
   {
      /* Convertir punteros de shadow RAM a offsets para el VPC3 */
      addr1 = (VPC3_ADR)((uintptr_t)pToVpc3Memory1 - (uintptr_t)vpc3_shadow_ram);
      addr2 = (VPC3_ADR)((uintptr_t)pToVpc3Memory2 - (uintptr_t)vpc3_shadow_ram);
      
      if( Vpc3Read( addr1 ) != Vpc3Read( addr2 ) )
      {
         bRetValue = 1;
         break;
      }
      pToVpc3Memory1++;
      pToVpc3Memory2++;
   }

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
	/* CORREGIDO: Calcular offset relativo al shadow RAM, no dirección absoluta */
	/* pToVpc3Memory apunta a shadow RAM (0x2000XXXX), pero VPC3 espera offset (0x0000-0x0FFF) */
	extern uint8_t vpc3_shadow_ram[];
	uint16_t wAddress = (uint16_t)((uintptr_t)pToVpc3Memory - (uintptr_t)vpc3_shadow_ram);
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
	/* CORREGIDO: Calcular offset relativo al shadow RAM, no dirección absoluta */
	/* pToVpc3Memory apunta a shadow RAM (0x2000XXXX), pero VPC3 espera offset (0x0000-0x0FFF) */
	extern uint8_t vpc3_shadow_ram[];
	uint16_t wAddress = (uint16_t)((uintptr_t)pToVpc3Memory - (uintptr_t)vpc3_shadow_ram);
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
 * @brief Set VPC3+ reset.
 *
 * @attention The VPC3+ reset is high-active!
 *
 */
void DpAppl_SetResetVPC3Channel1( void )
{
   // Original: SET_RESET_PIN;LO PONGO EN RESET PARA PROBAR YA QUE EL PUERTO PARECE ESTAR CONFIGURADO AL REVES
   HAL_GPIO_WritePin(VPC3_RESET_GPIO_Port, VPC3_RESET_Pin, GPIO_PIN_SET);

}//void DpAppl_SetResetVPC3Channel1( void )

/*---------------------------------------------------------------------------*/
/* function: DpAppl_ClrResetVPC3Channel1                                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief Clear VPC3+ reset.
 *
 * @attention The VPC3+ reset is high-active!
 *
 */
void DpAppl_ClrResetVPC3Channel1( void )
{
   // Original: RESET_RESET_PIN; LO PONGO EN SET PARA PROBAR YA QUE EL PUERTO PARECE ESTAR CONFIGURADO AL REVES
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

