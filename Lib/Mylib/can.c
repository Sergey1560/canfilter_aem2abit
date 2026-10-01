#include "can.h"
#include "common_data.h"
#include "led.h"
#include "ws2812b.h"

volatile struct fdcan_sram_t * const fdcan_sram = (volatile struct fdcan_sram_t *)(APB1PERIPH_BASE + 0xA400UL);

static void get_msg(FDCAN_GlobalTypeDef *CAN, struct can_message_t* msg, uint8_t idx);
static volatile struct fdcan_mem_t *get_mem_base_addr(FDCAN_GlobalTypeDef *CAN);
static void fdcan_init(FDCAN_GlobalTypeDef *CAN, uint8_t can_bauderate);

static void fdcan1_filters(void);
static void fdcan2_filters(void);

static void aem2abit(struct can_message_t *aem, struct can_message_t *abit);

void can_hw_init(void){
/*
FDCAN1:
PB7 - CAN1 IO
PB8 - RX
PB9 - TX
PC13 - Silent


FDCAN2:
PB12 - RX
PB13 - TX
PB14 - Silent
*/
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN|RCC_AHB2ENR_GPIOCEN;
	
	/*********************************** FDCAN1 *****************************/
	/* PB8 RX, PB9 TX*/
	GPIOB->MODER &= ~(GPIO_MODER_MODER8|GPIO_MODER_MODER9);
    GPIOB->MODER |= (GPIO_MODER_MODER8_1|GPIO_MODER_MODER9_1);
	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9);
	
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDR_OSPEED8_Pos)|(S_VH << GPIO_OSPEEDR_OSPEED9_Pos);
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8|GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (9 << GPIO_AFRH_AFSEL8_Pos)|(9 << GPIO_AFRH_AFSEL9_Pos);
	
	//IO PB7
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0);
	CAN1_DISABLE_IO;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);

	//Silent PC13
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	GPIOC->MODER |= (GPIO_MODER_MODE13_0);
	CAN1_SILENT_ON;
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT13);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD13_1);

	/*********************************** FDCAN2 *****************************/
	/* PB12 RX, PB13 TX*/
	GPIOB->MODER &= ~(GPIO_MODER_MODER12|GPIO_MODER_MODER13);
    GPIOB->MODER |= (GPIO_MODER_MODER12_1|GPIO_MODER_MODER13_1);
	//Push-pull mode 0
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_12|GPIO_OTYPER_OT_13);
	
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12|GPIO_OSPEEDER_OSPEEDR13);
    GPIOB->OSPEEDR |= (S_VH << GPIO_OSPEEDR_OSPEED12_Pos)|(S_VH << GPIO_OSPEEDR_OSPEED13_Pos);
	
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL12|GPIO_AFRH_AFSEL13);
	GPIOB->AFR[1] |= (9 << GPIO_AFRH_AFSEL12_Pos)|(9 << GPIO_AFRH_AFSEL13_Pos);
	
	//Silent PB14
	GPIOB->MODER &= ~(GPIO_MODER_MODE14);
	GPIOB->MODER |= (GPIO_MODER_MODE14_0);
	CAN2_SILENT_ON;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT14);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD14);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD14_1);

	RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;
	//PLLQ, 85Mhz
	RCC->CCIPR |= (1 << RCC_CCIPR_FDCANSEL_Pos);

	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

	fdcan_init(FDCAN1,CAN_BD_500);
	fdcan_init(FDCAN2,CAN_BD_500);

	DEBUG("Can init done ");
}

static void fdcan_init(FDCAN_GlobalTypeDef *CAN, uint8_t can_bauderate){
	uint32_t can_psc = 0;

	DEBUG("Init FDCAN at 0x%0X",(uint32_t)CAN);

	volatile struct fdcan_mem_t *fdcan_mem = get_mem_base_addr(CAN);

	if(fdcan_mem == NULL){
		ERROR("Failed FDCAN mem addr");
		return;
	}

    CAN->CCCR |= FDCAN_CCCR_INIT;     //Initialize the CAN module
	while(!(CAN->CCCR & FDCAN_CCCR_INIT)){__NOP();};         // wait 
    
	CAN->CCCR   |= FDCAN_CCCR_CCE;     //CCE bit enabled to start configuration
    CAN->CCCR &= ~ (FDCAN_CCCR_FDOE);  //Classic CAN

	/* Clean-up message RAM */
	volatile uint32_t *mem = (volatile uint32_t *)fdcan_mem;
	for(uint32_t i = 0; i < sizeof(struct fdcan_mem_t) / sizeof(uint32_t); i++){
			mem[i] = 0;
	}

	switch (can_bauderate)
	{
	case CAN_BD_125:
		can_psc = 8;
		break;
	case CAN_BD_250:
		can_psc = 4;
		break;
	case CAN_BD_500:
		can_psc = 2;
		break;
	case CAN_BD_1000:
		can_psc = 1;
		break;
	
	default:
		ERROR("Unknown CAN BUDRATE value, set 500k");
		can_psc = 2;
		break;
	}

	/* Timings */
	CAN->NBTP = ((CAN_JW-1) << FDCAN_NBTP_NSJW_Pos)|\
				((can_psc-1) << FDCAN_NBTP_NBRP_Pos)|\
				((CAN_SEG1-1) << FDCAN_NBTP_NTSEG1_Pos)|\
				((CAN_SEG2-1) << FDCAN_NBTP_NTSEG2_Pos);


	if(CAN == FDCAN1){
		fdcan1_filters();
	}else if(CAN == FDCAN2){
		fdcan2_filters();
	}

	/* TX FIFO mode */
	CAN->TXBC  = 0;
					
	CAN->IE |= FDCAN_IE_RF0NE|FDCAN_IE_RF0FE|FDCAN_IE_BOE;
	CAN->ILE |= FDCAN_ILE_EINT0;

	CAN->CCCR   &= ~(FDCAN_CCCR_INIT);
	
	if(CAN == FDCAN1){
		CAN1_ENABLE_IO;
		CAN1_SILENT_OFF;
		NVIC_SetPriority(FDCAN1_IT0_IRQn,5);
		NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	}else if(CAN == FDCAN2){
		CAN2_SILENT_OFF;
		NVIC_SetPriority(FDCAN2_IT0_IRQn,5);
		NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
	}else if(CAN == FDCAN3){
		NVIC_SetPriority(FDCAN3_IT0_IRQn,5);
		NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
	}else{
		ERROR("Unknow FDCAN interface 0x%0X",(uint32_t)CAN);
	}
}

static volatile struct fdcan_mem_t *get_mem_base_addr(FDCAN_GlobalTypeDef *CAN){
	if(CAN == FDCAN1){
		return &fdcan_sram->fdcan1;
	}else if(CAN == FDCAN2){
		return &fdcan_sram->fdcan2;
	}else if(CAN == FDCAN3){
		return &fdcan_sram->fdcan3;
	}else{
		ERROR("Unknow FDCAN interface 0x%0X",(uint32_t)CAN);
		return NULL;
	}
}

static void get_msg(FDCAN_GlobalTypeDef *CAN, struct can_message_t* msg, uint8_t idx){
	volatile struct fdcan_mem_t *fdcan_mem = get_mem_base_addr(CAN);

	if(fdcan_mem == NULL){
		ERROR("Failed FDCAN mem addr");
		return;
	}

	if(idx > 2){
		ERROR("Index out ouf buffer");
		return;
	}

	volatile struct can_rx_fifo_element_t *fifo = &fdcan_mem->rx0_fifo[idx];

	if(fifo->b[0] & CAN_29BIT_FLAG){//29bit ID
		msg->ext_id = 1;
		msg->id = fifo->b[0] & 0x1FFFFFFF;
	}else{
		msg->ext_id = 0;
		msg->id  = (fifo->b[0] >> 18) & 0x7FF;
	}
	msg->padding = 0;
	msg->len = (fifo->b[1] >> 16) & 0xF;

	uint32_t *ptr = (uint32_t *)msg->msg;
	
	*ptr++ = fifo->b[2];
	*ptr = fifo->b[3];
}

int can_send(FDCAN_GlobalTypeDef *CAN, struct can_message_t *msg){
	volatile struct fdcan_mem_t *fdcan_mem = get_mem_base_addr(CAN);

	if(fdcan_mem == NULL){
		ERROR("Failed FDCAN mem addr");
		return -1;
	}

	#ifdef CAN_DEBUG
	//DEBUG("Count %d TFGI: %d TFQPI: %d TFFL: %d TFQF: %d",msg->msg[0],(CAN->TXFQS >> FDCAN_TXFQS_TFGI_Pos) & 0x3,(CAN->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 0x3, CAN->TXFQS & 0x03,(CAN->TXFQS >> FDCAN_TXFQS_TFQF_Pos) & 0x1);
	#endif

	if ((CAN->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
		#ifdef CAN_DEBUG
		ERROR("TX FIFO full");
		#endif
		return -1;
	};

	uint8_t tx_index= (CAN->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 0x3;

	if(tx_index > 2){
		ERROR("TX index out of buffer");
		return -1;
	}

	volatile struct can_tx_fifo_element_t *fifo = &fdcan_mem->can_tx_fifo[tx_index];

	if(msg->ext_id == 1){//29bit
		fifo->b[0] = CAN_29BIT_FLAG|(msg->id & 0x1FFFFFFF);
	}else{//11bit
		fifo->b[0] = ((msg->id & 0x7ff) << 18);
	}

	fifo->b[1] = ((msg->len & 0xF) << 16);  //Data size

	uint32_t *ptr = (uint32_t *)msg->msg;

	fifo->b[2] = *ptr++;
	fifo->b[3] = *ptr;

	CAN->TXBAR = (1UL << tx_index);
	return 0;   
}

static void fdcan2_filters(void){

	#ifdef CAN2_FILTERS
	struct can_ext_filter_element_t ext_filter_element = {0};
	volatile uint32_t *ext_filter = (volatile uint32_t *)&fdcan_sram->fdcan2.ext_filter;

	ext_filter_element.efid1 = AEM_CANID & 0x1FFFFFFF;
	ext_filter_element.efid2 = AEM_CANID & 0x1FFFFFFF;
	ext_filter_element.efec = CAN_STD_FILTER_SFEC_FIFO0;
	ext_filter_element.eft = CAN_STD_FILTER_SFT_DUAL;
	#ifdef CAN_DEBUG
	DEBUG("Ext Filter at 0x%0X",ext_filter);
	#endif
	*ext_filter++ = ext_filter_element.b0_value;
	*ext_filter++ = ext_filter_element.b1_value;

	uint32_t ext_filters_count = ((ext_filter - (volatile uint32_t *)&fdcan_sram->fdcan2.ext_filter) * sizeof(*ext_filter)) / sizeof(struct can_ext_filter_element_t);
	DEBUG("Total ext filters: %d",ext_filters_count);

	if(ext_filters_count > 8){
		ERROR("To many ext filters: %d of 8",ext_filters_count);
		ext_filters_count = 8;
	};

	/* AEM bus: standard and remote frames are not used */
	FDCAN2->RXGFC = FDCAN_RXGFC_ANFS|FDCAN_RXGFC_RRFS|FDCAN_RXGFC_RRFE;

	if(ext_filters_count > 0){
		DEBUG("Apply %d ext filters",ext_filters_count);
		FDCAN2->RXGFC |= (ext_filters_count << FDCAN_RXGFC_LSE_Pos)|FDCAN_RXGFC_ANFE;
	}

	#else
	FDCAN2->RXGFC = 0;
	#endif
}

static void fdcan1_filters(void){

	#ifdef CAN1_FILTERS

	struct std_filter_t filter_element = {0};
	volatile uint32_t *filter = (volatile uint32_t *)&fdcan_sram->fdcan1.std_filter;

	filter_element.sfid1 = 0;
	filter_element.sfid2 =  0x7FF;
	filter_element.sfec = CAN_STD_FILTER_SFEC_REJECT;
	filter_element.sft = CAN_STD_FILTER_SFT_CLASSIC;
	#ifdef CAN_DEBUG
	DEBUG("Filter at 0x%0X val 0x%0X",filter,filter_element.value);
	#endif
	*filter++ = filter_element.value;

	uint32_t std_filters_count = ((filter - (volatile uint32_t *)&fdcan_sram->fdcan1.std_filter) * sizeof(*filter)) / sizeof(struct std_filter_t);
	DEBUG("Total std filters: %d",std_filters_count);

	if(std_filters_count > 28){
		ERROR("To many std filters: %d of 28",std_filters_count);
		std_filters_count = 28;
	};

	/* ABIT bus: extended frames are not used */
	FDCAN1->RXGFC = FDCAN_RXGFC_ANFE;

	if(std_filters_count > 0){
		DEBUG("Apply %d std filters",std_filters_count);
		FDCAN1->RXGFC |= (std_filters_count << FDCAN_RXGFC_LSS_Pos)|FDCAN_RXGFC_ANFS;
	}
	
	#else
	FDCAN1->RXGFC = 0;
	#endif
}

void FDCAN1_IT0_IRQHandler(void) {
	FDCAN_GlobalTypeDef *CAN = FDCAN1;
	uint8_t rx_index;
	struct can_message_t msg;
	TRACE_ENTER_ISR;

	if((CAN->IR & FDCAN_IR_BO) != 0){
		/* Clear the flag first, otherwise the IRQ keeps re-firing */
		CAN->IR = FDCAN_IR_BO;
		if(CAN->PSR & FDCAN_PSR_BO){
			#ifdef CAN_DEBUG
			ERROR("FDCAN1 Enter Bus-off state");
			#endif

			/* Bus-off sets INIT; clearing it starts recovery (128 x 11 recessive bits) */
			if( (CAN->CCCR & FDCAN_CCCR_INIT) != 0){
				CAN->CCCR   &= ~(FDCAN_CCCR_INIT);
			}
		}
	}

	if((CAN->IR & FDCAN_IR_RF0F) != 0){
		WARNING("CAN1 RX IRQ FIFO full");
	};

	if((CAN->IR & (FDCAN_IR_RF0N|FDCAN_IR_RF0F)) != 0){
		/* Clear before draining: a frame arriving during the loop re-raises the IRQ */
		CAN->IR = FDCAN_IR_RF0N|FDCAN_IR_RF0F;
		while(CAN->RXF0S & FDCAN_RXF0S_F0FL_Msk){
			rx_index= (uint8_t)((CAN->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3);
			get_msg(CAN, &msg, rx_index);
			CAN->RXF0A = rx_index;
			#ifdef CAN_DEBUG
			//DEBUG("CAN1 msg ID(%d): 0x%0X  %0X:%0X:%0X:%0X:%0X:%0X:%0X:%0X",msg.ext_id,msg.id,msg.msg[0],msg.msg[1],msg.msg[2],msg.msg[3],msg.msg[4],msg.msg[5],msg.msg[6],msg.msg[7]);
			#endif
		}
	};

	if((CAN->IR & FDCAN_IR_RF0L) != 0){
			CAN->IR = FDCAN_IR_RF0L;
			WARNING("CAN1 RX IRQ, lost message");
	};

	TRACE_EXIT_ISR;
}

void FDCAN2_IT0_IRQHandler(void) {
	FDCAN_GlobalTypeDef *CAN = FDCAN2;
	uint8_t rx_index;
	struct can_message_t msg, abit;
	TRACE_ENTER_ISR;

	if((CAN->IR & FDCAN_IR_BO) != 0){
		/* Clear the flag first, otherwise the IRQ keeps re-firing */
		CAN->IR = FDCAN_IR_BO;
		if(CAN->PSR & FDCAN_PSR_BO){
			#ifdef CAN_DEBUG
			ERROR("FDCAN2 Enter Bus-off state");
			#endif

			/* Bus-off sets INIT; clearing it starts recovery (128 x 11 recessive bits) */
			if( (CAN->CCCR & FDCAN_CCCR_INIT) != 0){
				CAN->CCCR   &= ~(FDCAN_CCCR_INIT);
			}
		}
	}

	if((CAN->IR & FDCAN_IR_RF0F) != 0){
		WARNING("CAN2 RX IRQ FIFO full");
	};

	if((CAN->IR & (FDCAN_IR_RF0N|FDCAN_IR_RF0F)) != 0){
		/* Clear before draining: a frame arriving during the loop re-raises the IRQ */
		CAN->IR = FDCAN_IR_RF0N|FDCAN_IR_RF0F;
		while(CAN->RXF0S & FDCAN_RXF0S_F0FL_Msk){
			rx_index= (uint8_t)((CAN->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3);
			get_msg(CAN, &msg, rx_index);
			CAN->RXF0A = rx_index;
			#ifdef CAN_DEBUG
			//DEBUG("CAN2 msg ID(%d): 0x%0X  %0X:%0X:%0X:%0X:%0X:%0X:%0X:%0X",msg.ext_id,msg.id,msg.msg[0],msg.msg[1],msg.msg[2],msg.msg[3],msg.msg[4],msg.msg[5],msg.msg[6],msg.msg[7]);
			#endif
			if(msg.ext_id && (msg.id == AEM_CANID) && (msg.len == 8)){
				aem2abit(&msg,&abit);
				can_send(FDCAN1,&abit);
			}
		}
	};

	if((CAN->IR & FDCAN_IR_RF0L) != 0){
			CAN->IR = FDCAN_IR_RF0L;
			WARNING("CAN2 RX IRQ, lost message");
	};

	TRACE_EXIT_ISR;
}

void FDCAN3_IT0_IRQHandler(void) {
	FDCAN_GlobalTypeDef *CAN = FDCAN3;
	uint8_t rx_index;
	struct can_message_t msg = {0};
	TRACE_ENTER_ISR;

	if((CAN->IR & FDCAN_IR_BO) != 0){
		/* Clear the flag first, otherwise the IRQ keeps re-firing */
		CAN->IR = FDCAN_IR_BO;
		if(CAN->PSR & FDCAN_PSR_BO){
			#ifdef CAN_DEBUG
			ERROR("FDCAN3 Enter Bus-off state");
			#endif

			/* Bus-off sets INIT; clearing it starts recovery (128 x 11 recessive bits) */
			if( (CAN->CCCR & FDCAN_CCCR_INIT) != 0){
				CAN->CCCR   &= ~(FDCAN_CCCR_INIT);
			}
		}
	}

	if((CAN->IR & FDCAN_IR_RF0F) != 0){
		WARNING("CAN3 RX IRQ FIFO full");
	};

	if((CAN->IR & (FDCAN_IR_RF0N|FDCAN_IR_RF0F)) != 0){
		/* Clear before draining: a frame arriving during the loop re-raises the IRQ */
		CAN->IR = FDCAN_IR_RF0N|FDCAN_IR_RF0F;
		while(CAN->RXF0S & FDCAN_RXF0S_F0FL_Msk){
			rx_index= (uint8_t)((CAN->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3);
			get_msg(CAN, &msg, rx_index);
			CAN->RXF0A = rx_index;
			#ifdef CAN_DEBUG
			DEBUG("CAN3 msg ID(%d): 0x%0X  %0X:%0X:%0X:%0X:%0X:%0X:%0X:%0X",msg.ext_id,msg.id,msg.msg[0],msg.msg[1],msg.msg[2],msg.msg[3],msg.msg[4],msg.msg[5],msg.msg[6],msg.msg[7]);
			#endif
		}
	};

	if((CAN->IR & FDCAN_IR_RF0L) != 0){
			CAN->IR = FDCAN_IR_RF0L;
			WARNING("CAN3 RX IRQ, lost message");
	};

	TRACE_EXIT_ISR;
}

static void aem2abit(struct can_message_t *aem, struct can_message_t *abit){
	uint16_t aem_lambda;
	int16_t aem_oxygen;
	uint16_t abit_lambda;
	uint16_t abit_oxygen;
	uint8_t data_valid;
	uint8_t sensor_fault;
	
	/* AEM (doc/aem_30-0310.pdf): big-endian, lambda 0.0001/bit unsigned, O2 0.001 %/bit signed */
	aem_lambda = (aem->msg[0] << 8)|aem->msg[1];
	aem_oxygen = (int16_t)((aem->msg[2] << 8)|aem->msg[3]);

    data_valid = (aem->msg[6] >> 7) & 1;
	sensor_fault = (aem->msg[7] >> 6) & 1;

	/* ABIT ADLM 0x211 (doc/abit_adlm.pdf): little-endian, lambda 0.001/bit, O2 0.01 %/bit, both unsigned */
	abit_lambda = (uint16_t)(((uint32_t)aem_lambda + 5) / 10);
	abit_oxygen = (aem_oxygen > 0) ? (uint16_t)((aem_oxygen + 5) / 10) : 0;

//    printf("Aem Lambda: %d (0x%04X) Oxy: %d (0x%04X)\nAbt Lambda: %d (0x%04X) Oxy: %d (0x%04X)\n",aem_lambda,aem_lambda,aem_oxygen,aem_oxygen,abit_lambda,abit_lambda,abit_oxygen,abit_oxygen);

	*((uint32_t *)&abit->msg[0]) = 0;
	*((uint32_t *)&abit->msg[4]) = 0;

	abit->msgid = 0x211;
	abit->len = 8;

    abit->msg[0] = abit_lambda & 0xFF;
	abit->msg[1] = (abit_lambda >> 8) & 0xFF;
	
	abit->msg[2] = abit_oxygen & 0xFF;
	abit->msg[3] = (abit_oxygen >> 8) & 0xFF;

	abit->msg[7] = (data_valid && !sensor_fault) ? 1 : 0;	//bit 0 - sensor ready

	if(data_valid){
		ws2812b_set_rgb(0, LED_COLOR_GREEN);
	}else{
		ws2812b_set_rgb(0, LED_COLOR_OFF);
	}

	if(sensor_fault){
		ws2812b_set_rgb(1, LED_COLOR_RED);
	}else{
		ws2812b_set_rgb(1, LED_COLOR_OFF);
	}

	#ifdef CAN_DEBUG
	DEBUG("AEM L: %X O:%X EN: %d Abit L: %X O: %X", aem_lambda, aem_oxygen, data_valid, abit_lambda, abit_oxygen);
	DEBUG("ID 211 %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:",abit->msg[0],abit->msg[1],abit->msg[2],abit->msg[3],abit->msg[4],abit->msg[5],abit->msg[6],abit->msg[7]);
	#endif
}

