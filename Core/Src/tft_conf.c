/*
******************************************************************************
* @file    tft_conf.c
******************************************************************************
*/

#include "tft_conf.h"
#include "Components/stmpe811/stmpe811.h"
#include "lvgl/lvgl.h"

DMA_HandleTypeDef   hdma;
I2C_HandleTypeDef hi2c3;
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi5;
SDRAM_HandleTypeDef hsdram1;


static __IO uint16_t * my_fb = (__IO uint16_t*) (LCD_FRAME_BUFFER);

static lv_disp_drv_t disp_drv;
static int32_t x1_flush;
static int32_t y1_flush;
static int32_t x2_flush;
static int32_t y2_fill;
static int32_t y_fill_act;
static const lv_color_t * buf_to_flush;


static void tft_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
static bool touchpad_read(lv_indev_drv_t * drv, lv_indev_data_t *data);
static bool touchpad_get_xy(int16_t *x, int16_t *y);

static void SPI5_Init(void);

/**
  * @brief  Writes a byte to device.
  * @param  Value: value to be written
  */
static void SPIx_Write(uint16_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hspi5, (uint8_t *) &Value, 1, SPIx_TIMEOUT_MAX);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* error handle */
	  HAL_Delay(1);
  }
}

/**
  * @brief  Writes a value in a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @param  Value: The target register value to be written
  */
static void I2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2Cx_TIMEOUT_MAX);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Error handle */
    
  }
}

/**
  * @brief  Reads a register of the device through BUS.
  * @param  Addr: Device address on BUS Bus.
  * @param  Reg: The target register address to write
  * @retval Data read at register address
  */
static uint8_t I2Cx_ReadData(uint8_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;

  status = HAL_I2C_Mem_Read(&hi2c3, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX);

  /* Check the communication status */
  if (status != HAL_OK)
  {

  }
  return value;
}


/**
  * @brief  Reads multiple data on the BUS.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @param  pBuffer: pointer to read data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
static uint8_t I2Cx_ReadBuffer(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c3, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Length, I2Cx_TIMEOUT_MAX);

  /* Check the communication status */
  if (status == HAL_OK)
  {
    return 0;
  }
  else
  {
    // error handle
    return 1;
  }
}

static uint8_t Is_LCD_IO_Initialized = 0;

void LCD_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(Is_LCD_IO_Initialized == 0)
  {
    Is_LCD_IO_Initialized = 1; 

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure NCS in Output Push-Pull mode */
     __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin     = GPIO_PIN_13;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin     = GPIO_PIN_12;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure the LCD Control pins ----------------------------------------*/
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure NCS in Output Push-Pull mode */
    GPIO_InitStructure.Pin     = GPIO_PIN_2;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Set or Reset the control line */
    LCD_CS_LOW();
    LCD_CS_HIGH();

    SPI5_Init();
  }
}

/* USER CODE BEGIN 4 */
static void FMC_SDRAM_Init_Sequence(void)
{
	FMC_SDRAM_CommandTypeDef command;

	/* Step 1 and Step 2 already done in HAL_SDRAM_Init() */
	/* Step 3: Configure a clock configuration enable command */
	command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE; /* Set MODE bits to "001" */
	command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK2; /* configure the Target Bank bits */
	command.AutoRefreshNumber      = 1;
	command.ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);
	HAL_Delay(100); /* Step 4: Insert 100 us minimum delay - Min HAL Delay is 1ms */
	/* Step 5: Configure a PALL (precharge all) command */
	command.CommandMode            = FMC_SDRAM_CMD_PALL; /* Set MODE bits to "010" */
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);
	/* Step 6: Configure an Auto Refresh command */
	command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE; /* Set MODE bits to "011" */
	command.AutoRefreshNumber      = 4;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);
	/* Step 7: Program the external memory mode register */
	command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;/*set the MODE bits to "100" */
	command.AutoRefreshNumber      = 1;
	command.ModeRegisterDefinition =  (uint32_t)0 | 0<<3 | 3<<4 | 0<<7 | 1<<9;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);
	/* Step 8: Set the refresh rate counter - refer to section SDRAM refresh timer register in RM0455 */
	/* Set the device refresh rate
	* COUNT = [(SDRAM self refresh time / number of row) x  SDRAM CLK] – 20
			= [(64ms/4096) * 90MHz] - 20 = 1406.25 - 20 ~ 1386 = 0x56A */
	HAL_SDRAM_ProgramRefreshRate(&hsdram1, 0x56A);
}

static void SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance            = SPI5;
  hspi5.Init.Mode           = SPI_MODE_MASTER;
  hspi5.Init.Direction      = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize       = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity    = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase       = SPI_PHASE_1EDGE;
  hspi5.Init.NSS            = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode             = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial      = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/* FMC initialization function */
static void FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  FMC_SDRAM_Init_Sequence();
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */
  ili9341_Init();
  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}


/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
	y_fill_act ++;

	if(y_fill_act > y2_fill) {
		  lv_disp_flush_ready(&disp_drv);
	} else {
	  buf_to_flush += x2_flush - x1_flush + 1;
	  /*##-7- Start the DMA transfer using the interrupt mode ####################*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1);	/*Halt on error*/
	  }
	}
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{

}

static void DMA_Init(void)
{
  /*## -1- Enable DMA2 clock #################################################*/
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-2- Select the DMA functional Parameters ###############################*/
  hdma.Init.Channel                   =   DMA_CHANNEL_0;                /* DMA_CHANNEL_0                    */
  hdma.Init.Direction                 =   DMA_MEMORY_TO_MEMORY;         /* M2M transfer mode                */
  hdma.Init.PeriphInc                 =   DMA_PINC_ENABLE;              /* Peripheral increment mode Enable */
  hdma.Init.MemInc                    =   DMA_MINC_ENABLE;              /* Memory increment mode Enable     */
  hdma.Init.PeriphDataAlignment       =   DMA_PDATAALIGN_HALFWORD;      /* Peripheral data alignment : 16bit */
  hdma.Init.MemDataAlignment          =   DMA_MDATAALIGN_HALFWORD;      /* memory data alignment : 16bit     */
  hdma.Init.Mode                      =   DMA_NORMAL;                   /* Normal DMA mode                  */
  hdma.Init.Priority                  =   DMA_PRIORITY_HIGH;            /* priority level : high            */
  hdma.Init.FIFOMode                  =   DMA_FIFOMODE_ENABLE;          /* FIFO mode enabled                */
  hdma.Init.FIFOThreshold             =   DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
  hdma.Init.MemBurst                  =   DMA_MBURST_SINGLE;             /* Memory burst                     */
  hdma.Init.PeriphBurst               =   DMA_PBURST_SINGLE;             /* Peripheral burst                 */

  /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream0 #*/
  hdma.Instance = DMA2_Stream0;

  /*##-4- Initialize the DMA stream ##########################################*/
  if(HAL_DMA_Init(&hdma) != HAL_OK)
  {
    while(1);
  }

  HAL_DMA_RegisterCallback(&hdma, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
  HAL_DMA_RegisterCallback(&hdma, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);

  /*##-5- Configure NVIC for DMA transfer complete/error interrupts ##########*/
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}


void LCD_IO_WriteData(uint16_t RegValue)
{
  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Reset LCD control line(/CS) and Send data */
  LCD_CS_LOW();
  SPIx_Write(RegValue);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes register address.
  */
void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Reset WRX to send command */
  LCD_WRX_LOW();

  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  SPIx_Write(Reg);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Reads register value.
  * @param  RegValue Address of the register to read
  * @param  ReadSize Number of bytes to read
  * @retval Content of the register value
  */
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  uint32_t readvalue = 0;

  /* Select: Chip Select low */
  LCD_CS_LOW();

  /* Reset WRX to send command */
  LCD_WRX_LOW();

  SPIx_Write(RegValue);

  readvalue = SPIx_Read(ReadSize);

  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();

  return readvalue;
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


void IOE_Init(void)
{
  if(HAL_I2C_GetState(&hi2c3) == HAL_I2C_STATE_RESET)
  {
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void IOE_ITConfig(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin   = GPIO_PIN_15;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable and set GPIO EXTI Interrupt to the highest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
}

/**
  * @brief  IOE Delay.
  * @param  Delay in ms
  */
void IOE_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  IOE Writes single data operation.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @param  Value: Data to be written
  */
void IOE_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  I2Cx_WriteData(Addr, Reg, Value);
}

/**
  * @brief  IOE Reads single data.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @retval The read data
  */
uint8_t IOE_Read(uint8_t Addr, uint8_t Reg)
{
  return I2Cx_ReadData(Addr, Reg);
}

/**
  * @brief  IOE Reads multiple data.
  * @param  Addr: I2C Address
  * @param  Reg: Reg Address
  * @param  pBuffer: pointer to data buffer
  * @param  Length: length of the data
  * @retval 0 if no problems to read multiple data
  */
uint16_t IOE_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *pBuffer, uint16_t Length)
{
  return I2Cx_ReadBuffer(Addr, Reg, pBuffer, Length);
}


static volatile uint32_t t_saved = 0;
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p)
{
	t_saved = t;
}

void tft_init(void)
{
  static lv_color_t disp_buf1[TFT_HOR_RES * 60];
	static lv_color_t disp_buf2[TFT_HOR_RES * 60];
	static lv_disp_draw_buf_t buf;
	lv_disp_draw_buf_init(&buf, disp_buf1, disp_buf2, TFT_HOR_RES * 40);

  lv_disp_drv_init(&disp_drv);

  FMC_Init();
  LTDC_Init();
  DMA_Init();


  disp_drv.draw_buf = &buf;
	disp_drv.flush_cb = tft_flush;
	disp_drv.monitor_cb = monitor_cb;
	disp_drv.hor_res = TFT_HOR_RES;
	disp_drv.ver_res = TFT_VER_RES;
	lv_disp_drv_register(&disp_drv);
}


/**
 * Flush a color buffer
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
static void tft_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
  /*Return if the area is out the screen*/
	if(area->x2 < 0) return;
	if(area->y2 < 0) return;
	if(area->x1 > TFT_HOR_RES - 1) return;
	if(area->y1 > TFT_VER_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
	int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
	int32_t act_x2 = area->x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : area->x2;
	int32_t act_y2 = area->y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : area->y2;

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_fill = act_y2;
	y_fill_act = act_y1;
	buf_to_flush = color_p;


	  /*##-7- Start the DMA transfer using the interrupt mode #*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	HAL_StatusTypeDef err;
	err = HAL_DMA_Start_IT(&hdma,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
			  (x2_flush - x1_flush + 1));
	if(err != HAL_OK)
	{
		while(1);	/*Halt on error*/
	}
}


/**
 * Initialize your input devices here
 */
void touchpad_init(void)
{
  stmpe811_Init(TS_I2C_ADDRESS);
  stmpe811_TS_Start(TS_I2C_ADDRESS);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.read_cb = touchpad_read;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);
}

/**
 * Read an input device
 * @param indev_id id of the input device to read
 * @param x put the x coordinate here
 * @param y put the y coordinate here
 * @return true: the device is pressed, false: released
 */
static bool touchpad_read(lv_indev_drv_t * drv, lv_indev_data_t *data)
{
	static int16_t last_x = 0;
	static int16_t last_y = 0;

	bool detected;
	int16_t x;
	int16_t y;
	detected = touchpad_get_xy(&x, &y);
	if(detected) {
		data->point.x = x;
		data->point.y = y;
		last_x = data->point.x;
		last_y = data->point.y;

		data->state = LV_INDEV_STATE_PR;
	} else {
		data->point.x = last_x;
		data->point.y = last_y;
		data->state = LV_INDEV_STATE_REL;
	}

	return false;
}


static bool touchpad_get_xy(int16_t *x, int16_t *y)
{
	static int32_t _x = 0, _y = 0;
	int16_t xDiff, yDiff, xr, yr;
	uint16_t x_raw, y_raw;;

	bool detected;
	detected = stmpe811_TS_DetectTouch((uint16_t)TS_I2C_ADDRESS);

	if(!detected) return false;


	stmpe811_TS_GetXY((uint16_t)TS_I2C_ADDRESS, &x_raw, &y_raw);

	/* Y value first correction */
	y_raw -= 360;

	/* Y value second correction */
	yr = y_raw / 11;

	/* Return y_raw position value */
	if(yr <= 0) yr = 0;
	else if (yr > TFT_VER_RES) yr = TFT_VER_RES - 1;

	y_raw = yr;

	/* X value first correction */
	if(x_raw <= 3000) x_raw = 3870 - x_raw;
	else  x_raw = 3800 - x_raw;

	/* X value second correction */
	xr = x_raw / 15;

	/* Return X position value */
	if(xr <= 0) xr = 0;
	else if (xr > TFT_HOR_RES) xr = TFT_HOR_RES - 1;

	x_raw = xr;
	xDiff = x_raw > _x? (x_raw - _x): (_x - x_raw);
	yDiff = y_raw > _y? (y_raw - _y): (_y - y_raw);

	if (xDiff + yDiff > 5) {
		_x = x_raw;
		_y = y_raw;
	}

	/* Update the X and Y position */
	*x = _x;
	*y = _y;

	return true;
}


/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA_IRQHandler(&hdma);
}


