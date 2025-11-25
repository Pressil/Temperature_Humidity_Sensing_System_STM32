#include "main.h"
#include "drv_lcd.h"
#include "drv_aht21.h"
#include <stdlib.h>

// --- CONFIGURATION MACROS ---
/**
 * @brief  Filter Window Size
 * @why    A size of 10 provides a good balance between smoothing out noise and
 * keeping the display responsive.
 * - Size 5: Still too jittery.
 * - Size 50: Introduces noticeable lag when temperature changes.
 */
#define FILTER_SIZE 10

SRAM_HandleTypeDef hsram1;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);

/**
 * SECTION 1: CUSTOM DISPLAY FUNCTIONS
 * We implemented custom drawing and printing functions to avoid using the standard
 * <stdio.h> library (sprintf/printf), which is very heavy on Flash memory (~20KB).
 * Our custom implementation uses less than 1KB.
 */

// Draws a hollow rectangle (used for UI borders)
void LCD_DrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    LCD_DrawPixel(x1, y1, color);
    LCD_DrawPixel(x2, y2, color);
    // Draw Horizontal Lines
    for (uint16_t x = x1; x <= x2; x++) {
        LCD_DrawPixel(x, y1, color);
        LCD_DrawPixel(x, y2, color);
    }
    // Draw Vertical Lines
    for (uint16_t y = y1; y <= y2; y++) {
        LCD_DrawPixel(x1, y, color);
        LCD_DrawPixel(x2, y, color);
    }
}

// Recursive function to print integers digit by digit
void LCD_ShowNum(uint16_t x, uint16_t y, int num, uint16_t color, uint16_t back_color) {
    char buf[12]; int i = 0;
    if (num == 0) { LCD_ShowChar(x, y, '0', color, back_color); return; }
    // Handle negative numbers
    if (num < 0) { LCD_ShowChar(x, y, '-', color, back_color); x += 8; num = -num; }

    // Extract digits into buffer
    while (num > 0) { buf[i++] = (num % 10) + '0'; num /= 10; }

    // Print in reverse order (MSB first)
    while (--i >= 0) { LCD_ShowChar(x, y, buf[i], color, back_color); x += 8; }
}

/**
 * @brief  Manually renders a float value to the screen.
 * @why    Handling floats usually requires the FPU and heavy libraries.
 * Here, we simply split the float into two Integers:
 * 1. The Integer Part (e.g., 25)
 * 2. The Decimal Part (e.g., .43)
 * Then we print them separately with a dot in between.
 */
void LCD_ShowFloatManual(uint16_t x, uint16_t y, float val, uint16_t color, uint16_t back_color) {
    // 1. Extract Integer Part
    int intPart = (int)val;
    LCD_ShowNum(x, y, intPart, color, back_color);

    // Calculate cursor offset based on number length (for the decimal point)
    int offset = 8;
    if (intPart < 0) offset += 8;
    if (abs(intPart) > 9) offset += 8;
    if (abs(intPart) > 99) offset += 8;

    // 2. Print Decimal Point
    LCD_ShowChar(x + offset, y, '.', color, back_color);

    // 3. Extract Decimal Part (2 decimal places)
    // Multiply by 100 to convert .43 to 43
    int decPart = (int)((val - (float)intPart) * 100.0f);
    if (decPart < 0) decPart = -decPart;

    // Handle leading zeros (e.g., .05 should not print as .5)
    if (decPart < 10) {
        LCD_ShowChar(x + offset + 8, y, '0', color, back_color);
        LCD_ShowNum(x + offset + 16, y, decPart, color, back_color);
    } else {
        LCD_ShowNum(x + offset + 8, y, decPart, color, back_color);
    }
}

/*
 * SECTION 2: SIGNAL PROCESSING (Digital Filter)
 * Raw sensor data is often noisy due to electrical interference or airflow.
 * This Moving Average Filter smooths the data for a stable display.
 */
float Filter_Value(float new_val, float *buffer, uint8_t *index, uint8_t *count) {
    // STEP 1: SPIKE REJECTION (Sanity Check)
    // If the sensor reads >100C or <-40C, it is physically impossible.
    // This is likely an I2C bit error or electrical glitch. Ignore it.
    if (new_val > 100.0f || new_val < -40.0f) {
        if (*count > 0) {
            // Return the previous valid average instead
            float sum = 0;
            for(int i=0; i<*count; i++) sum += buffer[i];
            return sum / *count;
        }
        return new_val; // Fallback if it's the very first reading
    }

    // STEP 2: UPDATE BUFFER
    buffer[*index] = new_val;
    *index = (*index + 1) % FILTER_SIZE; // Circular buffer wrap-around
    if (*count < FILTER_SIZE) (*count)++;

    // STEP 3: CALCULATE AVERAGE
    float sum = 0;
    for (uint8_t i = 0; i < *count; i++) {
        sum += buffer[i];
    }
    return sum / (float)(*count);
}

// --- MAIN APPLICATION LOOP ---
int main(void)
{
  // 1. Initialize Core Hardware
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init(); // Configures Pins
  MX_FSMC_Init(); // Configures External Memory Bus for LCD

  // 2. Initialize LCD
  LCD_Init();
  LCD_Clear(WHITE);

  // 3. Draw User Interface (Static Elements)
  LCD_DrawRect(5, 5, 235, 40, BLUE);
  LCD_ShowString(40, 15, "RT-Spark AHT21", BLUE, WHITE);
  LCD_ShowString(20, 80, "Temp:", BLACK, WHITE);
  LCD_ShowString(20, 140, "Hum:", BLACK, WHITE);
  LCD_ShowString(20, 200, "Sensor Active ", GREEN, WHITE);

  // 4. Initialize Sensor (Sends Calibration Command 0xBE)
  AHT21_Init();

  // Variables for Data Processing
  float raw_temp = 0.0f;
  float raw_hum = 0.0f;
  float final_temp = 0.0f;
  float final_hum = 0.0f;

  // Circular Buffers for Filtering
  float temp_history[FILTER_SIZE] = {0};
  float hum_history[FILTER_SIZE] = {0};
  uint8_t temp_idx = 0, hum_idx = 0;
  uint8_t temp_cnt = 0, hum_cnt = 0;

  uint8_t count = 0;

  while (1)
  {
    // 5. Acquire Data
    uint8_t status = AHT21_Read(&raw_temp, &raw_hum);

    if (status == 1)
    {
        // 6. Process Data (Apply Filter)
        final_temp = Filter_Value(raw_temp, temp_history, &temp_idx, &temp_cnt);
        final_hum  = Filter_Value(raw_hum, hum_history, &hum_idx, &hum_cnt);

        // 7. Refresh Display
        // Note: We overwrite the old value area with spaces to "clear" it
        // before printing the new value to prevent flickering.
        LCD_ShowString(90, 100, "      ", WHITE, WHITE);
        LCD_ShowString(90, 160, "      ", WHITE, WHITE);

        // Print Temperature (Red for Heat)
        LCD_ShowFloatManual(90, 100, final_temp, RED, WHITE);
        LCD_ShowString(160, 100, "C", RED, WHITE);

        // Print Humidity (Blue for Water)
        LCD_ShowFloatManual(90, 160, final_hum, BLUE, WHITE);
        LCD_ShowString(160, 160, "%", BLUE, WHITE);

        // 8. System Heartbeat
        // Blinks a single pixel to show the main loop is running and not stuck
        if (count++ % 2) LCD_DrawPixel(230, 230, RED);
        else             LCD_DrawPixel(230, 230, WHITE);
    }
    else
    {
        // Error Handling: If I2C fails, show error and try to reset sensor
        LCD_ShowString(120, 200, "Read Err", RED, WHITE);
        HAL_Delay(100);
        AHT21_Init();
    }

    // Update Rate: 1Hz (Every 1000ms)
    // This is fast enough for environmental data but slow enough to be readable.
    HAL_Delay(1000);
  }
}

/*
 * SECTION 3: PERIPHERAL CONFIGURATION (Auto-Generated Logic)
 */

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_FSMC_Init(void) {
  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK3;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;

  /**
   * CRITICAL CONFIGURATION: 8-BIT BUS WIDTH
   * The RT-Spark schematic shows only D0-D7 connected to the LCD.
   * If we select 16-bit here, the MCU will try to send data on D8-D15,
   * which are not connected, resulting in corrupted colors.
   */
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;

  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;

  // Timing Parameters (Tuned for ST7789)
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 60;
  Timing.BusTurnAroundDuration = 5;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;

  HAL_SRAM_Init(&hsram1, &Timing, NULL);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  // Initial Pin States
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // Backlight Off initially
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);   // Reset High (Inactive)

  // Configure LCD Pins (Backlight & Reset)
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // Configure Sensor Pins (Bit-Banging I2C)
  // Must be Open-Drain (OD) to allow the line to be pulled Low by both MCU and Sensor
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
