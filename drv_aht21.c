#include "drv_aht21.h"

/*
 * SECTION 1: GPIO DIRECTION CONTROL (The Hardware Layer)
 */

// Switch SDA pin to INPUT mode (Receive)
// We do this when waiting for an ACK or reading data bits from the sensor.
void SDA_IN(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AHT_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;     // Internal resistor pulls line high when idle
    HAL_GPIO_Init(AHT_SDA_PORT, &GPIO_InitStruct);
}

// Switch SDA pin to OUTPUT mode (Transmit)
// We do this when sending commands (Start, Address, Data) to the sensor.
void SDA_OUT(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AHT_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-Drain is standard for I2C
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AHT_SDA_PORT, &GPIO_InitStruct);
}

/* Pin State Macros - Shortcuts for setting pins High/Low */
#define SCL_H HAL_GPIO_WritePin(AHT_SCL_PORT, AHT_SCL_PIN, GPIO_PIN_SET)
#define SCL_L HAL_GPIO_WritePin(AHT_SCL_PORT, AHT_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H HAL_GPIO_WritePin(AHT_SDA_PORT, AHT_SDA_PIN, GPIO_PIN_SET)
#define SDA_L HAL_GPIO_WritePin(AHT_SDA_PORT, AHT_SDA_PIN, GPIO_PIN_RESET)
#define READ_SDA HAL_GPIO_ReadPin(AHT_SDA_PORT, AHT_SDA_PIN)

// Simple delay to control I2C bus speed (Bit-Banging speed limit)
void I2C_Delay(void) {
    volatile int i = 400;
    while(i--);
}

/*
 * SECTION 2: I2C PROTOCOL PRIMITIVES (The Transport Layer)
 * These functions generate the specific electrical signals required by the
 * I2C standard.
 */

// Generate START condition: SDA goes High->Low while SCL is High
void I2C_Start(void) {
    SDA_OUT();    // Ensure we are in control
    SDA_H; SCL_H;
    I2C_Delay();
    SDA_L;        // Pull Data Low (Start Signal)
    I2C_Delay();
    SCL_L;        // Hold Clock Low (Bus Busy)
}

// Generate STOP condition: SDA goes Low->High while SCL is High
void I2C_Stop(void) {
    SDA_OUT();
    SCL_L; SDA_L; // Prepare Data Low
    I2C_Delay();
    SCL_H;        // Release Clock
    I2C_Delay();
    SDA_H;        // Release Data High (Stop Signal)
    I2C_Delay();
}

// Wait for the Sensor to acknowledge (ACK) our command
// Returns 0 if ACK received, 1 if Timeout/Error
uint8_t I2C_WaitAck(void) {
    uint32_t errTime = 0;
    SDA_IN();     // Switch to Input to listen
    SDA_H;        // Release line
    I2C_Delay();
    SCL_H;        // Clock High (Tell sensor to send ACK bit)
    I2C_Delay();

    // Wait for Sensor to pull SDA Low (ACK)
    while(READ_SDA) {
        errTime++;
        // Safety timeout to prevent infinite loop if sensor is missing
        if(errTime > 3000) {
            I2C_Stop();
            return 1;
        }
    }
    SCL_L; // Clock Low (Finish ACK cycle)
    return 0;
}

// Send 8 bits (1 Byte) to the sensor, MSB first
void I2C_SendByte(uint8_t byte) {
    SDA_OUT();
    SCL_L;
    for(int i=0; i<8; i++) {
        // Check if the current bit is 1 or 0
        if(byte & 0x80) SDA_H;
        else            SDA_L;

        byte <<= 1;   // Shift left to next bit
        I2C_Delay();
        SCL_H;        // Clock High (Sensor reads bit)
        I2C_Delay();
        SCL_L;        // Clock Low
        I2C_Delay();
    }
}

// Read 8 bits (1 Byte) from the sensor
uint8_t I2C_ReadByte(unsigned char ack) {
    unsigned char i, receive = 0;
    SDA_IN(); // Switch to Input to read
    for(i=0; i<8; i++) {
        SCL_L;
        I2C_Delay();
        SCL_H;        // Clock High (Data is valid)
        receive <<= 1; // Shift current data left
        if(READ_SDA) receive++; // Read pin, if High add 1
        I2C_Delay();
    }
    SDA_OUT(); // Switch back to Output to send response

    // Send ACK (Low) if we want more data, or NACK (High) to stop
    if (!ack) {
    	SCL_L; SDA_H; SCL_H;
    	I2C_Delay();
    	SCL_L;
    } else {
    	SCL_L; SDA_L; SCL_H;
    	I2C_Delay();
    	SCL_L;
    }
    return receive;
}

/*
 * SECTION 3: AHT21 SPECIFIC LOGIC (The Application Layer)
 */

// Initialize the sensor by sending the 0xBE command
void AHT21_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clock
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // Default pin configuration
    GPIO_InitStruct.Pin = AHT_SCL_PIN | AHT_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AHT_SCL_PORT, &GPIO_InitStruct);

    SCL_H; SDA_H;
    HAL_Delay(100); // Power-on delay required by datasheet

    // Send Calibration Command
    I2C_Start();
    I2C_SendByte(AHT_ADDR << 1); // Address + Write bit
    if(!I2C_WaitAck()) {
        I2C_SendByte(0xBE); // Init Command
        I2C_WaitAck();
        I2C_SendByte(0x08); // Parameter 1
        I2C_WaitAck();
        I2C_SendByte(0x00); // Parameter 2
        I2C_WaitAck();
        I2C_Stop();
    }
    HAL_Delay(10);
}

// Read Temperature and Humidity
uint8_t AHT21_Read(float *Temperature, float *Humidity) {
    uint8_t data[6];

    // 1. Trigger Measurement Command (0xAC)
    I2C_Start();
    I2C_SendByte(AHT_ADDR << 1);
    if(I2C_WaitAck()) return 0; // Check for device presence
    I2C_SendByte(0xAC); // Trigger Measure
    if(I2C_WaitAck()) return 0;
    I2C_SendByte(0x33); // Param 1
    if(I2C_WaitAck()) return 0;
    I2C_SendByte(0x00); // Param 2
    if(I2C_WaitAck()) return 0;
    I2C_Stop();

    // 2. Wait for Measurement (Sensor needs ~80ms to process)
    HAL_Delay(80);

    // 3. Read Data (6 Bytes)
    I2C_Start();
    I2C_SendByte((AHT_ADDR << 1) | 1); // Address + Read bit
    if(I2C_WaitAck()) return 0;

    for(int i=0; i<6; i++) {
        data[i] = I2C_ReadByte(i < 5); // ACK first 5 bytes, NACK last one
    }
    I2C_Stop();

    // 4. Parse Data (Bit shifting)
    // The sensor sends 20-bit values split across bytes. We assume the data is valid.
    // Humidity: Combined from Byte 1, Byte 2, and top 4 bits of Byte 3
    uint32_t rawHum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((data[3] & 0xF0) >> 4);
    // Temperature: Combined from lower 4 bits of Byte 3, Byte 4, and Byte 5
    uint32_t rawTemp = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

    // 5. Convert to Floating Point (Formulas from Datasheet)
    *Humidity = (float)rawHum * 100.0f / 1048576.0f;
    *Temperature = ((float)rawTemp * 200.0f / 1048576.0f) - 50.0f;

    return 1;
}
