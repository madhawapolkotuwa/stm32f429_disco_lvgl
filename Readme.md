### Watch full video ⇒
[![Youtube Video](https://img.youtube.com/vi/ArRN7JFltDU/0.jpg)](https://www.youtube.com/watch?v=ArRN7JFltDU)

## Display Connection

The STM32F429I-DISCO board uses both SPI and LTDC for TFT Dispaly, (but not simultaneously for the same purpose.)

### ✔️ 1. SPI is used for Register Configuration (ILI9341 control)

- SPI5 (with pins PF7 = SCK, PF9 = MOSI) is used to send commands/data to configure the ILI9341 registers.
- This includes initialization commands such as:

  - Display ON
  - Memory access mode
  - Pixel format
  - Frame rate, etc.

- GPIOs like PD13 (WRX_DCX), PC2 (CSX), PF10 (RESET) are also used.

### ✔️ 2. LTDC is used for Framebuffer-Based Pixel Streaming

- The LTDC controller streams actual pixel data (framebuffer) to the display using RGB interface (parallel RGB signals + control lines).
- The pixel data is fetched from a memory location (FBStartAddress = 0xD0000000 (used onboard SDRAM)) and sent to the display without CPU intervention.
- This is hardware-accelerated and gives smooth graphics performance.

### 💡 So Why Use SPI If LTDC Is Doing the Pixel Job?

The ILI9341 has two interface roles:

1. Register Access (Control) – via SPI
2. Pixel Data Streaming – via RGB (parallel) through LTDC

The ILI9341 supports this dual role, where registers are configured through SPI, and pixel data is streamed through parallel RGB interface once it’s properly configured.

### 📌 Key Takeaway

- SPI (4-wire, IM[3:0] = 0110) is used only to configure the ILI9341 registers.
- LTDC uses the RGB parallel data interface (DB0–DB17 mapped to color signals: RGB) to continuously push pixel data.
- This is a hybrid configuration:
  - 🔧 SPI = command/control
  - 📺 LTDC = framebuffer image output

### In the Code:

- ili9341_Init() is called before LTDC init — this sets up the controller via SPI.
- LTDC_Init() sets up the pixel streaming path using a framebuffer.
- GPIOs for WRX, CSX, etc., are toggled manually during initialization.
- SPI5_Init() configures the SPI peripheral for ILI9341 communication.

## 📊 System-Level Diagram: STM32F429 ↔ ILI9341

```yml
+-------------------+          +---------------------------+
|                   |   SPI    |                           |
|   STM32F429 MCU   +---------->  ILI9341 TFT Controller   |
|                   |          |                           |
|   (SPI5: PF7, PF9)|          |  Control Interface (SPI)  |
|   PD13 = WRX/DCX  +---------->  CMD/DATA                 |
|   PC2  = CSX      |          |                           |
|   PF10 = RESET    +---------->  RESET                    |
|                   |          |                           |
|                   |          |                           |
|     (AHB Master)  |   RGB    |      Display Data (RGB)   |
|    LTDC Hardware  +=========> R[5:0], G[5:0], B[5:0]     |
|    via DB0–DB17   |          |  (RGB565 Format)          |
|                   |          |                           |
+-------------------+          +---------------------------+
                                      |
                                      v
                             +-------------------+
                             |  240x320 TFT LCD   |
                             +-------------------+
```

## 🔧 How It Works

### 1. Startup Phase (Using SPI)

- SPI commands are sent to:

  - Reset the ILI9341
  - Configure orientation, pixel format (e.g. RGB565), inversion, gamma
  - Enable memory write mode for LTDC streaming

- Example SPI command:
  - WriteReg(0x36, 0x48); → sets Memory Access Control (row/column order, RGB/BGR)

### 2. Display Phase (Using LTDC)

- LTDC reads from a framebuffer (e.g., 0xD0000000) in external RAM
- It sends pixel data to the display using parallel RGB signals plus:
  - HSYNC
  - VSYNC
  - DOTCLK
  - DE (Data Enable)

The ILI9341, now in RGB interface mode, behaves like a passive display that only needs timing + data signals. It no longer needs SPI during normal operation.

### 🖼️ Example Framebuffer Flow (LTDC)

```c
#define FRAMEBUFFER_ADDR  ((uint32_t)0xD0000000)

// Framebuffer format: RGB565 = 16 bits per pixel

// Fill screen with red
uint16_t* fb = (uint16_t*)FRAMEBUFFER_ADDR;
for (int y = 0; y < 320; ++y)
{
    for (int x = 0; x < 240; ++x)
    {
        fb[y * 240 + x] = 0xF800; // RGB565: Red
    }
}
```

## ✅ Summary of Signal Roles

| Signal      | STM32 Pin | Purpose             | Used In       |
| ----------- | --------- | ------------------- | ------------- |
| `SPI5_SCK`  | PF7       | Clock for SPI       | Init phase    |
| `SPI5_MOSI` | PF9       | Data out to ILI9341 | Init phase    |
| `WRX_DCX`   | PD13      | Data/Command select | Init phase    |
| `RESET`     | PF10      | Hardware reset      | Init phase    |
| `CSX`       | PC2       | Chip select         | Init phase    |
| `DB0–DB17`  | Multiple  | RGB data            | Display phase |
| `DOTCLK`    | PG7       | Pixel clock         | Display phase |
| `HSYNC`     | PC6       | Horizontal sync     | Display phase |
| `VSYNC`     | PC7       | Vertical sync       | Display phase |
| `ENABLE`    | PF10      | Data Enable         | Display phase |
