#define ARM_MATH_CM4

#include "main.h"
#include "core_cm4.h"
#include "stm32f4xx_conf.h"
#include "mp3dec.h"
#include "Audio.h"
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx.h"         // Core header for STM32F4
#include "stm32f4xx_usart.h"   // USART functions and definitions
#include "stm32f4xx_gpio.h"    // GPIO configuration
#include "stm32f4xx_rcc.h"     // Clock configuration
#include "misc.h"              // NVIC configuration
#include "stm32_ub_uart.h"     // UART lib
#include "arm_math.h"
#include "arm_stereo.h"

// Macros
#define f_tell(fp)		((fp)->fptr)
#define BUTTON			(GPIOA->IDR & GPIO_Pin_0)

// Variables
volatile uint32_t		time_var1, time_var2;
USB_OTG_CORE_HANDLE		USB_OTG_Core;
USBH_HOST				USB_Host;
RCC_ClocksTypeDef		RCC_Clocks;
volatile int			enum_done = 0;

// MP3 Variables
#define FILE_READ_BUFFER_SIZE 8192
MP3FrameInfo			mp3FrameInfo;
HMP3Decoder				hMP3Decoder;
FIL						file;
char					file_read_buffer[FILE_READ_BUFFER_SIZE];
volatile int			bytes_left;
char					*read_ptr;

// Variables for UART command reception
UART_RXSTATUS_t  check;
UART_RX_t UART_RX[UART_ANZ];
volatile uint16_t uartCmdIndex = 0;
volatile uint8_t uartCmdComplete = 0;

// Define packet sizes and header values
#define VOL_PACKET_SIZE    5
#define COEF_PACKET_SIZE   25
#define PACKET_HEADER1     0xAA
#define PACKET_HEADER2     0x55

// Command IDs
#define CMD_VOLUME 0x01
#define CMD_COEF   0x02

// Global buffer for incoming packet
#define MAX_PACKET_SIZE  25  // Maximum packet size we expect
volatile uint8_t uartPacketBuffer[MAX_PACKET_SIZE * 3];
volatile uint8_t packetIndex = 0;
volatile uint8_t packetComplete = 0;

// Function prototypes
void processUARTPacket(void);
void updateFilterCoefficients(uint8_t filterID, float coef[5]);

// Biquad filter
#define BLOCK_SIZE_FLOAT 2304

arm_biquad_casd_df1_inst_f32 filter32Hz_settings;
arm_biquad_casd_df1_inst_f32 filter64Hz_settings;
arm_biquad_casd_df1_inst_f32 filter125Hz_settings;
arm_biquad_casd_df1_inst_f32 filter250Hz_settings;
arm_biquad_casd_df1_inst_f32 filter500Hz_settings;
arm_biquad_casd_df1_inst_f32 filter1000Hz_settings;
arm_biquad_casd_df1_inst_f32 filter2000Hz_settings;
arm_biquad_casd_df1_inst_f32 filter4000Hz_settings;
arm_biquad_casd_df1_inst_f32 filter8000Hz_settings;
arm_biquad_casd_df1_inst_f32 filter16000Hz_settings;

float filter32Hz_state [4];
float filter64Hz_state [4];
float filter125Hz_state [4];
float filter250Hz_state [4];
float filter500Hz_state [4];
float filter1000Hz_state [4];
float filter2000Hz_state [4];
float filter4000Hz_state [4];
float filter8000Hz_state [4];
float filter16000Hz_state [4];

float filter32Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter64Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter125Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter250Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter500Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter1000Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter2000Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter4000Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter8000Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};
float filter16000Hz_coefs [5] = {
		1.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
};

float buf_in [BLOCK_SIZE_FLOAT]; // 2304

// Private function prototypes
static void AudioCallback(void *context,int buffer);
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist,
		uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize);
static void play_mp3(char* filename);
static FRESULT play_directory (const char* path, unsigned char seek);

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
        // Read received data (only lower 8 bits are valid)
        uint16_t data = USART_ReceiveData(USART2);
        uint8_t receivedByte = data & 0xFF;

        // Accumulate byte in the packet buffer if there's room
        if (packetIndex < MAX_PACKET_SIZE) {
            uartPacketBuffer[packetIndex++] = receivedByte;

            // If we detect a complete packet based on the expected size,
            // you might choose to check the command ID to decide the expected length.
            // For simplicity, let's say we check header first:
            if (packetIndex >= 3) {
                // Check header to determine packet type
                if (uartPacketBuffer[0] != PACKET_HEADER1 || uartPacketBuffer[1] != PACKET_HEADER2) {
                    // Invalid header, reset buffer
                    packetIndex = 0;
                    memset((void*)uartPacketBuffer, 0, MAX_PACKET_SIZE);
                    return;
                }
                // Now, use command ID (byte 2) to decide expected packet size
                if (uartPacketBuffer[2] == CMD_VOLUME && packetIndex >= VOL_PACKET_SIZE) {
                    packetComplete = 1;
                } else if (uartPacketBuffer[2] == CMD_COEF && packetIndex >= COEF_PACKET_SIZE) {
                    packetComplete = 1;
                }
            }
        } else {
            // Buffer overflow: reset
            packetIndex = 0;
            memset((void*)uartPacketBuffer, 0, MAX_PACKET_SIZE);
        }
    }
}

// Function to compute checksum: simple sum modulo 256
uint8_t calculateChecksum(const uint8_t *data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// Process a complete packet
void processUARTPacket(void) {
    if (!packetComplete)
        return;

    // Determine packet type based on command ID (byte 2)
    uint8_t cmdID = uartPacketBuffer[2];
    if (cmdID == CMD_VOLUME && packetIndex == VOL_PACKET_SIZE) {
        // Verify checksum for volume packet
        uint8_t checksum = calculateChecksum(uartPacketBuffer, VOL_PACKET_SIZE - 1);
        if (checksum == uartPacketBuffer[VOL_PACKET_SIZE - 1]) {
            uint8_t volume = uartPacketBuffer[3];
            int volume16_t = ((int)volume) + 155;
            SetAudioVolume(volume16_t);
        }
    }
    else if (cmdID == CMD_COEF && packetIndex == COEF_PACKET_SIZE) {
        // Verify checksum for coefficient packet
        uint8_t checksum = calculateChecksum(uartPacketBuffer, COEF_PACKET_SIZE - 1);
        if (checksum == uartPacketBuffer[COEF_PACKET_SIZE - 1]) {
            uint8_t filterID = uartPacketBuffer[3];
            float coef[5];
            for (int i = 0; i < 5; i++) {
                union {
                    float f;
                    uint8_t bytes[4];
                } converter;
                memcpy(converter.bytes, &uartPacketBuffer[4 + i * 4], 4);
                coef[i] = converter.f;
            }
            // Call a function to update the corresponding filter with these coefficients
            updateFilterCoefficients(filterID, coef);
        }
    }

    // Reset buffer after processing
    packetIndex = 0;
    packetComplete = 0;
    memset((void*)uartPacketBuffer, 0, MAX_PACKET_SIZE);
}

// Example function: update filter coefficients for a given filter ID
void updateFilterCoefficients(uint8_t filterID, float coef[5]) {
    // Array of pointers to the coefficient arrays for each frequency band.
    float* filterCoeffsArray[10] = {
        filter32Hz_coefs,
        filter64Hz_coefs,
        filter125Hz_coefs,
		filter250Hz_coefs,
        filter500Hz_coefs,
        filter1000Hz_coefs,
        filter2000Hz_coefs,
		filter4000Hz_coefs,
		filter8000Hz_coefs,
		filter16000Hz_coefs
    };

    // Ensure filterID is within range.
    if (filterID < 10) {
        float* target = filterCoeffsArray[filterID];
        for (int i = 0; i < 5; i++) {
            // Invert the sign for feedback coefficients (i > 2), keep the others as is.
            target[i] = (i > 2) ? -coef[i] : coef[i];
        }
    }
}

void SystemClock_Config(void)
{
    // Step 1: Reset RCC settings to default
    RCC_DeInit();

    // Step 2: Enable High-Speed External (HSE) oscillator
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait for HSE to stabilize
    if (RCC_WaitForHSEStartUp() == SUCCESS)
    {
        // Step 3: Configure the PLL
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);

        // Step 4: Enable the PLL
        RCC_PLLCmd(ENABLE);

        // Wait for PLL to lock
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        // Step 5: Set system clock source to PLL
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Step 6: Configure the AHB, APB1, and APB2 prescalers
        RCC_HCLKConfig(RCC_SYSCLK_Div1);      // AHB = 168 MHz
        RCC_PCLK1Config(RCC_HCLK_Div4);        // APB1 = 42 MHz
        RCC_PCLK2Config(RCC_HCLK_Div2);        // APB2 = 84 MHz

        // Step 7: Enable the Flash prefetch buffer and set latency
        FLASH_SetLatency(FLASH_Latency_5);
        FLASH_PrefetchBufferCmd(ENABLE);
    }
}

/*
 * Main function. Called when startup code is done with
 * copying memory and setting up clocks.
 */
int main(void) {
	SystemClock_Config();

	// Intitialize the filters settings
	arm_biquad_cascade_df1_init_f32(&filter32Hz_settings, 1, &filter32Hz_coefs[0], &filter32Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter64Hz_settings, 1, &filter64Hz_coefs[0], &filter64Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter125Hz_settings, 1, &filter125Hz_coefs[0], &filter125Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter250Hz_settings, 1, &filter250Hz_coefs[0], &filter250Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter500Hz_settings, 1, &filter500Hz_coefs[0], &filter500Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter1000Hz_settings, 1, &filter1000Hz_coefs[0], &filter1000Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter2000Hz_settings, 1, &filter2000Hz_coefs[0], &filter2000Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter4000Hz_settings, 1, &filter4000Hz_coefs[0], &filter4000Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter8000Hz_settings, 1, &filter8000Hz_coefs[0], &filter8000Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter16000Hz_settings, 1, &filter16000Hz_coefs[0], &filter16000Hz_state[0]);

	GPIO_InitTypeDef  GPIO_InitStructure;
	// SysTick interrupt each 1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);


	// GPIOD Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Initialize USB Host Library
	USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &USBH_MSC_cb, &USR_Callbacks);

	// Configure USART2 using standard peripheral library functions
	UB_Uart_Init();

	for(;;) {
		USBH_Process(&USB_OTG_Core, &USB_Host);

		if (enum_done >= 2) {
			enum_done = 0;
			play_directory("", 0);
		}
	}
}

const char *get_filename_ext(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

static FRESULT play_directory (const char* path, unsigned char seek) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	char *fn; /* This function is assuming non-Unicode cfg. */
	char buffer[200];
#if _USE_LFN
	static char lfn[_MAX_LFN + 1];
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif


	res = f_opendir(&dir, path); /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break; /* Break on error or end of dir */
			if (fno.fname[0] == '.') continue; /* Ignore dot entry */
#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif
			if (fno.fattrib & AM_DIR) { /* It is a directory */

			} else { /* It is a file. */
				sprintf(buffer, "%s/%s", path, fn);

				// Check if it is an mp3 file
				if (strcmp("mp3", get_filename_ext(buffer)) == 0) {

					// Skip "seek" number of mp3 files...
					if (seek) {
						seek--;
						continue;
					}

					play_mp3(buffer);
				}
			}
		}
	}

	return res;
}

static void play_mp3(char* filename) {
	unsigned int br, btr;
	FRESULT res;

	bytes_left = FILE_READ_BUFFER_SIZE;
	read_ptr = file_read_buffer;

	if (FR_OK == f_open(&file, filename, FA_OPEN_EXISTING | FA_READ)) {

		// Read ID3v2 Tag
		char szArtist[120];
		char szTitle[120];
		Mp3ReadId3V2Tag(&file, szArtist, sizeof(szArtist), szTitle, sizeof(szTitle));

		// Fill buffer
		f_read(&file, file_read_buffer, FILE_READ_BUFFER_SIZE, &br);

		// Play mp3
		hMP3Decoder = MP3InitDecoder();
		Delay(200);
		InitializeAudio(Audio44100HzSettings);

		SetAudioVolume(0xAF);
		PlayAudioWithCallback(AudioCallback, 0);
		SetAudioVolume(205);

		for(;;) {
			/*
			 *
			 * If past half of buffer, refill...
			 *
			 * When bytes_left changes, the audio callback has just been executed. This
			 * means that there should be enough time to copy the end of the buffer
			 * to the beginning and update the pointer before the next audio callback.
			 * Getting audio callbacks while the next part of the file is read from the
			 * file system should not cause problems.
			 */

			// Process received UART commands asynchronously
			processUARTPacket();

			if (bytes_left < (FILE_READ_BUFFER_SIZE / 2)) {
				// Copy rest of data to beginning of read buffer
				memcpy(file_read_buffer, read_ptr, bytes_left);

				// Update read pointer for audio sampling
				read_ptr = file_read_buffer;

				// Read next part of file
				btr = FILE_READ_BUFFER_SIZE - bytes_left;
				res = f_read(&file, file_read_buffer + bytes_left, btr, &br);

				// Update the bytes left variable
				bytes_left = FILE_READ_BUFFER_SIZE;

				// Out of data or error or user button... Stop playback!
				if (br < btr || res != FR_OK || BUTTON) {
					StopAudio();

					// Re-initialize and set volume to avoid noise
					InitializeAudio(Audio44100HzSettings);
					SetAudioVolume(0);

					// Close currently open file
					f_close(&file);

					// Wait for user button release
					while(BUTTON){};

					// Return to previous function
					return;
				}
			}
		}
	}
}

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
static void AudioCallback(void *context, int buffer) {
	static int16_t audio_buffer0[2304];
	static int16_t audio_buffer1[2304];
// 4096
	int offset, err;
	int outOfData = 0;

	int16_t *samples;
	if (buffer) {
		samples = audio_buffer0;
		GPIO_SetBits(GPIOD, GPIO_Pin_13);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	} else {
		samples = audio_buffer1;
		GPIO_SetBits(GPIOD, GPIO_Pin_14);
		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;
	read_ptr += offset;

	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, (int*)&bytes_left, samples, 0);

	if (err) {
		/* error occurred */
		switch (err) {
		case ERR_MP3_INDATA_UNDERFLOW:
			outOfData = 1;
			break;
		case ERR_MP3_MAINDATA_UNDERFLOW:
			/* do nothing - next call to decode will provide more mainData */
			break;
		case ERR_MP3_FREE_BITRATE_SYNC:
		default:
			outOfData = 1;
			break;
		}
	} else {
		// no error
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

		// Duplicate data in case of mono to maintain playback speed
		if (mp3FrameInfo.nChans == 1) {
			for(int i = mp3FrameInfo.outputSamps;i >= 0;i--) 	{
				samples[2 * i]=samples[i];
				samples[2 * i + 1]=samples[i];
			}
			mp3FrameInfo.outputSamps *= 2;
		}
	}
	if (!outOfData) {
		// Converting all samples to float
		for (int i = 0; i < BLOCK_SIZE_FLOAT; i++) {
				buf_in[i] = (float)samples[i];
		}

		arm_biquad_cascade_df1_f32(&filter32Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter64Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter125Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter250Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter500Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter1000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter2000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter4000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter8000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		arm_biquad_cascade_df1_f32(&filter16000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);

		// Converting samples back to int
		for (int i = 0; i < BLOCK_SIZE_FLOAT; i++) {
			samples[i] = (int)(buf_in[i] * 0.05);
		}

		// Send buffer to codec for playing
		ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);
	}
}

/*
 * Called by the SysTick interrupt
 */
void TimingDelay_Decrement(void) {
	if (time_var1) {
		time_var1--;
	}
	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nTime) {
	time_var1 = nTime;
	while(time_var1){};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Text(FIL* pInFile, uint32_t unDataLen, char* pszBuffer, uint32_t unBufferSize)
{
	UINT unRead = 0;
	BYTE byEncoding = 0;
	if((f_read(pInFile, &byEncoding, 1, &unRead) == FR_OK) && (unRead == 1))
	{
		unDataLen--;
		if(unDataLen <= (unBufferSize - 1))
		{
			if((f_read(pInFile, pszBuffer, unDataLen, &unRead) == FR_OK) ||
					(unRead == unDataLen))
			{
				if(byEncoding == 0)
				{
					// ISO-8859-1 multibyte
					// just add a terminating zero
					pszBuffer[unDataLen] = 0;
				}
				else if(byEncoding == 1)
				{
					// UTF16LE unicode
					uint32_t r = 0;
					uint32_t w = 0;
					if((unDataLen > 2) && (pszBuffer[0] == 0xFF) && (pszBuffer[1] == 0xFE))
					{
						// ignore BOM, assume LE
						r = 2;
					}
					for(; r < unDataLen; r += 2, w += 1)
					{
						// should be acceptable for 7 bit ascii
						pszBuffer[w] = pszBuffer[r];
					}
					pszBuffer[w] = 0;
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			// we won't read a partial text
			if(f_lseek(pInFile, f_tell(pInFile) + unDataLen) != FR_OK)
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist, uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize)
{
	pszArtist[0] = 0;
	pszTitle[0] = 0;

	BYTE id3hd[10];
	UINT unRead = 0;
	if((f_read(pInFile, id3hd, 10, &unRead) != FR_OK) || (unRead != 10))
	{
		return 1;
	}
	else
	{
		uint32_t unSkip = 0;
		if((unRead == 10) &&
				(id3hd[0] == 'I') &&
				(id3hd[1] == 'D') &&
				(id3hd[2] == '3'))
		{
			unSkip += 10;
			unSkip = ((id3hd[6] & 0x7f) << 21) | ((id3hd[7] & 0x7f) << 14) | ((id3hd[8] & 0x7f) << 7) | (id3hd[9] & 0x7f);

			// try to get some information from the tag
			// skip the extended header, if present
			uint8_t unVersion = id3hd[3];
			if(id3hd[5] & 0x40)
			{
				BYTE exhd[4];
				f_read(pInFile, exhd, 4, &unRead);
				size_t unExHdrSkip = ((exhd[0] & 0x7f) << 21) | ((exhd[1] & 0x7f) << 14) | ((exhd[2] & 0x7f) << 7) | (exhd[3] & 0x7f);
				unExHdrSkip -= 4;
				if(f_lseek(pInFile, f_tell(pInFile) + unExHdrSkip) != FR_OK)
				{
					return 1;
				}
			}
			uint32_t nFramesToRead = 2;
			while(nFramesToRead > 0)
			{
				char frhd[10];
				if((f_read(pInFile, frhd, 10, &unRead) != FR_OK) || (unRead != 10))
				{
					return 1;
				}
				if((frhd[0] == 0) || (strncmp(frhd, "3DI", 3) == 0))
				{
					break;
				}
				char szFrameId[5] = {0, 0, 0, 0, 0};
				memcpy(szFrameId, frhd, 4);
				uint32_t unFrameSize = 0;
				uint32_t i = 0;
				for(; i < 4; i++)
				{
					if(unVersion == 3)
					{
						// ID3v2.3
						unFrameSize <<= 8;
						unFrameSize += frhd[i + 4];
					}
					if(unVersion == 4)
					{
						// ID3v2.4
						unFrameSize <<= 7;
						unFrameSize += frhd[i + 4] & 0x7F;
					}
				}

				if(strcmp(szFrameId, "TPE1") == 0)
				{
					// artist
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszArtist, unArtistSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else if(strcmp(szFrameId, "TIT2") == 0)
				{
					// title
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszTitle, unTitleSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else
				{
					if(f_lseek(pInFile, f_tell(pInFile) + unFrameSize) != FR_OK)
					{
						return 1;
					}
				}
			}
		}
		if(f_lseek(pInFile, unSkip) != FR_OK)
		{
			return 1;
		}
	}

	return 0;
}

