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

// IRR filter
#define BLOCK_SIZE_FLOAT 2304

arm_biquad_casd_df1_inst_f32 filter60Hz_settings;
arm_biquad_casd_df1_inst_f32 filter170Hz_settings;
arm_biquad_casd_df1_inst_f32 filter350Hz_settings;
arm_biquad_casd_df1_inst_f32 filter1000Hz_settings;
arm_biquad_casd_df1_inst_f32 filter3500Hz_settings;
arm_biquad_casd_df1_inst_f32 filter10000Hz_settings;

float filter60Hz_state [4];
float filter170Hz_state [4];
float filter350Hz_state [4];
float filter1000Hz_state [4];
float filter3500Hz_state [4];
float filter10000Hz_state [4];

// REVERS 2 LAST SETTINGS!!!

// -10
float filter60Hz_coefs [5] = {
		0.9953066925387944f,
		-1.9785807925620864f,
		0.9833463973228215f,
		1.978502629144255f,
		-0.978731253279448f
};
// -5
float filter170Hz_coefs [5] = {
		0.9943049831526733f,
		-1.9545547672989163f,
		0.9608232458483684f,
		1.9543316105813113f,
		-0.9553513857186469f
};
// -5
float filter350Hz_coefs [5] = {
		0.9883156865826839f,
		-1.9069608295631226f,
		0.9210185959129932f,
		1.906037224789071f,
		-0.9102578872697289f
};
// +10
float filter1000Hz_coefs [5] = {
		1.080968292566556f,
		-1.7791861633846875f,
		0.7564543569181826f,
		1.7990964094846682f,
		-0.817512403384758f
};
// -5
float filter3500Hz_coefs [5] = {
		0.8930974185177668f,
		-1.1722479703379898f,
		0.4417000925925752f,
		1.1089934899907876f,
		-0.398051991457544f
};
// -5
float filter10000Hz_coefs [5] = {
		0.7502282171046981f,
		-0.12847012300253685f,
		0.13261136424009284f,
		-0.16508498549245873f,
		-0.17639468983978665f
};

float buf_in [BLOCK_SIZE_FLOAT]; // 2304

// Private function prototypes
static void AudioCallback(void *context,int buffer);
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist,
		uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize);
static void play_mp3(char* filename);
static FRESULT play_directory (const char* path, unsigned char seek);

// Process received UART command
void ProcessUARTCommand(void) {
	  check = UB_Uart_ReceiveString(COM2, UART_RX->rx_buffer);
	  if(check == RX_READY) {
		  if (strncmp((char*)UART_RX->rx_buffer, "VOL:", 4) == 0)
			{
				int newVolume = atoi((char*)UART_RX->rx_buffer + 4);  // Extract volume value

				// Ensure volume is within a valid range (adjust limits as needed)
				if (newVolume >= 0 && newVolume <= 255)
				{
					SetAudioVolume((uint16_t)newVolume);
				}
				memset(UART_RX[0].rx_buffer, 0, sizeof(UART_RX[0].rx_buffer));

			    UART_RX[0].rx_buffer[0]=RX_END_CHR;
			    UART_RX[0].wr_ptr=0;
			    UART_RX[0].rd_ptr=0;
			    UART_RX[0].status=RX_EMPTY;
			}
	  }
}

/*
 * Main function. Called when startup code is done with
 * copying memory and setting up clocks.
 */
int main(void) {
	// Intitialize the filters settings
	arm_biquad_cascade_df1_init_f32(&filter60Hz_settings, 1, &filter1000Hz_coefs[0], &filter60Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter170Hz_settings, 1, &filter170Hz_coefs[0], &filter170Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter350Hz_settings, 1, &filter350Hz_coefs[0], &filter350Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter1000Hz_settings, 1, &filter1000Hz_coefs[0], &filter1000Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter3500Hz_settings, 1, &filter3500Hz_coefs[0], &filter3500Hz_state[0]);
	arm_biquad_cascade_df1_init_f32(&filter10000Hz_settings, 1, &filter10000Hz_coefs[0], &filter10000Hz_state[0]);

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
		SetAudioVolume(200);

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
			ProcessUARTCommand();

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

		arm_biquad_cascade_df1_f32(&filter60Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);
		arm_biquad_cascade_df1_f32(&filter170Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);
		arm_biquad_cascade_df1_f32(&filter350Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);
		//arm_biquad_cascade_df1_f32(&filter1000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);
		arm_biquad_cascade_df1_f32(&filter3500Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);
		arm_biquad_cascade_df1_f32(&filter10000Hz_settings, &buf_in[0], &buf_in[0], BLOCK_SIZE_FLOAT);


		// Converting samples back to int
		for (int i = 0; i < BLOCK_SIZE_FLOAT; i++) {
			samples[i] = (int)buf_in[i];
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

