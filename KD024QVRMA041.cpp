#include "KD024QVRMA041.h"

KD024QVRMA041::KD024QVRMA041(SPI &spi, PinName pinCMD, PinName pinRST, PinName pinBacklight, 
                               uint32_t nBufferRows, uint32_t resolutionX, uint32_t resolutionY):
    LVGLDispILI9341(spi, pinCMD, pinRST, pinBacklight, nBufferRows, resolutionX, resolutionY){}

// int KD024QVRMA041::tft_init()
// {
// 	//hard_reset();
//     //write_table(init_cmd_list, sizeof(init_cmd_list));
//     //setRotation(2);

//     _backlight =1;
	
// 	return 0;
// }
                               