//
// do_fonts.h
//
// Adapted from Multifont GFX library by Paul Kourany v1.0.1 2014, adapted from Adafruit_GFX library
// for use with Sharp Memory display. (see https://github.com/pkourany/Arduino_Adafruit_mfGFX_Library )
// 11.13.15 dmf
//
// Originally written to allow handling 6+ fonts at a time on Spark Core -> impossible to use with Arduino
// due to static memory limits. Workaround is by using PROGMEM and by defining selection of fonts for
// compiling, so only one or two fonts are available at a time. Note that there is an accompanying switch
// statement in the setFont() function of do_Adafruit_mfGFX.cpp, which means that any addition/change here
// must be reflected there, as well. Pain in the ass.
//
// Long term goal:  a) simplify, b) configure a large digits-only font (to limit memory allottment) for use
// in temperature display.  For now: a) incorporate large fonts that Tomek came up with
// (see https://forums.adafruit.com/viewtopic.php?f=47&t=61388#p317346 )
//

#ifndef _fonts_h
#define _fonts_h

#include "application.h"

//Font selection for compiling - comment out or uncomment definitions as required
#define GLCDFONT
//#define TIMESNEWROMAN8
//#define CENTURYGOTHIC8
#define ARIAL8
//#define COMICSANSMS8
//#define TESTFONT
// additional fonts -
#define CALIBRI18
//#define CALIBRI38     // character "x" only
#define CALIBRI48
#define CALIBRId48      // digit 0-9 only
//#define VERDANA88     // digits 0-9 only + ".","/"
//#define VERDANA120    // digits 0-9 only

// Font selection descriptors - Add an entry for each new font and number sequentially
#define TIMESNR_8	0
#define CENTURY_8	1   // note - uncoupling of define (above) and these descriptors - dumb: revise this
#define ARIAL_8		2
#define COMICS_8	3
#define GLCDFONT	4
#define TEST		5
// additional fonts -
#define CALIBRI_18       6
#define CALIBRI_X_38     7  // character "x" only
#define CALIBRI_48       8
#define CALIBRI_dig_48   9  // digits 0-9 only
#define VERDANA_dig_88  10  // digits 0-9 only + ".","/"
#define VERDANA_dig_120 11  // digits 0-9 only

#define FONT_START 0
#define FONT_END 1

struct FontDescriptor
{
	uint8_t	width;		// width in bits
	uint8_t	height; 	// char height in bits
	uint16_t offset;	// offset of char into char array
};

// Font references - add pair of references for each new font
#ifdef TIMESNEWROMAN8
extern const unsigned char timesNewRoman_8ptBitmaps[];
extern const FontDescriptor timesNewRoman_8ptDescriptors[];
#endif

#ifdef CENTURYGOTHIC8
extern const unsigned char centuryGothic_8ptBitmaps[];
extern const FontDescriptor centuryGothic_8ptDescriptors[];
#endif

#ifdef ARIAL8
extern const unsigned char arial_8ptBitmaps[];
extern const FontDescriptor arial_8ptDescriptors[];
#endif

#ifdef COMICSANSMS8
extern const unsigned char comicSansMS_8ptBitmaps[];
extern const FontDescriptor comicSansMS_8ptDescriptors[];
#endif

#ifdef GLCDFONT
extern const unsigned char glcdfontBitmaps[];
extern const FontDescriptor glcdfontDescriptors[];
#endif

#ifdef TESTFONT
extern const unsigned char testBitmaps[];
extern const FontDescriptor testDescriptors[];
#endif

// additional fonts -

#ifdef CALIBRI18
extern const unsigned char calibri_18ptBitmaps[];
extern const FontDescriptor calibri_18ptDescriptors[];
#endif

#ifdef CALIBRI38
// character "x" only
extern const unsigned char calibri_38ptBitmaps[];
extern const FontDescriptor calibri_38ptDescriptors[];
#endif

#ifdef CALIBRI48
extern const unsigned char calibri_48ptBitmaps[];
extern const FontDescriptor calibri_48ptDescriptors[];
#endif

#ifdef CALIBRId48
// digits 0-9 only
extern const unsigned char calibri_d_48ptBitmaps[];
extern const FontDescriptor calibri_d_48ptDescriptors[];
#endif

#ifdef VERDANA88
// digits 0-9 only + ".","/"
extern const unsigned char verdana_88ptBitmaps[];
extern const FontDescriptor verdana_88ptDescriptors[];
#endif

#ifdef VERDANA120
// digits 0-9 only
extern const unsigned char verdana_120ptBitmaps[];
extern const FontDescriptor verdana_120ptDescriptors[];
#endif

#endif
