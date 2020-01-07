/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: nmea.h 17 2008-03-11 11:56:11Z xtimor $
 *
 */

#ifndef __NMEA_H__
#define __NMEA_H__

#include "./config.h"
#include "./units.h"
#include "./gmath.h"
#include "./info.h"
#include "./sentence.h"
#include "./generate.h"
#include "./generator.h"
#include "./parse.h"
#include "./parser.h"
#include "./context.h"

void trace(const char *str, int str_size);
void error(const char *str, int str_size);
void gps_info(const char *str, int str_size);
void GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, uint8_t GMT,uint8_t AREA) ;
int Nmea_Decode_test(void);
nmeaINFO Nmea_Decode_para(char *buf, int size);

#endif /* __NMEA_H__ */
