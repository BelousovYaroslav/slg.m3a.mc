/***********************************************************************/
/*                                                                     */
/*  SERIAL.C:  Low Level Serial Routines                               */
/*                                                                     */
/***********************************************************************/

#include <aduc7026.H>                      /* ADuC7024 definitions          */

#define CR     0x0D

#pragma INTERWORK

char gl_c_numbers[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

int putchar( int ch)  {                   /* Write character to Serial Port  */
  if( ch == '\n') {
    while( !( 0x020 == ( COMSTA0 & 0x020))) {}
    COMTX = CR;     /* output CR */
  }

  while( !( 0x020 == ( COMSTA0 & 0x020))) {}
  return( COMTX = ch);
}

int putchar_nocheck( int ch)  {                   /* Write character to Serial Port w/o adding \r to \n */
  while( !( 0x020 == ( COMSTA0 & 0x020))) {}
  return( COMTX = ch);
}

int getchar( void)  {                      /* Read character from Serial Port */
  while( !( 0x01 == ( COMSTA0 & 0x01)))
  {}
  return( COMRX);
}

int write( char * ptr, int len) {
  int i;
  for( i = 0; i < len; i++)
    putchar( *ptr++);

  return len;
}

void PrintHexIntNumber( int n) {
  putchar( gl_c_numbers[((n & 0xf0000000) >> 28)]);
  putchar( gl_c_numbers[((n & 0xf000000) >> 24)]);

  putchar( gl_c_numbers[((n & 0xf00000) >> 20)]);
  putchar( gl_c_numbers[((n & 0xf0000) >> 16)]);

  putchar( gl_c_numbers[((n & 0xf000) >> 12)]);
  putchar( gl_c_numbers[((n & 0xf00) >> 8)]);

  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
}

void PrintHexShortNumber( short n) {
  putchar( gl_c_numbers[((n & 0xf000) >> 12)]);
  putchar( gl_c_numbers[((n & 0xf00) >> 8)]);

  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
}

void PrintHexCharNumber( char n) {
  putchar( gl_c_numbers[((n & 0xf0) >> 4)]);
  putchar( gl_c_numbers[(n & 0xf)]);
}