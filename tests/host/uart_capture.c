#include "main.h"

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

int _write(int, char *, int);

static int capture(const char *format, va_list args)
{
    char text[2048];
    int size=vsnprintf(text,sizeof(text),format,args);
    assert(size>=0 && (size_t)size<sizeof(text));
    return _write(1,text,size);
}

int __wrap_printf(const char *format, ...)
{
    va_list args;
    va_start(args,format);
    int size=capture(format,args);
    va_end(args);
    return size;
}

int __wrap___printf_chk(int flag, const char *format, ...)
{
    (void)flag;
    va_list args;
    va_start(args,format);
    int size=capture(format,args);
    va_end(args);
    return size;
}

int __wrap_puts(const char *text)
{
    int size=(int)strlen(text);
    assert(_write(1,(char *)text,size)==size);
    assert(_write(1,"\n",1)==1);
    return size+1;
}
