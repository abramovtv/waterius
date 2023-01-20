#ifndef _WATERIUS_INIT_h
#define _WATERIUS_INIT_h
#include "setup.h"
/*
Запишем 0 в конце буфера принудительно.
*/

inline void strncpy0(char *dest, const char *src, const size_t len)
{
    strncpy(dest, src, len - 1);
    dest[len - 1] = '\0';
}

extern bool setClock();

extern void print_wifi_mode();

extern void set_hostname();

extern String get_device_name();

extern String get_mac_address_hex();

extern  String get_current_time();

extern uint16_t get_checksum(const Settings &sett);

extern  bool is_http(const char *url);

extern bool is_https(const char *url);

extern bool is_valid_proto(const char *url);

#endif