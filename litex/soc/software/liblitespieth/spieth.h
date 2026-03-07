#ifndef __SPIETH_H
#define __SPIETH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef void (*udp_callback)(uint32_t src_ip, uint16_t src_port,
    uint16_t dst_port, void *data, uint32_t length);

void spieth_init(void);
void udp_start(const uint8_t *macaddr, uint32_t ip);
void udp_set_ip(uint32_t ip);
uint32_t udp_get_ip(void);
void udp_set_mac(const uint8_t *macaddr);
int  udp_arp_resolve(uint32_t ip);
void *udp_get_tx_buffer(void);
int  udp_send(uint16_t src_port, uint16_t dst_port, uint32_t length);
void udp_service(void);
void udp_set_callback(udp_callback callback);
int  send_ping(uint32_t ip, unsigned short payload_length);
void eth_init(void);
int  spieth_dhcp(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPIETH_H */
