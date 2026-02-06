#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// LwIP sem sys_arch (NO_SYS = 1) para threadsafe_background
#define NO_SYS                             1
#define SYS_LIGHTWEIGHT_PROT               0

// APIs habilitadas
#define LWIP_RAW                           1
#define LWIP_NETCONN                       0
#define LWIP_SOCKET                        0

// Pilha TCP/IP basica
#define LWIP_TCP                           1
#define LWIP_UDP                           1
#define LWIP_ICMP                          1
#define LWIP_DHCP                          1
#define LWIP_DNS                           1
#define LWIP_ARP                           1
#define LWIP_IPV4                          1
#define LWIP_IPV6                          0

// Memoria
#define MEM_LIBC_MALLOC                    0
#define MEM_ALIGNMENT                      4
#define MEM_SIZE                           (16 * 1024)

// Pools e buffers
#define MEMP_NUM_TCP_PCB                   5
#define MEMP_NUM_TCP_PCB_LISTEN            5
#define MEMP_NUM_TCP_SEG                   64
#define MEMP_NUM_SYS_TIMEOUT               10

#define TCP_MSS                            1460
#define TCP_SND_BUF                        (4 * TCP_MSS)
#define TCP_WND                            (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                   (8 * TCP_SND_BUF / TCP_MSS)

#define PBUF_POOL_SIZE                     16
#define PBUF_POOL_BUFSIZE                  1024

// Callbacks de status
#define LWIP_NETIF_STATUS_CALLBACK         1
#define LWIP_NETIF_LINK_CALLBACK           1

// Outros
#define LWIP_STATS                         0
#define LWIP_TIMEVAL_PRIVATE               0
#define PPP_SUPPORT                        0

// Evita erro de sanity check e usa errno do newlib
#define LWIP_DISABLE_TCP_SANITY_CHECKS     1
#define LWIP_ERRNO_STDINCLUDE              1

#endif
