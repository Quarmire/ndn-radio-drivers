// Minimal raw-802.11 injector for a mac80211 monitor iface — a 0x8624 NDN source for bench tests
// (e.g. driving the ESP32-C5's promiscuous RX). Sends a broadcast 802.11 data frame carrying the
// canonical NDN LLC/SNAP ethertype 0x8624, prefixed with a minimal radiotap header, on a monitor iface.
//
//   cc inject_ndn.c -o inject_ndn ; sudo ./inject_ndn wlu1u1 [count] [gap_us]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if_packet.h>
#include <arpa/inet.h>

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <monitor-iface> [count] [gap_us]\n", argv[0]); return 1; }
    const char *ifname = argv[1];
    long count = argc > 2 ? atol(argv[2]) : 0;      // 0 = forever
    long gap_us = argc > 3 ? atol(argv[3]) : 5000;  // 5 ms => ~200/s

    int fd = socket(AF_PACKET, SOCK_RAW, htons(0x0003));
    if (fd < 0) { perror("socket"); return 1; }
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }
    struct sockaddr_ll sll; memset(&sll, 0, sizeof sll);
    sll.sll_family = AF_PACKET; sll.sll_ifindex = ifr.ifr_ifindex; sll.sll_halen = 6;

    // radiotap (8 B, no fields -> driver defaults) + 802.11 data + LLC/SNAP 0x8624 + NDN payload.
    unsigned char pkt[] = {
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,             // radiotap: ver,pad,len=8,present=0
        0x08, 0x00, 0x00, 0x00,                                     // FC data, dur
        0xff,0xff,0xff,0xff,0xff,0xff,                              // A1 = DA broadcast
        0x02,'M','T','J', 0x01, 0x01,                              // A2 = SA (an injector nonce)
        0xff,0xff,0xff,0xff,0xff,0xff,                              // A3 = BSSID
        0x00, 0x00,                                                // seq
        0xaa,0xaa,0x03,0x00,0x00,0x00,0x86,0x24,                    // LLC/SNAP -> ethertype 0x8624
        0x05,0x08,'m','t','-','n','d','n'                          // NDN-ish payload
    };
    long sent = 0;
    while (count == 0 || sent < count) {
        if (sendto(fd, pkt, sizeof pkt, 0, (struct sockaddr *)&sll, sizeof sll) < 0) {
            if (sent == 0) { perror("sendto"); return 1; }
        } else sent++;
        if (gap_us) usleep(gap_us);
    }
    printf("injected %ld 0x8624 frames on %s\n", sent, ifname);
    return 0;
}
