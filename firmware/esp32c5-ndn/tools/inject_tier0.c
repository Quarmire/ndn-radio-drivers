// Tier-0 test injector: like inject_ndn.c, but writes a caller-supplied 16-byte prefix-set filter into
// the 802.11 address octets (addr1 ‖ addr2 ‖ addr3[0..4]) so a receiver's on-device Tier-0 filter can
// accept/reject it. The filter is where the sender encodes its name's prefix set (named-filter-mac
// -redesign.md §3); here we hand it in directly (golden-vector wire bytes) to exercise a receiver.
//
//   cc inject_tier0.c -o inject_tier0 ; sudo ./inject_tier0 wlu1u1 <32-hex> [count] [gap_us]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if_packet.h>
#include <arpa/inet.h>

static int hex16(const char *s, unsigned char out[16]) {
    if (strlen(s) != 32) return -1;
    for (int i = 0; i < 16; i++) {
        unsigned v; if (sscanf(s + 2 * i, "%2x", &v) != 1) return -1; out[i] = (unsigned char)v;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <iface> <32-hex-filter> [count] [gap_us]\n", argv[0]); return 1; }
    const char *ifname = argv[1];
    unsigned char filt[16];
    if (hex16(argv[2], filt) < 0) { fprintf(stderr, "filter must be 32 hex chars\n"); return 1; }
    long count = argc > 3 ? atol(argv[3]) : 0;
    long gap_us = argc > 4 ? atol(argv[4]) : 5000;

    int fd = socket(AF_PACKET, SOCK_RAW, htons(0x0003));
    if (fd < 0) { perror("socket"); return 1; }
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }
    struct sockaddr_ll sll; memset(&sll, 0, sizeof sll);
    sll.sll_family = AF_PACKET; sll.sll_ifindex = ifr.ifr_ifindex; sll.sll_halen = 6;

    // radiotap(8) + 802.11 data + LLC/SNAP 0x8624 + NDN payload. addr1‖addr2‖addr3[0..4] = pkt[12..28].
    unsigned char pkt[] = {
        0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,                     // radiotap
        0x08,0x00,0x00,0x00,                                        // FC data, dur
        0,0,0,0,0,0,  0,0,0,0,0,0,  0,0,0,0,0,0,                    // A1(6) A2(6) A3(6) — filled below
        0x00,0x00,                                                  // seq
        0xaa,0xaa,0x03,0x00,0x00,0x00,0x86,0x24,                    // LLC/SNAP -> 0x8624
        0x05,0x08,'t','i','e','r','0'                              // NDN-ish payload
    };
    // Encode the prefix-set filter into addr1 ‖ addr2 ‖ addr3[0..4] (16 bytes at pkt[12..28]).
    memcpy(pkt + 12, filt, 16);

    long sent = 0;
    while (count == 0 || sent < count) {
        if (sendto(fd, pkt, sizeof pkt, 0, (struct sockaddr *)&sll, sizeof sll) < 0) {
            if (sent == 0) { perror("sendto"); return 1; }
        } else sent++;
        if (gap_us) usleep(gap_us);
    }
    printf("injected %ld frames on %s (filter %s)\n", sent, ifname, argv[2]);
    return 0;
}
