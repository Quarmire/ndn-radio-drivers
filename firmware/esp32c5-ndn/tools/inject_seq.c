// Sequence-numbered NDN injector for the two-node common-view test: broadcasts 0x8624 frames whose NDN
// payload carries an incrementing 4-byte counter, so two receivers can match "the same frame" and compare
// their per-frame RX timestamps (the offset between their clocks). Same monitor-iface AF_PACKET path as
// inject_ndn.c.  cc inject_seq.c -o inject_seq ; sudo ./inject_seq wlu1u1 [count] [gap_us]
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
    if (argc < 2) { fprintf(stderr, "usage: %s <iface> [count] [gap_us]\n", argv[0]); return 1; }
    const char *ifname = argv[1];
    long count = argc > 2 ? atol(argv[2]) : 0;
    long gap_us = argc > 3 ? atol(argv[3]) : 10000;

    int fd = socket(AF_PACKET, SOCK_RAW, htons(0x0003));
    if (fd < 0) { perror("socket"); return 1; }
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); return 1; }
    struct sockaddr_ll sll; memset(&sll, 0, sizeof sll);
    sll.sll_family = AF_PACKET; sll.sll_ifindex = ifr.ifr_ifindex; sll.sll_halen = 6;

    // radiotap(8) + 802.11 data + LLC/SNAP 0x8624 + NDN payload. The NDN payload begins at offset 40:
    // 0x05 0x08 then 8 bytes, of which the first 4 are the sequence counter (offset 42..46).
    unsigned char pkt[] = {
        0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
        0x08,0x00,0x00,0x00,
        0xff,0xff,0xff,0xff,0xff,0xff,
        0x02,'C','V','S', 0x01, 0x01,
        0xff,0xff,0xff,0xff,0xff,0xff,
        0x00,0x00,
        0xaa,0xaa,0x03,0x00,0x00,0x00,0x86,0x24,
        0x05,0x08, 0,0,0,0, 'c','v'                 // NDN: type,len, seq_le32 @42..46, "cv"
    };
    unsigned int seq = 0;
    while (count == 0 || seq < (unsigned long)count) {
        pkt[42] = seq & 0xff; pkt[43] = (seq >> 8) & 0xff; pkt[44] = (seq >> 16) & 0xff; pkt[45] = (seq >> 24) & 0xff;
        if (sendto(fd, pkt, sizeof pkt, 0, (struct sockaddr *)&sll, sizeof sll) < 0) {
            if (seq == 0) { perror("sendto"); return 1; }
        }
        seq++;
        if (gap_us) usleep(gap_us);
    }
    printf("injected %u seq-numbered frames on %s\n", seq, ifname);
    return 0;
}
