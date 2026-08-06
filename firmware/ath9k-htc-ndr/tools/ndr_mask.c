/*
 * Print the Tier-0 mask a receiver registers for a prefix, as C initialiser bytes.
 *
 *   cc -DNDR_HOST_TEST -I../src -O2 ndr_mask.c ../src/ndr_tier0.c -o /tmp/ndr_mask
 *   /tmp/ndr_mask /ndn/mds
 *
 * Used to bake a registered prefix into a firmware build for the A/B filter measurement, so the
 * two arms differ only in configuration bytes.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ndr_tier0.h"

int main(int argc, char **argv)
{
	const char *name = argc > 1 ? argv[1] : "/ndn/mds";
	unsigned long long key = argc > 2 ? strtoull(argv[2], NULL, 0) : 0;
	ndr_filter_t m;
	int i, bits = 0;

	ndr_mask_for(&m, key, (const a_uint8_t *)name, (a_uint32_t)strlen(name));

	for (i = 0; i < 12; i++) {
		int b;
		for (b = 0; b < 8; b++)
			if (m.b[i] & (1 << b))
				bits++;
	}

	fprintf(stderr, "prefix %s key %llu -> %d bits set\n", name, key, bits);
	for (i = 0; i < 12; i++)
		printf("%s0x%02x", i ? "," : "", m.b[i]);
	printf("\n");
	return 0;
}
