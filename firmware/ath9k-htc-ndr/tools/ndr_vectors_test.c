/*
 * Cross-check this C Tier-0 copy against the shared golden vectors (F2 / P0.2).
 *
 * Three implementations of this filter exist — ndn-ext `tier0.rs` (the generator),
 * `lr2021-nrf54l15-rs/src/tier0.rs`, and this one. They agree today by having been edited in sync,
 * which is not a guarantee, and a divergence shows up on air as a SILENT false negative: two nodes
 * in one group simply stop matching, with nothing logged. This binds the C copy to the file.
 *
 * ndr_tier0.h's NDR_HOST_TEST branch already existed for exactly this ("cross-checks it against the
 * Rust implementation's vectors") — the hook was there, the vectors were not.
 *
 * Build + run:
 *   cc -DNDR_HOST_TEST -Isrc -o /tmp/ndr_vec tools/ndr_vectors_test.c src/ndr_tier0.c
 *   /tmp/ndr_vec ../../golden/tier0/vectors.txt
 *
 * WHAT THIS CAN AND CANNOT CHECK. The ath9k firmware is receive-side only: it has no
 * insert-all-prefixes-of-a-name routine, so it cannot regenerate a row's wire bytes. What it can do
 * — and what actually pins the shared parameters — is take each row's bytes as given and verify
 * that ITS OWN siphash, bit layout, popcount, keying and fill cap agree with them. A hash or
 * bit-layout drift fails the mask tests below; that is the divergence that matters here.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ndr_tier0.h"

static int failures;

static void check(int ok, const char *what)
{
	if (!ok) {
		printf("FAIL: %s\n", what);
		failures++;
	}
}

/* "/ndn/alarm" -> "/ndn": the first component, which every row's filter must contain. */
static a_uint32_t first_component_len(const char *name)
{
	const char *slash = strchr(name + 1, '/');

	return slash ? (a_uint32_t)(slash - name) : (a_uint32_t)strlen(name);
}

static int hex_to_filter(const char *hex, ndr_filter_t *out)
{
	int i;

	if (strlen(hex) != 24)
		return 0;
	for (i = 0; i < 12; i++) {
		unsigned byte;

		if (sscanf(hex + 2 * i, "%2x", &byte) != 1)
			return 0;
		out->b[i] = (a_uint8_t)byte;
	}
	return 1;
}

int main(int argc, char **argv)
{
	char line[1024];
	FILE *f;
	int rows = 0;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <vectors.txt>\n", argv[0]);
		return 2;
	}
	f = fopen(argv[1], "r");
	if (!f) {
		perror("open vectors");
		return 2;
	}

	while (fgets(line, sizeof(line), f)) {
		char label[64], key[64], name[512], hex[64];
		unsigned pop;

		if (!strncmp(line, "params ", 7)) {
			unsigned k, m, depth, cap;

			/* The parameter header is itself a vector: if the file and this header
			 * disagree about k, M, the depth cap or the fill cap, the two
			 * implementations are not speaking the same protocol. */
			check(sscanf(line, "params version=%*d k=%u m=%u max_depth=%u fill_cap=%u",
				     &k, &m, &depth, &cap) == 4,
			      "params header parses");
			check(k == NDR_K, "k matches NDR_K");
			check(m == NDR_M_BITS, "M matches NDR_M_BITS");
			check(depth == NDR_MAX_DEPTH, "max_depth matches NDR_MAX_DEPTH");
			check(cap == NDR_FILL_CAP, "fill_cap matches NDR_FILL_CAP");
			continue;
		}
		if (strncmp(line, "row ", 4))
			continue;
		if (sscanf(line, "row %63s %63s %511s %63s %u", label, key, name, hex, &pop) != 5)
			continue;
		rows++;

		ndr_filter_t filt, mask;
		char what[256];

		check(hex_to_filter(hex, &filt), "row wire bytes parse");

		/* Bit layout + popcount agree with the generator's count. */
		snprintf(what, sizeof(what), "%s: popcount %u == recorded %u", label,
			 ndr_popcount(&filt), pop);
		check(ndr_popcount(&filt) == pop, what);

		/* The real cross-check: build the mask for the row's first component with THIS
		 * implementation's siphash + position mapping, and require it to match (or, for the
		 * wrong-key row, NOT to match) the generator's bytes. A hash or layout drift fails
		 * here, which is the whole point of the file. */
		ndr_mask_for(&mask, (const a_uint8_t *)key, (const a_uint8_t *)name,
			     first_component_len(name));

		if (!strcmp(label, "wrongkey")) {
			/* Same name, different group key: must NOT match a mask built under the
			 * generator's key. Pins that the filter is keyed at all (doctrine §8). */
			ndr_filter_t right;

			ndr_mask_for(&right, (const a_uint8_t *)"ndr/tier0-vec-01",
				     (const a_uint8_t *)name, first_component_len(name));
			snprintf(what, sizeof(what), "%s: does NOT match the other key's mask",
				 label);
			check(!ndr_may_match(&filt, &right), what);
		} else {
			snprintf(what, sizeof(what), "%s: matches its own first-component mask",
				 label);
			check(ndr_may_match(&filt, &mask) != 0, what);
		}
	}
	fclose(f);
	check(rows >= 4, "all vector rows were read");

	/* F1: the fill cap, asserted here too — it decides admissibility, so a copy that skips it
	 * is a hole even if every other byte agrees. */
	{
		ndr_filter_t all_ones, mask;
		int i;

		for (i = 0; i < 12; i++)
			all_ones.b[i] = 0xff;
		ndr_mask_for(&mask, (const a_uint8_t *)"ndr/tier0-vec-01",
			     (const a_uint8_t *)"/ndn", 4);
		check(ndr_popcount(&all_ones) > NDR_FILL_CAP, "all-ones exceeds the cap");
		check(!ndr_may_match(&all_ones, &mask),
		      "an over-full filter is inert (the amplified universal wake)");
	}

	printf("%s: %d rows, %d failure(s)\n", failures ? "FAILED" : "ok", rows, failures);
	return failures ? 1 : 0;
}
