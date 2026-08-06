/*
 * Cross-check the C Tier-0 port against the Rust implementation.
 *
 *   rustc -O --edition 2021 gen_vectors.rs -o /tmp/gen_vectors && /tmp/gen_vectors > /tmp/vectors.h
 *   cc -DNDR_HOST_TEST -I../src -I/tmp -O2 ndr_tier0_selftest.c ../src/ndr_tier0.c -o /tmp/selftest
 *   /tmp/selftest
 *
 * The AR9271 firmware and the nRF54L15 testbed must agree bit-for-bit on the wire. A divergence
 * raises no error anywhere -- a frame simply stops matching -- so it has to be caught mechanically.
 */

#include <stdio.h>
#include <string.h>

#include "ndr_tier0.h"
#include "vectors.h"

/*
 * The C side has no insert_name(); the firmware only ever *receives*. So the test reconstructs the
 * sender's filter here, from the same primitives, and checks it against the Rust value. That also
 * exercises ndr_name_hash() and the shift-and-add multiply, which is the part most likely to be
 * subtly wrong.
 */
static void insert_name(ndr_filter_t *f, unsigned long long key, const char *name)
{
	size_t len = strlen(name);
	size_t i, start;
	int depth = 0;

	/* The root, always. */
	{
		ndr_filter_t m;
		ndr_mask_for(&m, key, (const a_uint8_t *)"/", 1);
		for (i = 0; i < 12; i++)
			f->b[i] |= m.b[i];
	}

	for (start = 1; start < len; start++) {
		if (name[start] == '/') {
			ndr_filter_t m;
			if (++depth >= NDR_MAX_DEPTH)
				return;
			ndr_mask_for(&m, key, (const a_uint8_t *)name, (a_uint32_t)start);
			for (i = 0; i < 12; i++)
				f->b[i] |= m.b[i];
		}
	}

	if (len > 0 && depth < NDR_MAX_DEPTH) {
		ndr_filter_t m;
		ndr_mask_for(&m, key, (const a_uint8_t *)name, (a_uint32_t)len);
		for (i = 0; i < 12; i++)
			f->b[i] |= m.b[i];
	}
}

static void to_wire(ndr_filter_t *f)
{
	f->b[0] = (a_uint8_t)((f->b[0] & ~NDR_RESERVED_MASK0) | NDR_RESERVED_MASK0);
}

static int cmp12(const unsigned char *a, const unsigned char *b)
{
	return memcmp(a, b, 12) == 0;
}

static void dump(const char *tag, const unsigned char *b)
{
	int i;
	printf("    %-8s", tag);
	for (i = 0; i < 12; i++)
		printf("%02x", b[i]);
	printf("\n");
}

int main(void)
{
	int i, fails = 0;

	for (i = 0; i < NDR_NVECTORS; i++) {
		const struct ndr_vec *v = &NDR_VECTORS[i];
		ndr_filter_t f, m;

		memset(&f, 0, sizeof(f));
		insert_name(&f, v->key, v->name);
		to_wire(&f);

		ndr_mask_for(&m, v->key, (const a_uint8_t *)v->name,
			     (a_uint32_t)strlen(v->name));

		if (!cmp12(f.b, v->filter)) {
			printf("FAIL filter  key=%llu name=%s\n", v->key, v->name);
			dump("rust", v->filter);
			dump("c", f.b);
			fails++;
		}
		if (!cmp12(m.b, v->mask)) {
			printf("FAIL mask    key=%llu name=%s\n", v->key, v->name);
			dump("rust", v->mask);
			dump("c", m.b);
			fails++;
		}

		/*
		 * Zero false negatives, stated the way that actually bites: a receiver registered on
		 * ANY prefix of the name -- including one deeper than the cap -- must match. The
		 * deep case is the one that was broken; ndr_clamp_prefix() is what makes it hold.
		 */
		{
			ndr_filter_t frame, pm;
			size_t len = strlen(v->name), j;

			memcpy(&frame, v->filter, 12);

			for (j = 1; j <= len; j++) {
				if (j != len && v->name[j] != '/')
					continue;
				ndr_mask_for(&pm, v->key, (const a_uint8_t *)v->name, (a_uint32_t)j);
				if (!ndr_may_match(&frame, &pm)) {
					printf("FAIL FALSE NEGATIVE key=%llu name=%s prefix=%.*s\n",
					       v->key, v->name, (int)j, v->name);
					fails++;
				}
			}
		}
	}

	printf("%s: %d vectors, %d failures\n", fails ? "FAIL" : "PASS", NDR_NVECTORS, fails);
	return fails ? 1 : 0;
}
