/*
 * Copyright (C) 2008-2012 Numascale AS, support@numascale.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>

#include "dnc-regs.h"
#include "dnc-route.h"
#include "dnc-access.h"
#include "dnc-config.h"
#include "dnc-commonlib.h"

#define OFFS_RTBLL (LC3_CSR_ROUT_BXTBLL00 - LC3_CSR_ROUT_LCTBL00)
#define OFFS_RTBLM (LC3_CSR_ROUT_BLTBL00  - LC3_CSR_ROUT_LCTBL00)
#define OFFS_RTBLH (LC3_CSR_ROUT_BXTBLH00 - LC3_CSR_ROUT_LCTBL00)
#define OFFS_LTBL  0
#define DEBUG_ROUTE(fmt, ...)
/* #define DEBUG_ROUTE(fmt, ...) printf(fmt, __VA_ARGS__) */

#define SCI_ID_CHUNK(id) (((id) >> 8) & 0xf) /* Routing is deep enough to cover bit 12:8, but we use only 12bit NodeIDs */
#define SCI_ID_REGNR(id) (((id) >> 4) & 0xf)
#define SCI_ID_BITNR(id) (((id) >> 0) & 0xf)

/* NB: Only ROUTING_PAIR can be used on longer rings due to the multiple CAM hit scenario */
#define ROUTING_PAIR 1

#undef DEBUG_HARD
#define DEBUG_HARD 3

static inline uint8_t _load_balance(uint8_t dim, sci_t src, sci_t dst)
{
#if defined(ROUTING_PAIR)
	/* Pair-wise load-balancing */
	return ((src & 0xf) + ((src >> 4) & 0xf) + ((src >> 8) & 0xf)) & 1;
#else
	sci_t src2 = src >> (dim * 4);
	sci_t dst2 = dst >> (dim * 4);

#if defined(ROUTING_2QOS)
	/* 2QOS routing (dateline) */
	return ((dst2 < src2) ? 1 : 0);
#elif defined(ROUTING_SHORT)
	/* Shortest path routing */
	int len = cfg_fabric.size[dim];
	int forward = ((len - src2) + dst2) % len;
	int backward = ((src2 + (len - dst2)) + len) % len;
	return ((forward == backward) ? (src2 & 1) :
		(backward < forward) ? 1 : 0);
#else
	return 0;
#endif

#endif
}

uint8_t router0(sci_t src, const int node)
{
	sci_t dst = cfg_nodelist[node].sci;
	uint8_t dim = 0;

	sci_t dst2 = dst;
	sci_t src2 = src;
	while ((src2 ^ dst2) & ~0xf) {
		dim++;
		src2 >>= 4;
		dst2 >>= 4;
	}

	int out = dim * 2 + 1;
	out += (src2 ^ dst2) & 0x1; /* Load balance pairs */
	return out;
}

uint8_t router1(sci_t src, const int node)
{
	sci_t dst = cfg_nodelist[node].sci;
	uint8_t out = 0;

	/* Check usability of dimensions in order */
	for (int i = 0; i < 3; i++) {
		const int shift = dims[i] * 4;
		if (((src >> shift) & 0xf) ^ ((dst >> shift) & 0xf)) {
			out = dims[i] * 2 + 1;
			break;
		}
	}

	out += node % 1; /* Load balance */
	return out;
}

uint8_t Router::_lookup(const sci_t sci, const uint8_t lc, const sci_t dsci) const
{
	const uint16_t offs = (dsci >> 4) & 0xff;
	const uint16_t mask = 1 << (dsci & 0xf);

	// LC routing
	uint8_t out = ((shadow_rtbll[sci][lc][offs] & mask) >> (dsci & 0xf));
	out |= ((shadow_rtblm[sci][lc][offs] & mask) >> (dsci & 0xf)) << 1;
	out |= ((shadow_rtblh[sci][lc][offs] & mask) >> (dsci & 0xf)) << 2;

	// validate
	assertf(out == 0 || (1 << out) & possible_lcs, "Invalid out LC %u", out);

	return out;
}

// lookup: finds next link controller
// @lc: link controller packet is curently on
// @dst: destination SCI
uint8_t Router::lookup(const sci_t sci, const uint8_t lc, const sci_t dsci) const
{
	const uint16_t offs = (dsci >> 4) & 0xff;
	const uint16_t mask = 1 << (dsci & 0xf);

	assertf(lc == 0 || (1 << lc) & possible_lcs, "Invalid input LC %u", lc);

	// if not stripped, keep route out via incoming LC
	if (lc && !(shadow_ltbl[sci][lc][offs] & mask)) {
		printf("|");
		return lc;
	}

	return _lookup(sci, lc, dsci);
}

// update routing tables only for local SCI
// @sci: current SCI being traversed
// @dsci: destination that route entry is for
// @in: which SCC/LC being updated
// @out: which SCC/LC to route to
void Router::table(const sci_t sci, const sci_t dsci, const uint8_t in, const uint8_t out)
{
	assert(in != out);
//	debugf(2, "on %03x, packets to %03x on LC%u route via LC%u%s\n", sci, dsci, in, out, strip ? " (stripped)" : "");

	uint16_t offs = (dsci >> 4) & 0xff;
	uint16_t mask = 1 << (dsci & 0xf);

	if (in) {
		shadow_ltbl[sci][in][offs] |= mask;
		debugf(2, "+");
//		if (sci == 0x110 && in == 2 && dsci == 0x210)
//			printf("0x%p,0x%x", &shadow_ltbl[in][offs], mask);
//		printf("#%03x,%u,%03x#", sci, in, dsci);
	} else
		// strip bit expected to be 0
		assert(!(shadow_ltbl[sci][in][offs] & mask));

	// prevent conflicting overwrite
	const uint8_t cur = _lookup(sci, in, dsci);
	assertf(cur == 0 || cur == out, "New route %u conflicts with existing %u", out, cur);

	shadow_rtbll[sci][in][offs] |= ((out & 1) ? mask : 0);
	shadow_rtblm[sci][in][offs] |= ((out & 2) ? mask : 0);
	shadow_rtblh[sci][in][offs] |= ((out & 4) ? mask : 0);
}

sci_t Router::neigh(sci_t pos, const uint8_t lc) const
{
	const uint8_t axis = (lc - 1) / 2; // 0-2
	int ring_pos = (pos >> (axis * 4)) & 0xf;
	const bool dir = lc & 1;

	debugf(4, "<neigh: pos=0x%03x lc=%u axis=%u", pos, lc, axis);
	if (dir) {
		if (--ring_pos == -1)
			ring_pos = cfg_fabric.size[axis] - 1;
	} else
		ring_pos = (ring_pos + 1) % cfg_fabric.size[axis];

	pos &= ~(0xf << (axis * 4));
	pos |= ring_pos << (axis * 4);

	debugf(4, " pos=%03x>", pos);

	// validate
	for (unsigned i = 0; i < 3; i++)
		assert(((pos >> (i * 4)) & 0xf) < max(cfg_fabric.size[i], 1));

	return pos;
}

// increment path usage
void Router::update(void)
{
	if (bestcost == ~0U) {
		debugf(2, "route %03x > %03x failed to route\n", src, dst);

		for (int n = 0; n < cfg_nodes; n++)
			if (cfg_nodelist[n].sci == src || cfg_nodelist[n].sci == dst)
				cfg_nodelist[n].unavailable = 1;

		return;
	}

	debugf(2, "route %03x > %03x has cost %u:", src, dst, bestcost);
	sci_t pos = src;
	uint8_t lc;
	unsigned offset = 0;

	do {
		lc = bestroute[offset];
		uint8_t lastlc = offset ? bestroute[offset - 1] : 0;

		debugf(1, " %03x,%u:%u", pos, lastlc, lc);
		table(pos, dst, lastlc, lc); // route from SCC to correct LC

		debugf(1, " <%03x,%u:%u>", pos, lc, lastlc);
		table(pos, src, lc, lastlc); // route from SCC to correct LC

		uint8_t axis = (lc - 1) / 2;

		// move pos to next ring change
		sci_t ring_dst = pos & ~(0xf << (axis * 4));
		ring_dst |= dst & (0xf << (axis * 4));
		xbar_usage[pos]++;

		sci_t ring_pos = pos;

		// tally and update route on ringlet towards dst
		while (1) {
			lc_usage[ring_pos][lc - 1]++;
			ring_pos = neigh(ring_pos, lc);

			if (ring_pos == ring_dst)
				break;

			// don't strip the packet, leaving it on the ringlet
			debugf(1, " (%03x,%u:%u)", ring_pos, lc, lc);
		}

		// add route back for non-self route
		while (ring_pos != ring_dst) {
			lc_usage[ring_pos][lc - 1]++;
			ring_pos = neigh(ring_pos, lc);

			// don't strip the packet, leaving it on the ringlet
			debugf(1, " {%03x,%u:%u}", ring_pos, lc, lc);
		}

		pos = ring_dst;
	} while (bestroute[++offset]);

	assert(pos == dst);

	debugf(1, " %03x,%u:0", pos, lc);
	table(pos, dst, lc, 0); // route to SCC

	// route from SCC to correct LC for non-self routes
	if (src != dst) {
		debugf(1, " <%03x,0:%u>", pos, lc);
		table(pos, src, 0, lc);
	}
	debugf(1, "\n");
}

void Router::find(const sci_t pos, unsigned cost, const unsigned offset, const uint8_t available_axes)
{
	if (cost >= bestcost) {
		debugf(3, "<overcost>\n");
		return;
	}

	// save route, allowing route-to-self
	if (pos == dst && offset) {
		memcpy(bestroute, route, offset * sizeof(route[0]));
		bestroute[offset] = 0;
		bestcost = cost;
		debugf(3, "goal @ cost %u\n", cost);
		return;
	}

	for (unsigned lc = 1; lc <= 6; lc++) {
		// avoid unneeded axes or ones previously used
		if (!(available_axes & (1 << ((lc - 1) / 2))) || lcs_disallowed[pos][lc])
			continue;

		// can only use any existing route
		// if at offset 0, lc is 0, else last LC and pos is at offset-1
		const uint8_t xlc = _lookup(pos, offset ? route[offset-1] : 0, dst);
		debugf(3, "<xlc=%u>", xlc);
		if (xlc && lc != xlc) {
			debugf(3, "<existing>\n");
			return;
		}

		const uint8_t axis = (lc - 1) / 2;
		bool skip = 0;

		debugf(3, "<offset=%u axes=%u", offset, available_axes);
		// tally whole ring as response has to continue on same rings
		for (unsigned ring_pos = 0; ring_pos < cfg_fabric.size[axis]; ring_pos++) {
			sci_t sci = (pos & ~(0xf << (axis * 4))) | (ring_pos << (axis * 4));

			if (lcs_disallowed[sci][lc - 1]) {
				skip = 1;
				break;
			}

			cost += LC_COST + lc_usage[sci][lc - 1] * LC_USAGE_COST;
		}

		if (skip)
			continue;

		// tally ring change
		cost += XBAR_COST + xbar_usage[pos] * XBAR_USAGE_COST;

		// move pos to next ring change
		sci_t next = pos & ~(0xf << (axis * 4));
		next |= dst & (0xf << (axis * 4));

		route[offset] = lc;
		debugf(3, " pos=0x%03x lc=%u>", pos, lc);
		find(next, cost, offset + 1, available_axes & ~(1 << ((lc - 1) / 2)));
	}
}

void Router::router(const sci_t _src, const sci_t _dst)
{
	src = _src;
	dst = _dst;

	if (finished[src][dst])
		return;

	uint8_t available_axes = 0;

	// route to self can use any path
	if (src == dst) {
		for (uint8_t i = 0; i < 3; i++)
			if (cfg_fabric.size[i])
				available_axes |= 1 << i;
	} else {
		for (uint8_t i = 0; i < 3; i++)
			if ((cfg_fabric.size[i]) && ((src ^ dst) & (0xf << (i * 4))))
				available_axes |= 1 << i;
	}

	bestcost = ~0U;
	find(src, 0, 0, available_axes);
	update();

	// route back already calculated
	finished[dst][src] = 1;
}

Router::Router(const sci_t sci): possible_lcs(0), local_sci(sci)
{
	memset(finished, 0, sizeof(finished));
	memset(bestroute, 0, sizeof(bestroute));
	memset(route, 0, sizeof(route));
	memset(xbar_usage, 0, sizeof(xbar_usage));
	memset(lc_usage, 0, sizeof(lc_usage));
	memset(lcs_disallowed, 0, sizeof(lcs_disallowed));

	memset(shadow_rtbll, 0, sizeof(shadow_rtbll));
	memset(shadow_rtblm, 0, sizeof(shadow_rtblm));
	memset(shadow_rtblh, 0, sizeof(shadow_rtblh));
	memset(shadow_ltbl, 0, sizeof(shadow_ltbl));

	for (unsigned i = 0; i < 3; i++)
		if (cfg_fabric.size[i])
			possible_lcs |= 3 << (i * 2 + 1);
}

void Router::run(void)
{
	for (unsigned sz = 0; sz < max(cfg_fabric.size[2], 1); sz++)
		for (unsigned sy = 0; sy < max(cfg_fabric.size[1], 1); sy++)
			for (unsigned sx = 0; sx < max(cfg_fabric.size[0], 1); sx++)
				for (unsigned dz = 0; dz < max(cfg_fabric.size[2], 1); dz++)
					for (unsigned dy = 0; dy < max(cfg_fabric.size[1], 1); dy++)
						for (unsigned dx = 0; dx < max(cfg_fabric.size[0], 1); dx++)
							router(SCI(sx, sy, sz), SCI(dx, dy, dz));
}

void Router::disable_node(const uint16_t sci)
{
	for (unsigned lc = 0; lc < 6; lc++)
		lcs_disallowed[sci][lc] = 1;
}

void Router::show_usage(void) const
{
	printf("\nUsage on %03x:    LC:   1   2   3   4\n", local_sci);
	for (unsigned z = 0; z < max(cfg_fabric.size[2], 1); z++) {
		for (unsigned y = 0; y < max(cfg_fabric.size[1], 1); y++) {
			for (unsigned x = 0; x < max(cfg_fabric.size[0], 1); x++) {
				printf("%03x xbar %5u, lcs:", SCI(x, y, z), xbar_usage[SCI(x, y, z)]);
				for (uint8_t i = 1; i <= 6; i++)
					if (possible_lcs & (1 << i))
						printf(" %3u", lc_usage[SCI(x, y, z)][i - 1]);
				printf("\n");
			}
		}
	}

	printf("\nRouting tables on %03x:\nLC       0  1  2  3  4  5  6\n", local_sci);
	for (int n = 0; n < cfg_nodes; n++) {
		printf("to %03x:", cfg_nodelist[n].sci);

		for (unsigned lc = 0; lc <= 6; lc++)
			if ((1 << lc) & possible_lcs)
				printf(" %u", lookup(cfg_nodelist[n].sci, lc, cfg_nodelist[n].sci));
		printf("\n");
	}
}

#ifdef LEGACY
/* Route all 256 bxbar entries for chunk corresponding to "dest" over "link"
 * on "sci" */
void add_chunk_route(uint16_t dest, const sci_t sci, uint8_t link)
{
	int scc = ((sci == 0xfff0) || ((sci & 0xf000) == 0));
	uint16_t base = scc ? H2S_CSR_G0_ROUT_LCTBL00 : LC3_CSR_ROUT_LCTBL00;
	uint16_t reg;
	dnc_write_csr(sci,
	              scc
	              ? H2S_CSR_G0_ROUT_TABLE_CHUNK
	              : LC3_CSR_SW_INFO3,
	              SCI_ID_CHUNK(dest));

	for (reg = 0; reg < 16; reg++) {
		dnc_write_csr(sci, base + OFFS_RTBLL + reg * 4,
		              (link & 1) ? 0xffff : 0);
		dnc_write_csr(sci, base + OFFS_RTBLM + reg * 4,
		              (link & 2) ? 0xffff : 0);
		dnc_write_csr(sci, base + OFFS_RTBLH + reg * 4,
		              (link & 4) ? 0xffff : 0);

		if (!scc) dnc_write_csr(sci, base + OFFS_LTBL + reg * 4, 0xffff);
	}

	DEBUG_ROUTE("add_chunk_route: on %04x to %04x over %d\n",
	            sci, dest, link);
}

/* Unroute all 256 entries for chunk corresponding to "dest" on "sci" */
void del_chunk_route(uint16_t dest, const sci_t sci)
{
	int scc = ((sci == 0xfff0) || ((sci & 0xf000) == 0));

	if (!scc) {
		dnc_write_csr(sci, LC3_CSR_SW_INFO3, SCI_ID_CHUNK(dest));

		for (uint16_t reg = 0; reg < 16; reg++) {
			dnc_write_csr(sci, LC3_CSR_ROUT_LCTBL00 + OFFS_LTBL + reg * 4, 0x0000);
		}

		DEBUG_ROUTE("del_chunk_route: on %04x from %04x\n",
		            sci, dest);
	}
}

/* Set route on "sci" towards "dest" over "link"; bitmask "width"
   signifies width of route */
void set_route(uint16_t dest, const sci_t sci, uint16_t width, uint8_t link)
{
	int scc = ((sci == 0xfff0) || ((sci & 0xf000) == 0));
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);
	uint16_t base = scc ? H2S_CSR_G0_ROUT_LCTBL00 : LC3_CSR_ROUT_LCTBL00;
	dnc_write_csr(sci,
	              scc
	              ? H2S_CSR_G0_ROUT_TABLE_CHUNK
	              : LC3_CSR_SW_INFO3,
	              SCI_ID_CHUNK(dest));
	dnc_write_csr(sci, base + OFFS_RTBLL + reg, ((link & 1) ? width : 0) << bit);
	dnc_write_csr(sci, base + OFFS_RTBLM + reg, ((link & 2) ? width : 0) << bit);
	dnc_write_csr(sci, base + OFFS_RTBLH + reg, ((link & 4) ? width : 0) << bit);

	if (!scc) dnc_write_csr(sci, base + OFFS_LTBL + reg, width << bit);

	DEBUG_ROUTE("sdd_route: on %04x to %04x/%x over %d\n",
	            sci, dest, width, link);
}

/* Add route on "sci" towards "dest" over "link"; bitmask "width"
   signifies width of route */
void add_route(uint16_t dest, const sci_t sci, uint16_t width, uint8_t link)
{
	int scc = ((sci == 0xfff0) || ((sci & 0xf000) == 0));
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);
	uint16_t base = scc ? H2S_CSR_G0_ROUT_LCTBL00 : LC3_CSR_ROUT_LCTBL00;
	uint32_t csr;
	dnc_write_csr(sci,
	              scc
	              ? H2S_CSR_G0_ROUT_TABLE_CHUNK
	              : LC3_CSR_SW_INFO3,
	              SCI_ID_CHUNK(dest));

	if (!scc) {
		csr = dnc_read_csr(sci, LC3_CSR_CONFIG4);
		csr |= (1 << 6); /* CONFIG4.tblrd is needed to read routing tables through CSR */
		dnc_write_csr(sci, LC3_CSR_CONFIG4, csr);
	}

	csr = dnc_read_csr(sci, base + OFFS_RTBLL + reg);
	csr = (csr & ~(width << bit)) | (((link & 1) ? width : 0) << bit);
	dnc_write_csr(sci, base + OFFS_RTBLL + reg, csr);
	csr = dnc_read_csr(sci, base + OFFS_RTBLM + reg);
	csr = (csr & ~(width << bit)) | (((link & 2) ? width : 0) << bit);
	dnc_write_csr(sci, base + OFFS_RTBLM + reg, csr);
	csr = dnc_read_csr(sci, base + OFFS_RTBLH + reg);
	csr = (csr & ~(width << bit)) | (((link & 4) ? width : 0) << bit);
	dnc_write_csr(sci, base + OFFS_RTBLH + reg, csr);

	if (!scc) {
		csr = dnc_read_csr(sci, base + OFFS_LTBL + reg);
		csr |= (width << bit);
		dnc_write_csr(sci, base + OFFS_LTBL + reg, csr);
		csr = dnc_read_csr(sci, LC3_CSR_CONFIG4);
		csr &= ~(1 << 6); /* Disable CONFIG4.tblrd to enable normal routing operation */
		dnc_write_csr(sci, LC3_CSR_CONFIG4, csr);
	}

	DEBUG_ROUTE("add_route: on %04x to %04x/%x over %d\n",
	            sci, dest, width, link);
}

/* Remove route on "sci" towards "dest"; bitmask "width" signifies
   width of route */
void del_route(uint16_t dest, const sci_t sci, uint16_t width)
{
	int scc = ((sci == 0xfff0) || ((sci & 0xf000) == 0));
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);

	if (!scc) {
		dnc_write_csr(sci, LC3_CSR_SW_INFO3, SCI_ID_CHUNK(dest));
		uint32_t csr = dnc_read_csr(sci, LC3_CSR_CONFIG4);
		csr |= (1 << 6); /* CONFIG4.tblrd is needed to read routing tables through CSR */
		dnc_write_csr(sci, LC3_CSR_CONFIG4, csr);
		csr = dnc_read_csr(sci, LC3_CSR_ROUT_LCTBL00 + OFFS_LTBL + reg);
		csr &= ~(width << bit);
		dnc_write_csr(sci, LC3_CSR_ROUT_LCTBL00 + OFFS_LTBL + reg, csr);
		csr = dnc_read_csr(sci, LC3_CSR_CONFIG4);
		csr &= ~(1 << 6); /* Disable CONFIG4.tblrd to enable normal routing operation */
		dnc_write_csr(sci, LC3_CSR_CONFIG4, csr);
		DEBUG_ROUTE("del_route: on %04x to %04x/%x\n",
		            sci, dest, width);
	}
}

/* Set route towards "dest" over "link" on blink id "bid" via "sci";
   bitmask "width" signifies width of route */
void set_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width, uint8_t link)
{
	int scc = (bid == 0);
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);
	uint16_t base = scc ? H2S_CSR_G0_ROUT_LCTBL00 : LC3_CSR_ROUT_LCTBL00;
	dnc_write_csr_geo(sci, bid,
	                  scc
	                  ? H2S_CSR_G0_ROUT_TABLE_CHUNK
	                  : LC3_CSR_SW_INFO3,
	                  SCI_ID_CHUNK(dest));
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLL + reg, ((link & 1) ? width : 0) << bit);
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLM + reg, ((link & 2) ? width : 0) << bit);
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLH + reg, ((link & 4) ? width : 0) << bit);

	if (!scc) dnc_write_csr_geo(sci, bid, base + OFFS_LTBL + reg, width << bit);

	DEBUG_ROUTE("set_route_geo: on %04x bid %d to %04x/%x over %d\n",
	            sci, bid, dest, width, link);
}

/* Add route towards "dest" over "link" on blink id "bid" via "sci";
   bitmask "width" signifies width of route */
void add_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width, uint8_t link)
{
	int scc = (bid == 0);
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);
	uint16_t base = scc ? H2S_CSR_G0_ROUT_LCTBL00 : LC3_CSR_ROUT_LCTBL00;
	uint32_t csr;
	dnc_write_csr_geo(sci, bid,
	                  scc
	                  ? H2S_CSR_G0_ROUT_TABLE_CHUNK
	                  : LC3_CSR_SW_INFO3,
	                  SCI_ID_CHUNK(dest));

	if (!scc) {
		csr = dnc_read_csr_geo(sci, bid, LC3_CSR_CONFIG4);
		csr |= (1 << 6); /* CONFIG4.tblrd is needed to read routing tables through CSR */
		dnc_write_csr_geo(sci, bid, LC3_CSR_CONFIG4, csr);
	}

	csr = dnc_read_csr_geo(sci, bid, base + OFFS_RTBLL + reg);
	csr = (csr& ~(width << bit)) | (((link & 1) ? width : 0) << bit);
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLL + reg, csr);
	csr = dnc_read_csr_geo(sci, bid, base + OFFS_RTBLM + reg);
	csr = (csr & ~(width << bit)) | (((link & 2) ? width : 0) << bit);
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLM + reg, csr);
	csr = dnc_read_csr_geo(sci, bid, base + OFFS_RTBLH + reg);
	csr = (csr & ~(width << bit)) | (((link & 4) ? width : 0) << bit);
	dnc_write_csr_geo(sci, bid, base + OFFS_RTBLH + reg, csr);

	if (!scc) {
		csr = dnc_read_csr_geo(sci, bid, base + OFFS_LTBL + reg);
		csr |= (width << bit);
		dnc_write_csr_geo(sci, bid, base + OFFS_LTBL + reg, csr);
		csr = dnc_read_csr_geo(sci, bid, LC3_CSR_CONFIG4);
		csr &= ~(1 << 6); /* Disable CONFIG4.tblrd to enable normal routing operation */
		dnc_write_csr_geo(sci, bid, LC3_CSR_CONFIG4, csr);
	}

	DEBUG_ROUTE("add_route_geo: on %04x bid %d to %04x/%x over %d\n",
	            sci, bid, dest, width, link);
}

/* Remove route towards "dest" on blink id "bid" via "sci"; bitmask
   "width" signifies width of route */
void del_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width)
{
	int scc = (bid == 0);
	uint16_t reg = SCI_ID_REGNR(dest) * 4;
	uint16_t bit = SCI_ID_BITNR(dest);

	if (!scc) {
		dnc_write_csr_geo(sci, bid, LC3_CSR_SW_INFO3, SCI_ID_CHUNK(dest));
		uint32_t csr = dnc_read_csr_geo(sci, bid, LC3_CSR_CONFIG4);
		csr |= (1 << 6); /* CONFIG4.tblrd is needed to read routing tables through CSR */
		dnc_write_csr_geo(sci, bid, LC3_CSR_CONFIG4, csr);
		csr = dnc_read_csr_geo(sci, bid, LC3_CSR_ROUT_LCTBL00 + OFFS_LTBL + reg);
		csr &= ~(width << bit);
		dnc_write_csr_geo(sci, bid, LC3_CSR_ROUT_LCTBL00 + OFFS_LTBL + reg, csr);
		csr = dnc_read_csr_geo(sci, bid, LC3_CSR_CONFIG4);
		csr &= ~(1 << 6); /* Disable CONFIG4.tblrd to enable normal routing operation */
		dnc_write_csr_geo(sci, bid, LC3_CSR_CONFIG4, csr);
		DEBUG_ROUTE("del_route_geo: on %04x bid %d to %04x/%x\n",
		            sci, bid, dest, width);
	}
}
#endif
