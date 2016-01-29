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

#ifndef __DNC_ROUTE
#define __DNC_ROUTE 1

#include "dnc-bootloader.h"

#define MAXPATH (16 + 16 + 16)
#define XBAR_COST 1
#define XBAR_USAGE_COST 1
#define LC_COST 1
#define LC_USAGE_COST 1

typedef uint8_t (*router_t)(sci_t, const int);

class Router {
	bool lcs_disallowed[4096][6];
	bool finished[4096][4096];
	unsigned xbar_usage[4096]; // only used when changing rings
	unsigned lc_usage[4096][6]; // send buffering
	uint8_t route[MAXPATH], bestroute[MAXPATH];
	uint8_t possible_lcs;
	unsigned bestcost;
	sci_t src, dst;
	sci_t local_sci;

	void find(const sci_t pos, unsigned cost, const unsigned offset, const uint8_t available_lcs);
	void strip(const sci_t sci, const sci_t dsci, const uint8_t in);
	uint8_t _lookup(const sci_t sci, const uint8_t lc, const sci_t dsci) const;
	void table(const sci_t sci, const sci_t dsci, const uint8_t in, const uint8_t out);
	void update(void);
	void router(const sci_t _src, const sci_t _dst);
public:
	uint16_t shadow_rtbll[4096][7][256];
	uint16_t shadow_rtblm[4096][7][256];
	uint16_t shadow_rtblh[4096][7][256];
	uint16_t shadow_ltbl[4096][7][256];

	sci_t neigh(sci_t pos, const uint8_t lc) const;
	uint8_t lookup(const sci_t sci, const uint8_t lc, const sci_t dsci) const;
	Router(const sci_t sci);
	void run(void);
	void disable_node(const uint16_t sci);
	void show_usage(void) const;
};

extern uint8_t dims[];

uint8_t router0(sci_t src, const int node);
uint8_t router1(sci_t src, const int node);
#ifdef LEGACY
void add_chunk_route(uint16_t dest, const sci_t sci, uint8_t link);
void del_chunk_route(uint16_t dest, const sci_t sci);
void set_route(uint16_t dest, const sci_t sci, uint16_t width, uint8_t link);
void add_route(uint16_t dest, const sci_t sci, uint16_t width, uint8_t link);
void del_route(uint16_t dest, const sci_t sci, uint16_t width);
void set_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width, uint8_t link);
void add_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width, uint8_t link);
void del_route_geo(uint16_t dest, const sci_t sci, uint8_t bid, uint16_t width);
#endif
#endif
