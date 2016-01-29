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

#define _GNU_SOURCE 1

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>

#include "dnc-config.h"
#include "dnc-commonlib.h"
#include "dnc-route.h"

uint8_t dims[] = {0, 1, 2};
int cfg_nodes;
struct fabric_info cfg_fabric;
struct node_info *cfg_nodelist;
bool test_manufacture = 0;
static Router *routers[0x1000];

void broadcast_error(const bool persistent, const char *format, ...)
{
	exit(1);
}

static void walk(const sci_t src, const sci_t dst)
{
	sci_t pos = src;
	uint8_t lc = 0;
	unsigned hops = 0;

	printf("%03x to %03x:", src, dst);

	while (1) {
		printf(" %03x,%u", pos, lc);
		lc = routers[pos]->lookup(pos, lc, dst);
		printf(":%u", lc);
		if (lc == 0)
			break;

		pos = routers[pos]->neigh(pos, lc);

		if (hops++ > 48) {
			printf("%03x > %03x cyclic\n", src, dst);
			return;
		}
	};

	printf("\n");
}

int main(int argc, char **argv)
{
	cfg_fabric.size[0] = 3;
	cfg_fabric.size[1] = 3;
	cfg_fabric.size[2] = 2;
	printf("%ux%ux%u fabric\n", cfg_fabric.size[0], cfg_fabric.size[1], cfg_fabric.size[2]);

	cfg_nodes = max(cfg_fabric.size[0], 1) * max(cfg_fabric.size[1], 1) * max(cfg_fabric.size[2], 1);
	cfg_nodelist = (struct node_info *)malloc(cfg_nodes * sizeof(struct node_info));
	assert(cfg_nodelist);

	unsigned n = 0;

	for (unsigned z = 0; z < max(cfg_fabric.size[2], 1); z++) {
		for (unsigned y = 0; y < max(cfg_fabric.size[1], 1); y++) {
			for (unsigned x = 0; x < max(cfg_fabric.size[0], 1); x++) {
				const sci_t sci = SCI(x, y, z);
				cfg_nodelist[n++].sci = sci;
				printf("instantiating %03x\n", sci);
				routers[sci] = new Router(sci);
//				routers[sci]->disable_node(0x011);
				routers[sci]->run();
			}
		}
	}

	for (unsigned z = 0; z < max(cfg_fabric.size[2], 1); z++)
		for (unsigned y = 0; y < max(cfg_fabric.size[1], 1); y++)
			for (unsigned x = 0; x < max(cfg_fabric.size[0], 1); x++)
				routers[SCI(x,y,z)]->show_usage();
	printf("\n");

	for (unsigned sz = 0; sz < max(cfg_fabric.size[2], 1); sz++)
		for (unsigned sy = 0; sy < max(cfg_fabric.size[1], 1); sy++)
			for (unsigned sx = 0; sx < max(cfg_fabric.size[0], 1); sx++)
				for (unsigned dz = 0; dz < max(cfg_fabric.size[2], 1); dz++)
					for (unsigned dy = 0; dy < max(cfg_fabric.size[1], 1); dy++)
						for (unsigned dx = 0; dx < max(cfg_fabric.size[0], 1); dx++)
							walk(SCI(sx, sy, sz), SCI(dx, dy, dz));

	for (unsigned z = 0; z < max(cfg_fabric.size[2], 1); z++)
		for (unsigned y = 0; y < max(cfg_fabric.size[1], 1); y++)
			for (unsigned x = 0; x < max(cfg_fabric.size[0], 1); x++)
				delete routers[SCI(x,y,z)];

	free(cfg_nodelist);
	return 0;
}
