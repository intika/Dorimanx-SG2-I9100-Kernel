#ifndef CEPH_CRUSH_MAPPER_H
#define CEPH_CRUSH_MAPPER_H

/*
 * CRUSH functions for find rules and then mapping an input to an
 * output set.
 *
 * LGPL2
 */

#include "crush.h"

extern int crush_find_rule(struct crush_map *map, int pool, int type, int size);
extern int crush_do_rule(struct crush_map *map,
			 int ruleno,
			 int x, int *result, int result_max,
<<<<<<< HEAD
			 int forcefeed,    /* -1 for none */
			 __u32 *weights);
=======
			 const __u32 *weights, int weight_max,
			 int *scratch);
>>>>>>> d891ea2... Merge branch 'for-linus' of git://git.kernel.org/pub/scm/linux/kernel/git/sage/ceph-client

#endif
