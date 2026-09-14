/* Host stub. The simulator never calls cfg_load()/cfg_save() -- test
 * scenarios call cfg_defaults() directly and mutate the struct -- so these
 * are declared only so cfg.c links; always returning false from load()
 * matches the original stub's intent (an "empty flash" that always falls
 * through to defaults). */
#include "port_persist.h"

bool port_persist_load(cfg_t *out)  { (void)out; return false; }
bool port_persist_save(const cfg_t *in) { (void)in; return true; }
