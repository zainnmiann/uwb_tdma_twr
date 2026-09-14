#include "port_persist.h"

#include <Preferences.h>
#include <string.h>

static Preferences s_prefs;

bool port_persist_load(cfg_t *out)
{
    if (!s_prefs.begin("tdma_twr", true))
    {
        return false;
    }
    size_t n = s_prefs.getBytesLength("cfg");
    bool ok = false;
    if (n == sizeof(cfg_t))
    {
        ok = (s_prefs.getBytes("cfg", out, sizeof(cfg_t)) == sizeof(cfg_t));
    }
    s_prefs.end();
    return ok;
}

bool port_persist_save(const cfg_t *in)
{
    if (!s_prefs.begin("tdma_twr", false))
    {
        return false;
    }
    size_t n = s_prefs.putBytes("cfg", in, sizeof(cfg_t));
    s_prefs.end();
    return n == sizeof(cfg_t);
}
