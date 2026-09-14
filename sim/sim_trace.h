/**
 * @file    sim_trace.h
 * @brief   One event stream, two sinks: a JSON trace and the live terminal.
 *
 * Both views consume the same callbacks, so they cannot drift apart or tell
 * different stories from the assertions. The JSON trace carries full detail
 * for a bounded window of superframes plus whole-run aggregates; a 2000
 * superframe run is ~208k transmissions, far too much to embed in a page.
 */
#ifndef SIM_TRACE_H_
#define SIM_TRACE_H_

#include <stdint.h>
#include <stdbool.h>
#include "cfg.h"

/* Which kind of transmission, for colouring and for the slot strip. */
typedef enum { TK_POLL = 0, TK_RESP, TK_FINAL, TK_REPORT } tkind_t;

const char *trace_kind_str(tkind_t k);

/** Start writing a trace. @p window is how many superframes of per-event
 *  detail to keep; 0 disables the JSON sink. */
void trace_open(const char *path, uint32_t window);

/** Turn on the live terminal view. @p realtime maps simulated onto wall time
 *  (0 = run flat out, redraw throttled). */
void trace_live(double realtime);

bool trace_active(void);

void trace_meta(const cfg_t *c, const char *scenario, uint32_t n_tags,
                uint32_t n_anch, uint64_t seed, uint32_t superframes,
                uint32_t loss_x100);
void trace_node(uint32_t id, uint8_t role, uint16_t addr, double x, double y,
                double ppm, double drift_ppm, double drift_hz);
void trace_nodes_done(void);

/** Called as each superframe opens, so the sinks know where the window ends. */
void trace_superframe(uint32_t sfn, int64_t t_dtu);

void trace_tx(int64_t t0, int64_t t1, uint32_t node, tkind_t kind,
              uint32_t slot, uint32_t sfn);
void trace_loss(int64_t t, uint32_t tx, uint32_t rx, tkind_t kind);
void trace_sync_ev(int64_t t, uint32_t node, const char *state,
                   const char *event, int32_t err_us, int32_t beta_ppb,
                   uint32_t lambda);
void trace_collision(int64_t t, uint32_t a, uint32_t b, double overlap_us);
void trace_frame_lost(int64_t t, uint32_t tag, uint32_t sfn);

void trace_range(int64_t t, uint32_t tag, uint32_t anchor, uint32_t slot,
                 uint32_t sfn, int32_t d_mm, double d_true_mm,
                 uint32_t tof_ps, uint8_t flags, const uint64_t ts6[6]);

/** Per-pair aggregates for the whole run, then close the file. */
void trace_summary_pair(uint32_t tag, uint32_t anchor, uint32_t count,
                        double hz, double bias_mm, double peak_mm);
void trace_summary_tag(uint32_t tag, uint32_t polls, double hz,
                       int64_t jitter_ns, const char *state,
                       uint32_t rearm, uint32_t freerun, uint32_t dup,
                       uint32_t drift, int32_t last_err_us);
void trace_summary_run(uint32_t transmissions, uint32_t collisions,
                       double tightest_gap_us, bool pass);
/** Repaint the live view. Cheap and throttled; call it freely. */
void trace_live_draw(int64_t now_dtu, bool force);

void trace_close(void);

#endif /* SIM_TRACE_H_ */
