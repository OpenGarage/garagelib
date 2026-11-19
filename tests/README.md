# Native decoder regression

From the garagelib root:

```sh
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus2_decode.cpp secplus.c -o /tmp/og-test-secplus2-decode
/tmp/og-test-secplus2-decode
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus1.cpp secplus.c -o /tmp/og-test-secplus1
/tmp/og-test-secplus1
g++ -std=gnu++17 -I. tests/test_secplus1_short_cycles.cpp -o /tmp/og-test-secplus1-short-cycles
/tmp/og-test-secplus1-short-cycles
g++ -std=gnu++17 -I. tests/test_secplus1_diagnostics.cpp -o /tmp/og-test-secplus1-diagnostics
/tmp/og-test-secplus1-diagnostics
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus2_recovery.cpp secplus.c -o /tmp/og-test-secplus2-recovery
/tmp/og-test-secplus2-recovery
```

Uses the real wireline codec and packet decoder with minimal host-only Arduino/UART stubs. Exercises explicit Off/On, Toggle and reserved LIGHT/LOCK values from both previous states, and checks the other field is unchanged. The original source fails the explicit-On case; the corrected source passes all 16 cases. This does not simulate electrical timing or prove opener acceptance.

The Sec+ 1.0 suite exercises the production receiver and scheduler: parity, release echoes, confirmation, independent expiry, 12,155 interval/latency combinations, timer wraparound, bounded waits, no replay, two release bytes, partial-write cleanup, UART pump, lifecycle teardown and overdue release under backlog. Electrical timing still needs hardware verification.

The short-cycle suite covers query/reply pairs decoded together after a delayed UART read (0–5 ms apart). A reply read more than 60 ms after its query because the loop stalled between the two reads is handled the same way. Such a cycle is skipped without discarding the learned panel cadence, and the next clean exchange is checked against it. It replays a captured garage trace in which the previous rule dropped training twice and a command expired. It also checks that a faster cadence, two cycles decoded in one pass, more than four skipped cycles, a read too late to attribute and a missed poll still drop training.

It also replays the observed blocked-beam light/lock variant (0x5d → 0x1d →
0x5d), checking both accepted prefixes across light/lock combinations, bad parity,
confirmation, expiry and rejection of other prefixes.

The Sec+ 2.0 recovery suite also injects a corrupt queued release after a transmitted press: controls must remain faulted, the invalid release must not transmit, and GET_STATUS queries must continue without replaying an action.

The startup-query tests decode the transmitted OPENINGS/STATUS pair and verify adjacent rolling codes, a single post-sync openings refresh, opening-count decoding, control/release priority, and another single refresh after stale-link recovery.
