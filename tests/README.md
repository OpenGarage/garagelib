# Native decoder regression

From the garagelib root:

```sh
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus2_decode.cpp secplus.c -o /tmp/og-test-secplus2-decode
/tmp/og-test-secplus2-decode
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus1.cpp secplus.c -o /tmp/og-test-secplus1
/tmp/og-test-secplus1
g++ -std=gnu++17 -Itests/stubs -I. tests/test_secplus2_recovery.cpp secplus.c -o /tmp/og-test-secplus2-recovery
/tmp/og-test-secplus2-recovery
```

Uses the real wireline codec and packet decoder with minimal host-only Arduino/UART stubs. Exercises explicit Off/On, Toggle and reserved LIGHT/LOCK values from both previous states, and checks the other field is unchanged. The original source fails the explicit-On case; the corrected source passes all 16 cases. This does not simulate electrical timing or prove opener acceptance.

The Sec+ 1.0 suite exercises the production receiver and scheduler: parity, release echoes, confirmation, independent expiry, 11,492 interval/latency combinations, timer wraparound, bounded waits, no replay, two release bytes, partial-write cleanup, UART pump, lifecycle teardown and overdue release under backlog. Electrical timing still needs hardware verification.

The Sec+ 2.0 recovery suite also injects a corrupt queued release after a transmitted press: controls must remain faulted, the invalid release must not transmit, and GET_STATUS queries must continue without replaying an action.
