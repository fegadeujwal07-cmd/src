# ECBS–TM Authority Migration — Handover

**Date:** 2026-11-17 (group passage scheduler pass)  
**Status:** Identity handshake + group passage scheduler scaffolding landed; **still not production-ready**.

## Honest status

ECBS now builds occurrence groups, passage actions, dual dependency graphs (mandatory vs candidate + Tarjan), and a resumable PBS-style scheduler that returns verified incumbents or typed WAIT (`WAIT_CLEARANCE` / `WAIT_SEARCH` / `WAIT_HARDHOLD`). TM commits the **exact proposal identity** (canonical `task_id` ≠ execution `order_id`), gates new queries while a commit is open, and checks peer pending reservations. Unit scenarios exercise identity, chain clears, exclusive crossing, and search resume — **not** a live MQTT capacity proof for 10–30 robots.

## This pass

| Item | Notes |
|------|--------|
| Canonical identity | Proposal/commit use ScheduleCache / dispatch UUID; `execution_order_id` is separate |
| Conflict lifecycle | Active/Held/Cleared/Retired; retain beyond horizon |
| Authority ledger | Independent of group merge/split |
| `actions_compatible` | Geometry/intervals — not conflict-id equality |
| Dual graphs | Mandatory vs candidate; A→B = clearance-before-start; B2B front→rear |
| Resumable search | Node budget; verified incumbent; no fallback GO |
| Fairness | Bypass before priority; counters on exclusive commit only |
| TM txn | Provisional multi-robot check; timeout retains reservation |

## Acceptance still required before scaling

- Live two-robot MQTT: FREE_PROPOSAL → ACCEPTED → PathRelease with UUID ≠ order `"1"`
- Two incompatible commits race → one winner
- Idle robot on exit → winner holds before enter
- Replay determinism under node-expansion budget

## Rollback

`ecbs_owns_arbitration: false` or git revert.
