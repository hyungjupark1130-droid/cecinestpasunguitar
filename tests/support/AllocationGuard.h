#pragma once

#include <cstddef>

// AllocationGuard -- shared counting-new/delete instrumentation for the "no allocation"
// [contract] tests across cnpg_tests. docs/plan.md Task P1.2 acceptance criteria calls for
// "allocate performs no allocation (counting-new test)"; Task P1.5 acceptance criteria (line
// 1078) calls for "process verified alloc-free (counting-new test wrapping a 1000-block run)";
// Task P2.2 acceptance criteria (line 1204) refers to "the P1 allocation-guard hook" as a single
// shared mechanism reused across tasks. The global operator new/operator delete overrides that
// make the counting possible are defined in exactly ONE translation unit,
// tests/support/AllocationGuard.cpp, linked once into cnpg_tests -- a second TU defining
// operator new(std::size_t) would be a duplicate-symbol link error.
//
// Usage: cnpg::test::resetAllocationCount() immediately before the call under test, then assert
// cnpg::test::allocationCount() == 0 (or compare a before/after delta) immediately after.

namespace cnpg::test {

// Total heap allocations observed by the global operator new override since the last
// resetAllocationCount() call (or since program start, if never called). Counts every
// allocation in the whole cnpg_tests process, not just the code under test -- callers are
// expected to reset() immediately before, and read immediately after, the call they're
// measuring.
std::size_t allocationCount() noexcept;

// Zeroes the counter.
void resetAllocationCount() noexcept;

} // namespace cnpg::test
