#include "support/AllocationGuard.h"

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<std::size_t> g_allocationCount{0};
} // namespace

namespace cnpg::test {

std::size_t allocationCount() noexcept { return g_allocationCount.load(); }

void resetAllocationCount() noexcept { g_allocationCount.store(0); }

} // namespace cnpg::test

// Global replacement operator new/delete -- the single definition point for the whole
// cnpg_tests binary (see the header comment: a second TU defining these is a link error).
// Routed through malloc/free so program behavior is otherwise unchanged; every allocation
// across the whole process increments g_allocationCount, but individual tests only assert on a
// delta measured tightly around the call under test (see AllocationGuard.h usage note).
void* operator new(std::size_t size) {
    g_allocationCount.fetch_add(1, std::memory_order_relaxed);
    if (void* ptr = std::malloc(size))
        return ptr;
    throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }
