/* What the host's own MMU could cost as the guest's TLB: guest pages mapped at host addresses (a window per
   address space) so translated code reaches them with no table lookup, faults filling the window.
   The numbers that decide it: a page mapped into the window, a fault taken and filled, a page's
   protection changed, a page dropped, the whole window emptied (a TLB flush). Runs on the Mac and,
   through Henge's autopilot ("hostmmu"), on a device. */
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/vm_map.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE 16384ull
#define RAM_BYTES (256ull << 20)
#define WINDOW_BYTES (4ull << 30)
#define N 8192 /* pages worked on */

static uint8_t *ram, *window;
static uint32_t order[N];

static double now_ns(void) {
    static mach_timebase_info_data_t tb;
    if (!tb.denom) mach_timebase_info(&tb);
    return (double)mach_absolute_time() * tb.numer / tb.denom;
}

/* Guest page `g` of RAM shown at window page `w`. */
static int map_page(uint64_t w, uint64_t g, int writable) {
    vm_address_t at = (vm_address_t)(uintptr_t)(window + w * PAGE);
    vm_prot_t cur, max;
    kern_return_t kr = vm_remap(mach_task_self(), &at, PAGE, 0, VM_FLAGS_FIXED | VM_FLAGS_OVERWRITE, mach_task_self(),
                                     (vm_address_t)(uintptr_t)(ram + g * PAGE), 0, &cur, &max, VM_INHERIT_NONE);
    if (kr != KERN_SUCCESS) return -1;
    if (!writable) return vm_protect(mach_task_self(), at, PAGE, 0, VM_PROT_READ) == KERN_SUCCESS ? 0 : -1;
    return 0;
}

static volatile uint64_t faults;
static void on_fault(int sig, siginfo_t *si, void *uc) {
    (void)sig, (void)uc;
    uint8_t *a = si->si_addr;
    if (a < window || a >= window + WINDOW_BYTES) _exit(3);
    uint64_t w = (uint64_t)(a - window) / PAGE;
    if (map_page(w, w % (RAM_BYTES / PAGE), 1) != 0) _exit(4);
    faults++;
}

static int empty_window(void) {
    return mmap(window, WINDOW_BYTES, PROT_NONE, MAP_PRIVATE | MAP_ANON | MAP_FIXED, -1, 0) == window ? 0 : -1;
}

int hv_proto_hostmmu(void (*say)(const char *));
int hv_proto_hostmmu_text(char *out, size_t cap);
int hv_proto_hostmmu(void (*say)(const char *)) {
    char line[256];
    ram = mmap(NULL, RAM_BYTES, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    window = mmap(NULL, WINDOW_BYTES, PROT_NONE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (ram == MAP_FAILED || window == MAP_FAILED) {
        say("hostmmu: cannot reserve the RAM or the window");
        return -1;
    }
    for (uint64_t i = 0; i < RAM_BYTES; i += PAGE) ram[i] = (uint8_t)(i / PAGE); /* every page resident */
    uint32_t seed = 12345;
    for (uint32_t i = 0; i < N; i++) order[i] = i;
    for (uint32_t i = N - 1; i > 0; i--) {
        seed = seed * 1664525u + 1013904223u;
        uint32_t j = seed % (i + 1), t = order[i];
        order[i] = order[j], order[j] = t;
    }
    /* Window pages spread out, as a process's are: every 8th page of the first 1 GB. */
#define WPAGE(i) ((uint64_t)order[i] * 8)

    double t0 = now_ns();
    for (uint32_t i = 0; i < N; i++)
        if (map_page(WPAGE(i), order[i], 1) != 0) {
            say("hostmmu: mach_vm_remap into the window failed");
            return -1;
        }
    double t1 = now_ns();
    snprintf(line, sizeof line, "hostmmu: map a page (mach_vm_remap, fixed) %.0f ns", (t1 - t0) / N);
    say(line);

    /* First touch of each mapped page (the host's own fault for a mapping it has not entered yet), then again. */
    volatile uint64_t sum = 0;
    t0 = now_ns();
    for (uint32_t i = 0; i < N; i++) sum += window[WPAGE(i) * PAGE];
    t1 = now_ns();
    for (int k = 0; k < 16; k++)
        for (uint32_t i = 0; i < N; i++) sum += window[WPAGE(i) * PAGE];
    double t2 = now_ns();
    snprintf(line, sizeof line, "hostmmu: first touch of a mapped page %.0f ns, later touches %.1f ns (%u pages apart)", (t1 - t0) / N,
             (t2 - t1) / (16.0 * N), N);
    say(line);

    t0 = now_ns();
    for (uint32_t i = 0; i < N; i++) vm_protect(mach_task_self(), (vm_address_t)(uintptr_t)(window + WPAGE(i) * PAGE), PAGE, 0, VM_PROT_READ);
    t1 = now_ns();
    snprintf(line, sizeof line, "hostmmu: protect a page %.0f ns", (t1 - t0) / N);
    say(line);

    t0 = now_ns();
    for (uint32_t i = 0; i < N; i++) mmap(window + WPAGE(i) * PAGE, PAGE, PROT_NONE, MAP_PRIVATE | MAP_ANON | MAP_FIXED, -1, 0);
    t1 = now_ns();
    snprintf(line, sizeof line, "hostmmu: drop a page (mmap PROT_NONE over it) %.0f ns", (t1 - t0) / N);
    say(line);

    /* A fault filling the window: the signal, the remap, the return. */
    struct sigaction sa, old_segv, old_bus;
    memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = on_fault;
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, &old_segv);
    sigaction(SIGBUS, &sa, &old_bus);
    faults = 0;
    t0 = now_ns();
    for (uint32_t i = 0; i < N; i++) sum += window[WPAGE(i) * PAGE];
    t1 = now_ns();
    sigaction(SIGSEGV, &old_segv, NULL);
    sigaction(SIGBUS, &old_bus, NULL);
    snprintf(line, sizeof line, "hostmmu: a fault filled by a signal handler %.0f ns (%llu faults)", (t1 - t0) / N, (unsigned long long)faults);
    say(line);

    /* A whole-TLB flush: the window emptied with N pages in it. */
    t0 = now_ns();
    if (empty_window() != 0) say("hostmmu: emptying the window failed");
    t1 = now_ns();
    snprintf(line, sizeof line, "hostmmu: empty a %llu GB window holding %u pages %.0f us", WINDOW_BYTES >> 30, N, (t1 - t0) / 1000);
    say(line);

    /* How much address space there is for windows: 32 GB pieces until one is refused. */
    void *held[64];
    int nheld = 0;
    while (nheld < 64) {
        void *p = mmap(NULL, 32ull << 30, PROT_NONE, MAP_PRIVATE | MAP_ANON, -1, 0);
        if (p == MAP_FAILED) break;
        held[nheld++] = p;
    }
    snprintf(line, sizeof line, "hostmmu: %d windows of 32 GB can be reserved at once%s", nheld, nheld == 64 ? " (stopped there)" : "");
    say(line);
    for (int i = 0; i < nheld; i++) munmap(held[i], 32ull << 30);
    munmap(window, WINDOW_BYTES);
    munmap(ram, RAM_BYTES);
    return (int)(sum & 0);
}

/* The lines into one buffer, for callers without a C callback (the app's autopilot). */
static char *text_out;
static size_t text_cap;
static void say_text(const char *s) {
    size_t have = strlen(text_out);
    snprintf(text_out + have, text_cap - have, "%s\n", s);
}
int hv_proto_hostmmu_text(char *out, size_t cap) {
    if (!cap) return -1;
    out[0] = 0, text_out = out, text_cap = cap;
    return hv_proto_hostmmu(say_text);
}

#ifdef HOSTMMU_MAIN
static void say_stdout(const char *s) { puts(s); }
int main(void) { return hv_proto_hostmmu(say_stdout) != 0; }
#endif
