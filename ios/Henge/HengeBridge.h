#include "Native/henge-ios-jit.h"
#include "Native/henge-vm.h"
#include "Native/henge-tl.h"
#include "Native/HuskRPPairing.h"
#include "hvsparse.h"
/* host/proto_hostmmu.c: the host MMU timings, one line each, into out. */
#include <stddef.h>
int hv_proto_hostmmu_text(char *out, size_t cap);
