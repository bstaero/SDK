# Common Example Code

Shared utility code and build rules used by all SDK examples.

## Files

| File | Purpose |
|------|---------|
| `example_common.h` | Shared declarations: `CommType_t` enum, time/terminal/CLI functions |
| `example_common.cpp` | Implementations: `setupTime()`, `getElapsedTime()`, `inputAvailable()`, `initTerminal()`, `restoreTerminal()`, `printBaseHelp()`, `detectEndianness()` |
| `Makefile.common` | Shared build rules with `EXAMPLE_TYPE` support (`bst`, `can`, `raw`) |

## Usage

Each example Makefile includes this shared Makefile:

```makefile
EXAMPLE_TYPE = bst
include ../common/Makefile.common
```

See `Makefile.common` header comments for all available configuration variables.
