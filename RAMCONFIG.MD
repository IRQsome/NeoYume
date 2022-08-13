NeoYume external memory configuration
-------------------------------------

Configuring the external memory settings is a bit tricky, so here is a detailed explanation of what each setting does and known-good values for particular hardware configurations.

## Note on HyperRAM support

HyperRAM support is currently semi-experimental. The default values in config.spin2 should be fine for the Parallax HyperBus board on P2EVAL basepin 0.
Most of the configuration values are separate but follow the same logic as the PSRAM ones.

## Known-good configs

### P2EDGE 32MB

Since this is the most common config, these are the default values in config.spin2.

```spin
' Enable one of these to select the exmem type to use
#define USE_PSRAM16
'#define USE_PSRAM8
'#define USE_PSRAM4
'#define USE_HYPER

PSRAM_CLK = 56
PSRAM_SELECT = 57
PSRAM_BASE = 40
PSRAM_BANKS = 1
' \/ Uncomment if PSRAM_BANKS = 1 for speedup
#define USE_PSRAM_NOBANKS

PSRAM_WAIT  = 5
PSRAM_DELAY = 14
PSRAM_SYNC_CLOCK = true
PSRAM_SYNC_DATA = true

' \/ Uncomment for lameness
'#define USE_PSRAM_SLOW
```

### Rayslogic 96MB PSRAM board (on P2EVAL, basepin 0)

```spin
' Enable one of these to select the exmem type to use
'#define USE_PSRAM16
#define USE_PSRAM8
'#define USE_PSRAM4
'#define USE_HYPER

' For PSRAM (either type)
PSRAM_CLK = 8 addpins 1
PSRAM_SELECT = 10
PSRAM_BASE = 0
PSRAM_BANKS = 6
' \/ Uncomment if PSRAM_BANKS = 1 for speedup
'#define USE_PSRAM_NOBANKS

PSRAM_WAIT  = 5
PSRAM_DELAY = 17
PSRAM_SYNC_CLOCK = true
PSRAM_SYNC_DATA = true

' \/ Uncomment for lameness
#define USE_PSRAM_SLOW
```

## Configurable Values

### USE_PSRAM4 / USE_PSRAM8 / USE_PSRAM16 / USE_HYPER

One of these defines must be enabled to select the RAM type and bus width used (and thus the number of RAM chips in parallel).
 USE_PSRAM4 -> single PSRAM chip
 USE_PSRAM8 -> dual PSRAM chips
 USE_PSRAM16 -> quad PSRAM chips
 USE_HYPER -> single HyperRAM chip

### PSRAM_CLK

PSRAM clock output pin. This can be any pin (and if needed, 3 additional clock pins may be added using addpins syntax).

### PSRAM_SELECT

PSRAM select pin. This can be any pin.

### PSRAM_BANKS

Number of PSRAM banks to use. The second bank will use PSRAM_SELECT **+ 1** as its select pin (and so on for the third, etc).
If this is one, you can enable USE_PSRAM_NOBANKS to further improve memory throughput by eliminating all banking code.

### PSRAM_BASE

PSRAM data bus base pin. This must be a multiple of 8 (or 4 if USE_PSRAM4) and the data pins must be connected to this in order.

### PSRAM_WAIT

Number of data bit periods between address send and data return. **Leave this value at 5 unless you have a different type of memory chip.**

### PSRAM_DELAY

Read delay compensation in P2 machine cycles. Try changing this value if you have problems.

### PSRAM_SYNC_DATA

Enables `P_SYNC_IO` mode on the data bus. This is a sort-of "half step backwards" for PSRAM_DELAY and should be adjusted alongside it.

### PSRAM_SYNC_CLOCK

Enables `P_SYNC_IO` mode on the clock pin. This is a further timing adjustment. In most cases you can leave it on true.

### USE_PSRAM_SLOW

Enables a slower bus speed (CPU clock divided by 3 instead of 2). This is useful for setups with a large number of banks.

