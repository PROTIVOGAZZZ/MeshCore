---
name: run-meshcore
description: Build MeshCore firmware (nRF52 targets like LilyGo_T-Echo, RAK_4631, t1000e, Heltec_t114), produce a flashable UF2, inspect RAM/heap usage, and run the native googletest unit tests - offline, without the PlatformIO registry. Use when asked to build, compile, test, check memory/heap of, or make a test firmware for MeshCore.
---

MeshCore is embedded firmware: there is no emulator here, so "running" it means
(a) building a target and reading its memory map, and (b) running the host-side
googletest suites. Both go through `.claude/skills/run-meshcore/driver.py`, which
works even when `api/dl.registry.platformio.org` is blocked (as in Claude Code cloud
containers): it assembles the toolchain, platforms and libraries from apt + GitHub,
copies the source tree to a work dir, rewrites registry `lib_deps` to git URLs there,
and runs `pio`. The repo itself is never modified.

All paths are relative to the repo root.

## Prerequisites

Python 3, git, and apt access (the driver installs `gcc-arm-none-eabi`,
`libnewlib-arm-none-eabi` and `platformio` itself if missing).

## Setup (once per container, ~2 min)

```bash
python3 .claude/skills/run-meshcore/driver.py setup
```

Everything lands in `$MESHCORE_OFFLINE` (default `~/.cache/meshcore-offline`) and
`~/.platformio` (or `$PLATFORMIO_CORE_DIR`).

## Build a firmware target (agent path)

```bash
python3 .claude/skills/run-meshcore/driver.py build LilyGo_T-Echo_companion_radio_ble
```

First build of an env ~2-3 min (clones libs), rebuilds ~1 min. Output:

- `RAM:` / `Flash:` lines from PlatformIO
- `UF2: ~/.cache/meshcore-offline/out/<env>.uf2` - flashable file (`<env>-<hash>.uf2` for `--src` builds)
- heap size left after static RAM (`__HeapLimit - __HeapBase`) and the largest RAM symbols
- full log: `~/.cache/meshcore-offline/work/<env>/build.log`

Verified envs: `LilyGo_T-Echo_companion_radio_ble`, `RAK_4631_companion_radio_ble`,
`t1000e_companion_radio_ble`, `Heltec_t114_repeater`. Env names: `pio project config | grep env:`.

Build someone else's tree (a fork checkout, a patched copy) without touching this repo:

```bash
python3 .claude/skills/run-meshcore/driver.py build LilyGo_T-Echo_companion_radio_ble --src /path/to/other/checkout
```

Re-print the memory report of the last build of an env (add the same `--src` for other trees):

```bash
python3 .claude/skills/run-meshcore/driver.py mem LilyGo_T-Echo_companion_radio_ble
python3 .claude/skills/run-meshcore/driver.py mem LilyGo_T-Echo_companion_radio_ble --src /path/to/other/checkout
```

To compare two trees (e.g. a regression vs upstream), build both with `--src` and
diff the `mem` output; runtime `new` sizes can be read from the ELF with
`arm-none-eabi-objdump -d -C ~/.cache/meshcore-offline/work/<env>/.pio/build/<env>/firmware.elf | grep -B3 "operator new"`.

## Unit tests (direct invocation)

Same envs CI runs (`native` + `native_kiss_modem`):

```bash
python3 .claude/skills/run-meshcore/driver.py test
python3 .claude/skills/run-meshcore/driver.py test --env native -f test_utils
```

Exit code is PlatformIO's (non-zero on any failure). Expected today: 48/48 pass,
`test_companion_node_prefs` SKIPPED (emptied upstream in e78bff00).

## Run on hardware (human path)

Only a person with the board can do this: double-tap reset -> a USB drive appears
-> copy the `.uf2` onto it. UF2s from this driver (Ubuntu GCC 13) were flashed on a
real T-Echo and connected to the companion apps fine.

## Gotchas

- **Toolchain differs from CI.** Ubuntu's arm-none-eabi GCC 13.2 stands in for PIO's
  7.2.1 (registered under a fake version `1.70201.0`). Sizes differ slightly from
  official builds; behaviour on hardware has matched so far.
- **Libs are git HEADs**, not the registry versions, unless `LIB_MAP` in `driver.py`
  pins a tag. New registry `owner/Name @ ver` entries in any `platformio.ini` need a
  `LIB_MAP` line - the build prints the unmapped names when it fails.
- **`*** [...firmware.zip] Error 2` is expected**: `tool-adafruit-nrfutil` is a stub.
  The ELF/HEX and the UF2 are fine.
- **Only nRF52 + native envs.** ESP32/RP2040/STM32 platforms come from the registry;
  the driver says `platform ... is registry-only` and stops.
- `base64_arduino` makes PIO fetch `throwtheswitch/Unity` from the registry; the driver
  stubs it (the message `stubbing unreachable transitive lib` is normal).
- Work dirs are reused (`work/<env>`, or `work/<env>-<hash>` for `--src`); files deleted
  from the source tree are not deleted from the copy. `rm -rf` the work dir if that matters.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `HTTPClientError` while `Installing <owner>/<lib> @ ...` | add `"<owner>/<lib>": "https://github.com/...git#tag"` to `LIB_MAP` (or `PRELOAD` for a transitive dep), rebuild |
| `VCSBaseException ... Remote branch X not found` | the `#tag` in `LIB_MAP` doesn't exist on GitHub; check `git ls-remote --tags <url>` |
| `missing SConscript file .../arduino/adafruit.py` | platform clone lacks its submodule: `rm -rf ~/.cache/meshcore-offline/platform-nordicnrf52` and rerun `setup` |
| `fatal error: core_cm4.h` | CMSIS not copied: rerun `setup` |
| `libarm_cortexM4lf_math.a:1: syntax error` | git-LFS pointer instead of the lib: delete it and rerun `setup` (it writes an empty archive) |
| tests `ERRORED` after ~60 s each, `Installing google/googletest` | googletest not pre-seeded: rerun `setup`, then `test` |
