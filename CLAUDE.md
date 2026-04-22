# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A C++17 framework that computes optimal move counts for Rubik's Cube Cross / F2L sub-goals (Cross, XCross ... XXXXCross, plus Pair, Pseudo, and EO variants). Each analyzer is a standalone executable sharing a common IDA* search engine and large precomputed move/prune tables.

## Build & run

- `.\build.ps1` — parallel build of every target (wraps `mingw32-make -j8`, supports incremental rebuild).
- `.\build.ps1 <target>.exe` or `mingw32-make <target>.exe` — single target.
- `.\clean.ps1` — remove `*.o` and `*.exe`.
- Compiler flags (see `Makefile`): `g++ -std=c++17 -O3 -mavx2 -fopenmp -Wall -Wextra`. Toolchain is MinGW-w64 (`C:/msys64/mingw64`) — `.clangd` is configured to use its headers.
- Tables live in `tables/`. They are **not** in git (`*.bin` in `.gitignore`). Largest are `mt_edge6.bin` (~3 GB) and `pt_cross_C4C5E0E1.bin` / `pt_cross_C4C6E0E2.bin` (~10 GB each). Regenerate via `table_generator.exe` if missing — first-time generation is slow; thereafter analyzers only load the subset they need (may still be "<1 min" per run).

### Running an analyzer

Each analyzer reads the scramble filename from **stdin**, then writes `<input_stem>_<suffix>.csv`:

```powershell
"scramble_1000.txt" | .\std_analyzer.exe        # → scramble_1000_std.csv
"scramble_1000.txt" | .\pair_analyzer.exe       # → scramble_1000_pair.csv
"scramble_1000.txt" | .\pseudo_analyzer.exe     # → scramble_1000_pseudo.csv
"scramble_100.txt"  | .\pseudo_pair_analyzer.exe
"scramble_20.txt"   | .\eo_cross_analyzer.exe   # slowest — keep input small
```

Output columns are per-rotation: 6 axial views `z0,z1,z2,z3,x1,x3` (U/D/L/R/F/B bottom), each further split into the cascade of sub-goals for that analyzer.

## Tests

- `.\verify.ps1` — runs all five analyzers and compares the first 21 lines of each output CSV against `golden/*.txt`. Exits non-zero on any FAIL. This is the authoritative regression suite; use it after any change that could affect search correctness.
- There is no "single test" concept — correctness is whole-suite comparison against golden CSVs. To narrow scope while iterating, shrink the input file (e.g. use `scramble_5.txt` / `scramble_20.txt` / `scramble_100.txt`) and diff against a locally-saved known-good CSV.

## Architecture

Four layers (see `ARCHITECTURE.md` for the full picture — read it before making structural changes):

1. **Base** — `cube_common.{h,cpp}`: `State`, the 18-move set, index↔permutation/orientation converters, the move-table engine (`createMultiMoveTable*`), rotation/conjugation maps (`rot_map`, `conj_moves_flat`, `sym_moves_flat`), file I/O templates, `StateSpace` dimension constants.
2. **Data** — `move_tables.{h,cpp}` (`MoveTableManager`) and `prune_tables.{h,cpp}` + `prune_create.cpp` (`PruneTableManager`). Both are singletons; analyzers call `ensure*()` / `release*()` to load only what they need. Prune tables are 4-bit-packed, filled by BFS.
3. **Executor** — `analyzer_executor.h`: the `run_analyzer_app<SolverWrapper>()` template that every `main()` calls. Handles stdin prompt, scramble parsing, OpenMP-parallel solve across scrambles, ANSI progress bar, CSV emission, summary stats. Node-count stats use the `COUNT_NODE` macro with a thread-local accumulator.
4. **Application** — one `.cpp` per analyzer (`std_analyzer`, `pair_analyzer`, `pseudo_analyzer`, `pseudo_pair_analyzer`, `eo_cross_analyzer`) plus `table_generator.cpp`. Each defines a `Solver` (holds MT/PT pointers, has `search_N` per cascade stage) and a `SolverWrapper` (adapts to the executor: `global_init()`, `get_csv_header()`, `solve(alg, id)`). `cross_solver.h` is a shared Cross search shared by Std/Pseudo (switched by `isPseudo`); EO has its own because of dimension differences.

### Conjugation — the central optimization

Rather than storing a separate prune table per slot/rotation, only one "canon" table per shape is loaded; other views are queried by conjugating the move index through `conj_moves_flat[m][slot_k]`. `table_naming.csv` labels every table as **Canon** (loaded), **Conj** (runtime-mapped, not loaded), or **Zombie** (generated for validation only). When adding a new prune table, register it in `table_naming.csv` and decide its role — don't load a Conj table.

### Color neutrality

All analyzers emit 6 outputs per scramble (`z0..z3, x1, x3`), one per cube orientation, to simulate color neutrality. `eo_cross_analyzer` goes further: it searches 12 symmetric views and outputs `min` of each pair — the "pair-internal best sharing" optimization in `TODO.md` is specific to that pattern.

## Reference docs in-repo

- `ARCHITECTURE.md` — layer diagram, solver pattern, conjugation explanation. Authoritative.
- `README.md` — mathematical definitions (corner/edge IDs, slot numbering, each analyzer's goal, Pseudo vs. Pair semantics), plus golden output samples.
- `table_naming.csv` — every `.bin` file: filename, variable, generator, which analyzers reference it, Canon/Conj/Zombie role, size. Check this before adding or renaming a table.
- `create_functions.csv` — documents every `create*` engine in `prune_create.cpp` (state space, dimension formula, MT dependencies, callers).
- `TODO.md` — log of attempted optimizations (validated ✅, rejected ❌, open). Read before proposing perf work; several "obvious" ideas are listed as already tried or deliberately skipped with reasons.

## Conventions

- **Abbreviations used everywhere**: `sz`=size, `ed`=edge, `cn`/`corn`=corner, `cr`=cross, `ps`=pseudo, `ins`=insertion, `ex`=extra, `mt`=move table, `pt`=prune table, `adj`=adjacent, `diag`=diagonal. Follow these when naming new symbols.
- **File encoding**: UTF-8 **without BOM**, LF line endings (see commit `5b7acea`). The repo has `.gitattributes` enforcing this; don't re-introduce CRLF or BOM when editing.
- **Build artifacts are git-ignored** (`*.o`, `*.exe`, `*.bin`, `*.csv` except `table_naming.csv`). Don't commit them.

## Platform note

Scripts (`build.ps1`, `clean.ps1`, `verify.ps1`, `sync.ps1`) are PowerShell 7 (pwsh). Prefer `.\script.ps1` over invoking make directly unless you need a non-default target.
