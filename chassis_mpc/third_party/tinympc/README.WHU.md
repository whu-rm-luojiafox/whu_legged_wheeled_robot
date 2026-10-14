# TinyMPC import and runtime adaptation

Upstream: https://github.com/TinyMPC/TinyMPC

Pinned commit: `023f36bd5b27267e1a96ed47a774d120b5707f13`.
Upstream CMake project version: 0.2.0 (commit pin, not a release tag).

`src/`, `include/` and `LICENSE` are copied verbatim from this commit.
`UPSTREAM.json` records their SHA-256 hashes. Eigen is supplied by upstream;
its header notices are preserved and the MPL 2.0 license is in `LICENSES/`.

## What is compiled

The experimental target compiles `port/admm.cpp` plus the project's generated
data and C wrapper. It does NOT compile the desktop code generator,
`tiny_api.cpp`, adaptive-rho benchmark code or upstream examples on the MCU.

`port/` is generated from the pristine upstream by
`tools/chassis/mpc/generate_solver.py`. Its exact changes are in
`port/runtime.patch`:

- Optional `WHU_TINY_FLOAT` scalar type; otherwise original double.
- `WHU_TINY_FIXED_RHO` excludes adaptive-rho setup/branches and their temporary
  allocations. This experiment uses constant rho=1.
- `WHU_TINY_QUIET` suppresses the per-solve console message.
- Tracking linear costs use the unaugmented Q/R (workspace diagonals include
  rho). Terminal tracking uses Pinf-rho*I, matching the effective terminal
  Hessian after the ADMM proximal term is removed. These cost-consistency
  adaptations are checked against an independently condensed QP, including
  a nonzero reference. The runtime is an ADAPTED official solver, not an
  assertion that pristine upstream setup/codegen produces identical results.

## Why a project-specific data exporter is used

At the pinned commit, `codegen.cpp` emits positional TinyCache/TinyWorkspace
initializers that do not match `types.hpp` (for example APf/BPf fields).
Additionally, upstream `tiny_setup` augments Q/R before passing them to a
precompute routine that augments them again. The experimental generator
avoids both paths: it uses an offline, exact SciPy augmented DARE and emits
named runtime-field assignments. No ADMM solver has been rewritten.

The terminal weight used in the independent comparison is Pinf-rho*I; it is
NOT the unaugmented Pd exported by the model tool. This difference is recorded
in the generated manifest and numerical report.

## Embedded limits

Eigen dynamic allocation remains during initialization and potentially inside
expression evaluation during solve. A float build uses the M4 single-precision
FPU, but it is not a no-heap/fixed-workspace implementation. Compiler/archive
success does not verify startup, final linking, heap/stack capacity or timing.
ARMCC5 is rejected by bundled Eigen; independent ARMClang6.7 compilation works
with `-std=c++1z`. The legacy Keil project has been removed. Root CMake provides
ARMClang6 builds for the complete LQR firmware and a separate MPC probe image;
see the root README for commands and board-validation limits.
