#!/usr/bin/env bash
# [EXPERIMENT ONLY — ADR-0001, ON HOLD] Produce a copy of the live
# optimizer_params.yaml with the F-16-derived vehicle physics applied.
# The output lands in .jsbsim/ (gitignored); the production yaml is NEVER
# modified. Feed the result to the chain harness explicitly:
#
#   ./apply_f16ref.sh
#   ./install/path_manager/lib/path_manager/chain_experiment_test \
#       experiments/jsbsim_probe/.jsbsim/optimizer_params_f16ref.yaml 3 route
#
# Provenance of the six values: docs/adr/0001-reference-vehicle.md
# (rejected first draft, kept as methodology record). This profile exists
# so the rejected experiment stays reproducible while the default config
# stays on the generic neutral baseline.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_YAML="$HERE/../../src/path_manager/config/optimizer_params.yaml"
OUT="$HERE/.jsbsim/optimizer_params_f16ref.yaml"
mkdir -p "$HERE/.jsbsim"
sed -e 's|dynamics_mass_kg: 1300.0|dynamics_mass_kg: 9357.6|' \
    -e 's|dynamics_wing_area_m2: 1.0|dynamics_wing_area_m2: 27.871|' \
    -e 's|dynamics_cd0: 0.035|dynamics_cd0: 0.017|' \
    -e 's|dynamics_induced_drag_factor: 0.080|dynamics_induced_drag_factor: 0.152|' \
    -e 's|dynamics_cl_max: 1.40|dynamics_cl_max: 1.829|' \
    -e 's|dynamics_thrust_max_n: 3200.0|dynamics_thrust_max_n: 61000.0|' \
    "$SRC_YAML" > "$OUT"
echo "wrote $OUT (production yaml untouched)"
