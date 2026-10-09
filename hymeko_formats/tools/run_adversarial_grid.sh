#!/usr/bin/env bash
# Run from an Amaterasu source checkout or the documented source snapshot.
# Usage: bash hymeko_formats/tools/run_adversarial_grid.sh /absolute/output
set -euo pipefail
if [[ "$(hostname)" != "Amaterasu" && "$(hostname)" != "amaterasu" ]]; then
  echo 'This campaign and Gazebo validation must run on Amaterasu.' >&2
  exit 2
fi
output=${1:?absolute output directory required}
if [[ "$output" != /* ]]; then
  echo 'Use an absolute output directory.' >&2
  exit 2
fi
log_dir="${output}.logs"
if [[ -e "$output" || -e "$log_dir" ]]; then
  echo 'Choose a fresh output path; previous results and logs are preserved.' >&2
  exit 2
fi
mkdir -p "$log_dir"
export CARGO_BUILD_JOBS=2
export PATH="$HOME/.cargo/bin:$PATH"
{
  date -Iseconds
  hostname
  uname -sr
  rustc --version
  cargo --version
  gz sim --versions
  lscpu
  free -h
} > "$log_dir/environment.txt"
cargo tree -d --locked --offline > "$log_dir/dependency-audit.txt"
cargo test -p hymeko_formats --lib --test gazebo_obstacles --bin adversarial_grid --locked --offline \
  > "$log_dir/tests.txt" 2>&1
/usr/bin/time -v cargo build -p hymeko_formats --bin adversarial_grid --release --locked --offline \
  > "$log_dir/build.txt" 2>&1
/usr/bin/time -v timeout 300s target/release/adversarial_grid --run-campaign \
  --output "$output" --seeds 10 --proposals 100 --width 16 --height 16 --wall-fraction 0.25 \
  > "$log_dir/campaign.txt" 2>&1
python3 hymeko_formats/tools/adversarial_grid_artifacts.py "$output" \
  > "$log_dir/figures.txt" 2>&1
python3 hymeko_formats/tools/validate_adversarial_grid_gazebo.py "$output" \
  > "$log_dir/gazebo-validation.txt" 2>&1
cp -r "$log_dir" "$output/logs"
printf 'Campaign complete: %s\n' "$output"
