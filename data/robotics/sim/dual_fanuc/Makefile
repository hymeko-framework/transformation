# Dual-FANUC Gazebo demo — build / validate / run.
#
# One HymeKo cognitive description (fanuc_lrmate200id.hymeko) → one
# SDF model → two arm instances in a Gazebo world.

REPO_ROOT      ?= $(shell git rev-parse --show-toplevel 2>/dev/null || echo "../../..")
HYMEKO         ?= $(REPO_ROOT)/target/release/hymeko
TRANSFORMS_DIR ?= $(REPO_ROOT)/transforms
SRC             = fanuc_lrmate200id.hymeko
MODEL           = model.sdf
WORLD           = world.sdf

# Default: rebuild the model SDF from the HymeKo source.
.PHONY: all
all: $(MODEL)

$(MODEL): $(SRC)
	$(HYMEKO) emit -f sdf -n fanuc_lrmate200id --rich \
	    --transforms-dir $(TRANSFORMS_DIR) \
	    -o $(MODEL) $(SRC)

# Validate parse + topology (warnings allowed — the `world` parent
# warning is expected by the meta_kinematics pattern).
.PHONY: validate
validate:
	$(HYMEKO) validate $(SRC)

# Validate the emitted SDF against sdformat (modern Gazebo).
# Requires `gz` (or `ign`) on PATH.
.PHONY: check-sdf
check-sdf: $(MODEL)
	gz sdf --check $(MODEL)
	gz sdf --check $(WORLD)

# Run the simulation headless for one second to confirm the world loads.
# Useful as a CI smoke. -s = run server-only, -r = run on launch, --iterations
# limits the run length.
.PHONY: smoke
smoke: $(MODEL)
	@echo "Running Gazebo headless smoke (1000 iterations)..."
	GZ_SIM_RESOURCE_PATH="$$PWD/..:$$GZ_SIM_RESOURCE_PATH" \
		gz sim -s -r --iterations 1000 $(WORLD)

# Interactive run with GUI (default user workflow).
.PHONY: run
run: $(MODEL)
	GZ_SIM_RESOURCE_PATH="$$PWD/..:$$GZ_SIM_RESOURCE_PATH" \
		gz sim -v 4 -r $(WORLD)

.PHONY: clean
clean:
	rm -f $(MODEL)

# Self-documentation.
.PHONY: help
help:
	@echo "Targets:"
	@echo "  all        rebuild model.sdf from fanuc_lrmate200id.hymeko"
	@echo "  validate   parse + topology check on the .hymeko"
	@echo "  check-sdf  sdformat validation on the emitted SDF"
	@echo "  smoke      headless 1000-iteration GZ smoke"
	@echo "  run        interactive GZ session with GUI"
	@echo "  clean      delete the emitted model.sdf"
