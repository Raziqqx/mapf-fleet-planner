# Convenience entry points; everything is plain CMake underneath.
#   make test          C++ tests under ASan + UBSan
#   make python        release build including the pybind11 module mapf_py
#   make pytest        Python smoke test (PYTHON must have pybind11 and pytest)
#   make reproduce     MovingAI download + run-A benchmark run serially + profile (~20 min)
#   make replay        docs/replay.svg through the Python bindings
#   make format-check  clang-format, as in CI
PYTHON ?= python3
FORMAT_FILES := $(shell find include src bench tests validator python -name '*.cpp' -o -name '*.hpp')

.PHONY: build test python pytest bench profile reproduce replay format format-check

build:
	cmake --preset release
	cmake --build --preset release

test:
	cmake --preset asan
	cmake --build --preset asan
	ctest --preset asan

python:
	cmake --preset release -DMAPF_BUILD_PYTHON=ON \
	    -DPython_EXECUTABLE=$$($(PYTHON) -c 'import sys; print(sys.executable)') \
	    -Dpybind11_DIR=$$($(PYTHON) -m pybind11 --cmakedir)
	cmake --build --preset release

pytest: python
	PYTHONPATH=build/release $(PYTHON) -m pytest -q python/tests

bench:
	JOBS=1 scripts/run_benchmark.sh

profile: build
	scripts/download_movingai.sh
	python3 -I scripts/profile.py --reps 5

reproduce: bench profile

replay: python
	PYTHONPATH=build/release $(PYTHON) python/examples/replay_svg.py --out docs/replay.svg

format:
	clang-format -i $(FORMAT_FILES)

format-check:
	clang-format --dry-run --Werror $(FORMAT_FILES)
