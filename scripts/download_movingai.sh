#!/usr/bin/env bash
# Downloads a few MovingAI MAPF benchmark maps and their "random" scenario
# files into data/maps and data/scen. Source and licence: data/README.md.
# Idempotent: files that already exist are skipped.
set -euo pipefail

BASE_URL="https://movingai.com/benchmarks/mapf"
# MAPF_MAPS="room-32-32-4" limits the download (CI fetches one small map).
read -r -a MAPS <<< "${MAPF_MAPS:-empty-32-32 random-32-32-20 room-32-32-4 maze-32-32-2 warehouse-10-20-10-2-1}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE="$ROOT/data/.cache"
mkdir -p "$ROOT/data/maps" "$ROOT/data/scen" "$CACHE"

# Extracts a zip with Python's zipfile (unzip is not always installed).
# -I keeps the interpreter from importing anything from the working directory.
extract() { python3 -I -m zipfile -e "$1" "$2"; }

for map in "${MAPS[@]}"; do
    if [[ ! -f "$ROOT/data/maps/$map.map" ]]; then
        curl -fsSL "$BASE_URL/$map.map.zip" -o "$CACHE/$map.map.zip"
        extract "$CACHE/$map.map.zip" "$ROOT/data/maps"
    fi
    if [[ ! -f "$ROOT/data/scen/$map-random-1.scen" ]]; then
        curl -fsSL "$BASE_URL/$map.map-scen-random.zip" -o "$CACHE/$map.scen.zip"
        tmp="$CACHE/$map-scen"
        mkdir -p "$tmp"
        extract "$CACHE/$map.scen.zip" "$tmp"
        # The archive nests the files in a folder; flatten them into data/scen.
        find "$tmp" -name '*.scen' -exec mv {} "$ROOT/data/scen/" \;
        rm -rf "$tmp"
    fi
done
echo "MovingAI data ready: $(ls "$ROOT/data/maps" | wc -l) maps, $(ls "$ROOT/data/scen" | wc -l) scen files"
