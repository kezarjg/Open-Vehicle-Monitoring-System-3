#!/usr/bin/env bash
#
# Verify the committed OVMS web-asset bundles (.js/.css) and their .gz files are
# in sync with their sources. Mirrors the cat/gzip recipes in
#   vehicle/OVMS.V3/components/ovms_webserver/component.mk
#
# Why this exists:
#   script.js / charts.js / tables.js / style.css and their *.gz are committed
#   *generated* artifacts. The firmware embeds the *.gz directly
#   (COMPONENT_EMBED_FILES). On a fresh checkout every file has the same mtime,
#   so `make` does NOT regenerate them — it embeds whatever is committed. If a
#   source (e.g. ovms.js) is edited without regenerating the bundle/.gz, the
#   firmware ships a STALE asset. This guard fails when that happens.
#
# Note: gzip output is not byte-reproducible (the gzip header embeds a
#   timestamp), so we compare *decompressed* content, never the raw .gz bytes.
#
# Usage: run from anywhere inside the repo:  bash check-web-assets.sh
set -uo pipefail

cd "$(git rev-parse --show-toplevel)/vehicle/OVMS.V3/components/ovms_webserver/assets"

if ! command -v dos2unix >/dev/null 2>&1; then
  echo "::error::dos2unix not found (apt-get install dos2unix)"; exit 2
fi

status=0

# check_bundle <bundle> <source>...   (source list mirrors component.mk)
check_bundle() {
  local bundle="$1"; shift
  local exp; exp="$(mktemp)"
  cat "$@" | dos2unix > "$exp"
  if ! diff -q <(dos2unix < "$bundle") "$exp" >/dev/null; then
    echo "::error::$bundle is STALE — does not match cat of its sources."
    echo "         regenerate: cat $* | dos2unix > $bundle"
    status=1
  fi
  if ! diff -q <(gzip -dc "$bundle.gz" | dos2unix) "$exp" >/dev/null; then
    echo "::error::$bundle.gz is STALE — its decompressed content does not match the sources."
    echo "         regenerate: gzip -c $bundle > $bundle.gz"
    status=1
  fi
  rm -f "$exp"
}

check_bundle script.js  jquery.min.js bootstrap.min.js cbor.js ovms.js
check_bundle charts.js  highcharts.js highcharts-more.js \
                        hc-modules/bullet.js hc-modules/solid-gauge.js \
                        hc-modules/streamgraph.js hc-modules/xrange.js
check_bundle tables.js  datatables.min.js
check_bundle style.css  intro.css bootstrap.min.css bootstrap-theme.min.css \
                        highcharts.css datatables.min.css datatables.ovms.css ovms.css

# single-source gz (no concatenated bundle):
if ! diff -q <(gzip -dc zones.json.gz) zones.json >/dev/null; then
  echo "::error::zones.json.gz is STALE — does not match zones.json."
  echo "         regenerate: gzip -c zones.json > zones.json.gz"
  status=1
fi

if [ "$status" -eq 0 ]; then
  echo "OK: all web-asset bundles and .gz files are in sync with their sources."
fi
exit "$status"
