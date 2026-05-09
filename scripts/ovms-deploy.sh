#!/usr/bin/env bash
# Deploy a freshly built ovms3.bin to a running OVMS module via SCP + `ota flash vfs`.
#
# Usage:
#   scripts/ovms-deploy.sh [host] [user]
#     host    OVMS module IP/hostname  (default: $OVMS_HOST or 10.10.10.115)
#     user    SSH username             (default: $OVMS_USER or module)
#
# Env (also auto-loaded from scripts/.env if present; see scripts/.env.example):
#   OVMS_HOST       module IP/hostname (overrides arg default)
#   OVMS_USER       SSH username       (overrides arg default)
#   OVMS_FIRMWARE   override path to ovms3.bin
#   OVMS_REBOOT=1   issue `module reset` after flashing (default: off)
#   SSHPASS         password (used non-interactively if `sshpass` is installed)

set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Auto-load local defaults (SSHPASS, OVMS_HOST, OVMS_USER, ...) if present.
# This file is gitignored — see .gitignore.
if [[ -f "$SCRIPT_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$SCRIPT_DIR/.env"
  set +a
fi

HOST=${1:-${OVMS_HOST:-10.10.10.115}}
USER=${2:-${OVMS_USER:-module}}

FIRMWARE=${OVMS_FIRMWARE:-$REPO_ROOT/vehicle/OVMS.V3/build/ovms3.bin}

if [[ ! -f "$FIRMWARE" ]]; then
  echo "error: firmware not found at $FIRMWARE" >&2
  echo "       build it first: (cd vehicle/OVMS.V3 && make -j\$(nproc) all)" >&2
  exit 1
fi

# OVMS regenerates SSH host keys at first boot, so don't pollute ~/.ssh/known_hosts.
# wolfSSH on the module only offers an ssh-rsa (SHA1) host key, which OpenSSH 8.8+
# refuses by default — re-enable it explicitly for both host-key and pubkey auth.
SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o LogLevel=ERROR
  -o HostKeyAlgorithms=+ssh-rsa
  -o PubkeyAcceptedAlgorithms=+ssh-rsa
)

# OpenSSH 9.0+ scp defaults to SFTP, but wolfSSH only implements legacy SCP.
SCP_OPTS=("${SSH_OPTS[@]}" -O)

# If sshpass + SSHPASS are available, run non-interactively.
SSH_CMD=(ssh "${SSH_OPTS[@]}")
SCP_CMD=(scp "${SCP_OPTS[@]}")
if [[ -n "${SSHPASS:-}" ]] && command -v sshpass >/dev/null 2>&1; then
  SSH_CMD=(sshpass -e ssh "${SSH_OPTS[@]}")
  SCP_CMD=(sshpass -e scp "${SCP_OPTS[@]}")
fi

SIZE=$(stat -c %s "$FIRMWARE" 2>/dev/null || stat -f %z "$FIRMWARE")
echo ">> firmware: $FIRMWARE ($SIZE bytes)"
echo ">> target:   $USER@$HOST"

echo ">> SCP /sd/ovms3.bin ..."
"${SCP_CMD[@]}" "$FIRMWARE" "$USER@$HOST:/sd/ovms3.bin"

echo ">> ota flash vfs /sd/ovms3.bin ..."
"${SSH_CMD[@]}" "$USER@$HOST" "ota flash vfs /sd/ovms3.bin"

if [[ "${OVMS_REBOOT:-0}" == "1" ]]; then
  echo ">> module reset ..."
  "${SSH_CMD[@]}" "$USER@$HOST" "module reset" || true
  echo ">> done. Module is rebooting into the new image."
else
  echo ">> done. Run 'module reset' on the module (or set OVMS_REBOOT=1) to boot the new image."
fi
