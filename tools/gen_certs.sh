#!/usr/bin/env bash
# Generate the self-signed TLS server certificate and key the firmware embeds.
# EC P-256 (cheap handshake on the ESP32), valid 10 years, with SANs matching
# the mDNS hostname so client hostname verification can succeed.
#
# Run from the repo root:  ./tools/gen_certs.sh
# On Git Bash / MSYS, MSYS_NO_PATHCONV=1 stops the shell mangling the -subj.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p main/certs

MSYS_NO_PATHCONV=1 openssl req -x509 -newkey ec \
  -pkeyopt ec_paramgen_curve:prime256v1 \
  -keyout main/certs/prvtkey.pem -out main/certs/servercert.pem \
  -days 3650 -nodes -subj "/CN=aurora-gateway" \
  -addext "subjectAltName=DNS:aurora-gateway,DNS:aurora-gateway.local"

echo "wrote main/certs/servercert.pem and main/certs/prvtkey.pem (valid 10 years)"
echo "prvtkey.pem is gitignored - keep it private."
