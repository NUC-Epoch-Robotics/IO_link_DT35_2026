#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
"${CC:-cc}" -std=c99 -Wall -Wextra -Werror -IAPP/sensor_dt35 tests/dt35_protocol_test.c APP/sensor_dt35/dt35_protocol.c -o "$build/dt35_protocol"
"$build/dt35_protocol"
