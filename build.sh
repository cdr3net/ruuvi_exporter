#!/usr/bin/env bash

mkdir -p build

GOOS=linux GOARM=6 GOARCH=arm go build -o build/ruuvi_exporter_server_linux_armv6 cmd/main.go
GOOS=linux GOARCH=amd64 go build -o build/ruuvi_exporter_server_linux_amd64 cmd/main.go

