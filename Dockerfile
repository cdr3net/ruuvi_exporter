FROM golang:1.14-alpine as builder

RUN mkdir -p /build

WORKDIR /build

COPY go.mod /build/
COPY go.sum /build/

RUN go mod download

COPY cmd /build/cmd

RUN CGO_ENABLED=0 go build -o ruuvi_exporter cmd/main.go

FROM debian

RUN apt-get update && apt-get install -y \
  bluez \
  lsof \
  htop
#  bluez-tools \
#  libbluetooth-dev

COPY --from=builder /build/ruuvi_exporter /opt/ruuvi_exporter
COPY entrypoint.sh /entrypoint/entrypoint.sh
RUN ["chmod", "+x", "/entrypoint/entrypoint.sh"]

EXPOSE 10000

ENTRYPOINT "/entrypoint/entrypoint.sh"
