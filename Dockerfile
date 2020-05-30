FROM golang:1.14-alpine as builder

RUN mkdir -p /build

WORKDIR /build

COPY go.mod /build/
COPY go.sum /build/

RUN go mod download

COPY cmd /build/cmd

RUN CGO_ENABLED=0 go build -o ruuvi_exporter cmd/main.go

FROM alpine:3.11

#RUN apk --update --no-cache add ca-certificates && \
#    mkdir /ngs && \
#    mkdir /state

COPY --from=builder /build/ruuvi_exporter /opt/ruuvi_exporter

EXPOSE 10000

ENTRYPOINT "/opt/ruuvi_exporter"
