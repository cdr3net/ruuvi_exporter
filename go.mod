module github.com/milaboratory/ruuvi_exporter

go 1.14

require (
	github.com/go-ble/ble v0.0.0-20200407180624-067514cd6e24
	github.com/prometheus/client_golang v1.6.0
)

replace github.com/go-ble/ble => github.com/milaboratory/ble v0.0.0-20200531125715-a38877ffd926
