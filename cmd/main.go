package main

import (
	"context"
	"encoding/binary"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/go-ble/ble"
	"github.com/go-ble/ble/examples/lib/dev"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

const RUUVI_MANUFACTURER_ID uint16 = 0x0499
const RUUVI_PACKET_VERSION uint8 = 5

type RuuviPacket struct {
	Timestamp time.Time
	Address   ble.Addr
	RSSI      int

	Temperature        int16
	Humidity           uint16
	Pressure           uint16
	AccelerationX      int16
	AccelerationY      int16
	AccelerationZ      int16
	PowerInfoVoltage   uint16
	PowerInfoTX        uint8
	MovementCounter    uint8
	MeasurementCounter uint16
}

func (p RuuviPacket) GetTemperatureInCelsius() float64 {
	return 0.005 * float64(p.Temperature)
}

func (p RuuviPacket) GetHumidity() float64 {
	return 0.0025 * float64(p.Humidity)
}

func (p RuuviPacket) GetPressure() float64 {
	return float64(p.Pressure) + 50000
}

func (p RuuviPacket) GetAccelerationX() float64 {
	return float64(p.AccelerationX) / 1000
}

func (p RuuviPacket) GetAccelerationY() float64 {
	return float64(p.AccelerationY) / 1000
}

func (p RuuviPacket) GetAccelerationZ() float64 {
	return float64(p.AccelerationZ) / 1000
}

func (p RuuviPacket) GetPowerInfoVoltage() float64 {
	return 1.6 + float64(p.PowerInfoVoltage)/1000
}

func (p RuuviPacket) GetPowerInfoTX() float64 {
	return -40 + float64(p.PowerInfoTX)*2
}

func (p RuuviPacket) GetMovementCounter() int32 {
	return int32(p.MovementCounter)
}

func (p RuuviPacket) GetMeasurementCounter() int32 {
	return int32(p.MeasurementCounter)
}

func (p RuuviPacket) String() string {
	return fmt.Sprintf(p.Address.String()+
		" Temp = %.2f C° RSSI = %d Humnidity = %.2f %% Pressure = %.2f Pa AccelerationXYZ = (%.2f, %.2f, %.2f) Voltage = %.2f V "+
		"TXPower = %.2f dBm MovementCounter = %d MeasurmentCounter = %d",
		p.GetTemperatureInCelsius(), p.RSSI, p.GetHumidity(), p.GetPressure(), p.GetAccelerationX(), p.GetAccelerationY(), p.GetAccelerationZ(),
		p.GetPowerInfoVoltage(), p.GetPowerInfoTX(), p.GetMovementCounter(), p.GetMeasurementCounter())
}

func TryParseRuuviPacketFromManufacturerData(a ble.Advertisement) *RuuviPacket {
	packet := a.ManufacturerData()
	if len(packet) != 26 {
		return nil
	}
	if binary.LittleEndian.Uint16(packet[0:2]) != RUUVI_MANUFACTURER_ID {
		return nil
	}
	if packet[2] != RUUVI_PACKET_VERSION {
		return nil
	}

	power := binary.BigEndian.Uint16(packet[15:17])

	return &RuuviPacket{
		Timestamp:          time.Now(),
		Address:            a.Addr(),
		RSSI:               a.RSSI(),
		Temperature:        int16(binary.BigEndian.Uint16(packet[3:5])),
		Humidity:           binary.BigEndian.Uint16(packet[5:7]),
		Pressure:           binary.BigEndian.Uint16(packet[7:9]),
		AccelerationX:      int16(binary.BigEndian.Uint16(packet[9:11])),
		AccelerationY:      int16(binary.BigEndian.Uint16(packet[11:13])),
		AccelerationZ:      int16(binary.BigEndian.Uint16(packet[13:15])),
		PowerInfoVoltage:   power >> 5,
		PowerInfoTX:        uint8(power & 0x1F),
		MovementCounter:    packet[17],
		MeasurementCounter: binary.BigEndian.Uint16(packet[18:20]),
	}
}

func main() {
	log.Println("HiThere!")

	registry := prometheus.NewRegistry()
	temperature := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_temperature_celsius"}, []string{"mac"})
	humidity := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_relative_humidity"}, []string{"mac"})
	pressure := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_pressure_pa"}, []string{"mac"})
	accelerationX := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_x"}, []string{"mac"})
	accelerationY := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_y"}, []string{"mac"})
	accelerationZ := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_z"}, []string{"mac"})
	powerVoltage := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_battery_voltage"}, []string{"mac"})
	rssi := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_rssi"}, []string{"mac"})
	movementCounter := prometheus.NewCounterVec(prometheus.CounterOpts{Name: "ruuvi_movement_count"}, []string{"mac"})
	measurementCounter := prometheus.NewHistogramVec(prometheus.HistogramOpts{Name: "ruuvi_measurement_counter",
		Buckets: []float64{1, 2, 3, 5, 10}}, []string{"mac"})

	registry.MustRegister(temperature, humidity, pressure, accelerationX, accelerationY, accelerationZ, powerVoltage, movementCounter, measurementCounter, rssi)

	deviceName := "default"
	if env := os.Getenv("BT_DEVICE"); env != "" {
		deviceName = env
	}

	listenAddress := ":10000"
	if env := os.Getenv("LISTEN_ADDRESS"); env != "" {
		listenAddress = env
	}

	log.Println("Device: ", deviceName)
	log.Println("Address: ", listenAddress)

	device, err := dev.NewDevice(deviceName)
	if err != nil {
		log.Fatalln(err)
	}

	dataChannel := make(chan RuuviPacket, 16)
	scanStopped := make(chan struct{})

	scanCtx, cancelScan := context.WithCancel(context.Background())

	go func() {
		err := device.Scan(scanCtx, true, func(a ble.Advertisement) {
			packet := TryParseRuuviPacketFromManufacturerData(a)
			if packet != nil {
				log.Println(packet)
				dataChannel <- *packet
			}
		})
		if err != context.Canceled {
			log.Fatalln(err)
		}

		close(scanStopped)
	}()

	go func() {
		http.Handle("/metrics", promhttp.HandlerFor(registry, promhttp.HandlerOpts{}))
		log.Fatal(http.ListenAndServe(listenAddress, nil))
	}()

	previousData := make(map[string]RuuviPacket)
	expTime := 2 * time.Minute
	recheckPeriod := 10 * time.Second
	check := time.After(recheckPeriod)

	signals := make(chan os.Signal)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)

	for {
		select {
		case <-signals:
			log.Println("Termination signal received")
			cancelScan()
			<-scanStopped
			log.Println("Scan stopped")

			err := device.Stop()
			if err != nil {
				log.Fatalln(err)
			}
			log.Println("Device closed")

			os.Exit(0)

		case <-check:
			expPoint := time.Now().Add(-expTime)
			for mac, data := range previousData {
				if data.Timestamp.Before(expPoint) {
					labels := prometheus.Labels{"mac": mac}
					temperature.Delete(labels)
					humidity.Delete(labels)
					pressure.Delete(labels)
					accelerationX.Delete(labels)
					accelerationY.Delete(labels)
					accelerationZ.Delete(labels)
					powerVoltage.Delete(labels)
					movementCounter.Delete(labels)
					rssi.Delete(labels)
					delete(previousData, mac)
				}
			}
			check = time.After(recheckPeriod)

		case data := <-dataChannel:
			mac := data.Address.String()
			previousValue, hasPreviousValue := previousData[mac]
			labels := prometheus.Labels{"mac": mac}
			temperature.With(labels).Set(data.GetTemperatureInCelsius())
			humidity.With(labels).Set(data.GetHumidity())
			pressure.With(labels).Set(data.GetPressure())
			accelerationX.With(labels).Set(data.GetAccelerationX())
			accelerationY.With(labels).Set(data.GetAccelerationY())
			accelerationZ.With(labels).Set(data.GetAccelerationZ())
			powerVoltage.With(labels).Set(data.GetPowerInfoVoltage())
			rssi.With(labels).Set(float64(data.RSSI))
			if hasPreviousValue {
				movementCounter.With(labels).Add(float64(data.MovementCounter - previousValue.MovementCounter))
				measurementCounter.With(labels).Observe(float64(data.MeasurementCounter - previousValue.MeasurementCounter))

			}
			previousData[mac] = data
		}
	}
}
