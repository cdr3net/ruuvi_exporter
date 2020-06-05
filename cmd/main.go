package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/godbus/dbus"
	"github.com/muka/go-bluetooth/api"
	"github.com/muka/go-bluetooth/bluez/profile/adapter"
	"github.com/muka/go-bluetooth/bluez/profile/device"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

const RUUVI_MANUFACTURER_ID uint16 = 0x0499
const RUUVI_PACKET_VERSION uint8 = 5

type RuuviPacket struct {
	Timestamp time.Time
	// Temperature in celsius / 200
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

//
func (p RuuviPacket) String() string {
	return fmt.Sprintf(
		" Temp = %.2f C° Humnidity = %.2f %% Pressure = %.2f Pa AccelerationXYZ = (%.2f, %.2f, %.2f) Voltage = %.2f V "+
			"TXPower = %.2f dBm MovementCounter = %d MeasurmentCounter = %d",
		p.GetTemperatureInCelsius(), p.GetHumidity(), p.GetPressure(), p.GetAccelerationX(), p.GetAccelerationY(), p.GetAccelerationZ(),
		p.GetPowerInfoVoltage(), p.GetPowerInfoTX(), p.GetMovementCounter(), p.GetMeasurementCounter())
}

func TryParseRuuviPacketFromManufacturerData(data map[uint16]interface{}) *RuuviPacket {

	if data[RUUVI_MANUFACTURER_ID] == nil {
		return nil
	}
	packet := data[RUUVI_MANUFACTURER_ID].([]byte)

	if len(packet) != 24 {
		return nil
	}
	if packet[0] != RUUVI_PACKET_VERSION {
		return nil
	}

	power := binary.BigEndian.Uint16(packet[13:15])

	return &RuuviPacket{
		Timestamp:          time.Now(),
		Temperature:        int16(binary.BigEndian.Uint16(packet[1:3])),
		Humidity:           binary.BigEndian.Uint16(packet[3:5]),
		Pressure:           binary.BigEndian.Uint16(packet[5:7]),
		AccelerationX:      int16(binary.BigEndian.Uint16(packet[7:9])),
		AccelerationY:      int16(binary.BigEndian.Uint16(packet[9:11])),
		AccelerationZ:      int16(binary.BigEndian.Uint16(packet[11:13])),
		PowerInfoVoltage:   power >> 5,
		PowerInfoTX:        uint8(power & 0x1F),
		MovementCounter:    packet[15],
		MeasurementCounter: binary.BigEndian.Uint16(packet[16:18]),
	}
}

func main() {
	log.Println("HiThere!")

	signals := make(chan os.Signal)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)

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

	go func() {
		http.Handle("/metrics", promhttp.HandlerFor(registry, promhttp.HandlerOpts{}))
		log.Fatal(http.ListenAndServe(":10000", nil))
	}()

	a, err := api.GetAdapter(adapter.GetDefaultAdapterID())
	if err != nil {
		log.Println(err)
	}

	timeout, err := a.GetDiscoverableTimeout()
	if err != nil {
		log.Println(err)
	}
	log.Println("Timeout= ", timeout)

	// Set timeout for discovery
	err = a.SetDiscoverableTimeout(60)
	if err != nil {
		log.Println(err)
	}

	discoverEvents, cancel, err := api.Discover(a, nil)
	if err != nil {
		log.Println(err)
	}

	removedMap := make(map[dbus.ObjectPath]chan struct{})

	for {
		select {
		case <-signals:
			cancel()
			log.Println("Scan stopped")
			err := api.Exit()
			if err != nil {
				log.Fatalln(err)
			}
			log.Println("Device closed")
			os.Exit(0)

		case event := <-discoverEvents:

			if event.Type == adapter.DeviceRemoved {
				close(removedMap[event.Path])
				delete(removedMap, event.Path)

			} else if event.Type == adapter.DeviceAdded {

				removeCh := make(chan struct{})
				removedMap[event.Path] = removeCh

				go func() {

					dev, err := device.NewDevice1(event.Path)
					if err != nil {
						log.Fatalln(err)
					}

					properties, err := dev.WatchProperties()
					if err != nil {
						log.Fatalln(err)
					}

					for {
						select {
						case <-removeCh:
							log.Println("removed")
							err := dev.UnwatchProperties(properties)
							if err != nil {
								log.Fatalln(err)
							}
							return

						case property := <-properties:
							if property.Name == "ManufacturerData" {
								data := dev.Properties.ManufacturerData
								manufacturerData := TryParseRuuviPacketFromManufacturerData(data)
								log.Println(manufacturerData)
							}
						}

					}

				}()
			}

		}
	}

	//for discovered := range discoverEvents {
	//	fmt.Println(discovered)
	//	dev, err := device.NewDevice1(discovered.Path)
	//	if err != nil {
	//		log.Println(err)
	//	}
	//	channel := dev.GetWatchPropertiesChannel()
	//	go func() {
	//		for data := range channel {
	//			fmt.Println(data)
	//		}
	//	}()
	//	dev.Close()
	//}
	//cancel()
	//
	//registry := prometheus.NewRegistry()
	//temperature := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_temperature_celsius"}, []string{"mac"})
	//humidity := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_relative_humidity"}, []string{"mac"})
	//pressure := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_pressure_pa"}, []string{"mac"})
	//accelerationX := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_x"}, []string{"mac"})
	//accelerationY := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_y"}, []string{"mac"})
	//accelerationZ := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_z"}, []string{"mac"})
	//powerVoltage := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_battery_voltage"}, []string{"mac"})
	//rssi := prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_rssi"}, []string{"mac"})
	//movementCounter := prometheus.NewCounterVec(prometheus.CounterOpts{Name: "ruuvi_movement_count"}, []string{"mac"})
	//measurementCounter := prometheus.NewHistogramVec(prometheus.HistogramOpts{Name: "ruuvi_measurement_counter",
	//	Buckets: []float64{1, 2, 3, 5, 10}}, []string{"mac"})
	//
	//registry.MustRegister(temperature, humidity, pressure, accelerationX, accelerationY, accelerationZ, powerVoltage, movementCounter, measurementCounter, rssi)
	//
	//device, err := dev.NewDevice("default")
	//if err != nil {
	//	log.Fatalln(err)
	//}
	//
	//dataChannel := make(chan RuuviPacket, 16)
	//scanStopped := make(chan struct{})
	//
	//scanCtx, cancelScan := context.WithCancel(context.Background())
	//
	//go func() {
	//	err := device.Scan(scanCtx, true, func(a ble.Advertisement) {
	//		packet := TryParseRuuviPacketFromManufacturerData(a)
	//		if packet != nil {
	//			log.Println(packet)
	//			dataChannel <- *packet
	//		}
	//	})
	//	if err != context.Canceled {
	//		log.Fatalln(err)
	//	}
	//
	//	close(scanStopped)
	//}()
	//
	//go func() {
	//	http.Handle("/metrics", promhttp.HandlerFor(registry, promhttp.HandlerOpts{}))
	//	log.Fatal(http.ListenAndServe(":10000", nil))
	//}()
	//
	//previousData := make(map[string]RuuviPacket)
	//expTime := 2 * time.Minute
	//recheckPeriod := 10 * time.Second
	//check := time.After(recheckPeriod)
	//
	//signals := make(chan os.Signal)
	//signal.Notify(signals, os.Interrupt, syscall.SIGTERM)
	//
	//for {
	//	select {
	//	case <-signals:
	//		log.Println("Termination signal received")
	//		cancelScan()
	//		<-scanStopped
	//		log.Println("Scan stopped")
	//
	//		err := device.Stop()
	//		if err != nil {
	//			log.Fatalln(err)
	//		}
	//		log.Println("Device closed")
	//
	//		os.Exit(0)
	//
	//	case <-check:
	//		expPoint := time.Now().Add(-expTime)
	//		for mac, data := range previousData {
	//			if data.Timestamp.Before(expPoint) {
	//				labels := prometheus.Labels{"mac": mac}
	//				temperature.Delete(labels)
	//				humidity.Delete(labels)
	//				pressure.Delete(labels)
	//				accelerationX.Delete(labels)
	//				accelerationY.Delete(labels)
	//				accelerationZ.Delete(labels)
	//				powerVoltage.Delete(labels)
	//				movementCounter.Delete(labels)
	//				rssi.Delete(labels)
	//				delete(previousData, mac)
	//			}
	//		}
	//		check = time.After(recheckPeriod)
	//
	//	case data := <-dataChannel:
	//		mac := data.Address.String()
	//		previousValue, hasPreviousValue := previousData[mac]
	//		labels := prometheus.Labels{"mac": mac}
	//		temperature.With(labels).Set(data.GetTemperatureInCelsius())
	//		humidity.With(labels).Set(data.GetHumidity())
	//		pressure.With(labels).Set(data.GetPressure())
	//		accelerationX.With(labels).Set(data.GetAccelerationX())
	//		accelerationY.With(labels).Set(data.GetAccelerationY())
	//		accelerationZ.With(labels).Set(data.GetAccelerationZ())
	//		powerVoltage.With(labels).Set(data.GetPowerInfoVoltage())
	//		rssi.With(labels).Set(float64(data.RSSI))
	//		if hasPreviousValue {
	//			movementCounter.With(labels).Add(float64(data.MovementCounter - previousValue.MovementCounter))
	//			measurementCounter.With(labels).Observe(float64(data.MeasurementCounter - previousValue.MeasurementCounter))
	//
	//		}
	//		previousData[mac] = data
	//		}
	//}

}
