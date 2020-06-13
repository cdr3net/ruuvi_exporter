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
	"github.com/muka/go-bluetooth/bluez"
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

type gauges struct {
	temperature        *prometheus.GaugeVec
	humidity           *prometheus.GaugeVec
	pressure           *prometheus.GaugeVec
	accelerationX      *prometheus.GaugeVec
	accelerationY      *prometheus.GaugeVec
	accelerationZ      *prometheus.GaugeVec
	powerVoltage       *prometheus.GaugeVec
	movementCounter    *prometheus.CounterVec
	measurementCounter *prometheus.HistogramVec
}

type previousCountersStruct struct {
	previousMovement    uint8
	previousMeasurement uint16
	isInitialized       bool
}

func (g *gauges) setMetrics(label string, manufacturerData *RuuviPacket, previousCounters previousCountersStruct) {
	labels := prometheus.Labels{"mac": label}
	g.temperature.With(labels).Set(manufacturerData.GetTemperatureInCelsius())
	g.humidity.With(labels).Set(manufacturerData.GetHumidity())
	g.pressure.With(labels).Set(manufacturerData.GetPressure())
	g.accelerationX.With(labels).Set(manufacturerData.GetAccelerationX())
	g.accelerationY.With(labels).Set(manufacturerData.GetAccelerationY())
	g.accelerationZ.With(labels).Set(manufacturerData.GetAccelerationZ())
	g.powerVoltage.With(labels).Set(manufacturerData.GetPowerInfoVoltage())
	if previousCounters.isInitialized {
		g.movementCounter.With(labels).Add(float64(manufacturerData.MovementCounter - previousCounters.previousMovement))
		g.measurementCounter.With(labels).Observe(float64(manufacturerData.MeasurementCounter - previousCounters.previousMeasurement))
	}
}

func (g *gauges) removeMetrics(label string) {
	labels := prometheus.Labels{"mac": label}
	g.temperature.Delete(labels)
	g.humidity.Delete(labels)
	g.pressure.Delete(labels)
	g.accelerationX.Delete(labels)
	g.accelerationY.Delete(labels)
	g.accelerationZ.Delete(labels)
	g.powerVoltage.Delete(labels)
	g.movementCounter.Delete(labels)
	g.measurementCounter.Delete(labels)
}

var expTime = 2 * time.Minute

func deviceHandler(adapter *adapter.Adapter1, path dbus.ObjectPath, gauges *gauges, cancelCh chan struct{}) {
	mac := string(path)

	dev, err := device.NewDevice1(path)
	if err != nil {
		log.Fatalln(err)
	}

	properties, err := dev.WatchProperties()
	if err != nil {
		log.Fatalln(err)
	}

	previousCounters := previousCountersStruct{}
	timeout := time.After(expTime)

	terminate := func() {
		log.Println("Terminating handler for ", mac)
		gauges.removeMetrics(mac)
		err := dev.UnwatchProperties(properties)
		log.Println("Unwatch properties ", mac)

		if err != nil {
			log.Fatalln(err)
		}
		err = adapter.RemoveDevice(path)
		if err != nil {
			log.Fatalln(err)
		}
		log.Println("Device removed from adapter ", mac)
	}

	defer terminate()

	for {
		select {
		case <-cancelCh:
			log.Println("cancelCh received signal ", path)
			return

		case <-timeout:
			log.Println("timeout received signal ", path)
			return

		case property := <-properties:
			if property.Name == "ManufacturerData" {
				data := dev.Properties.ManufacturerData
				if manufacturerData := TryParseRuuviPacketFromManufacturerData(data); manufacturerData != nil {

					log.Println(mac + manufacturerData.String())
					gauges.setMetrics(mac, manufacturerData, previousCounters)

					previousCounters = previousCountersStruct{
						previousMovement:    manufacturerData.MovementCounter,
						previousMeasurement: manufacturerData.MeasurementCounter,
						isInitialized:       true,
					}

					timeout = time.After(expTime)
				}
			}
		}
	}
}

func main() {
	log.Println("HiThere!")

	cancelMap := make(map[dbus.ObjectPath]chan struct{})

	gauges := gauges{
		temperature:     prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_temperature_celsius"}, []string{"mac"}),
		humidity:        prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_relative_humidity"}, []string{"mac"}),
		pressure:        prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_pressure_pa"}, []string{"mac"}),
		accelerationX:   prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_x"}, []string{"mac"}),
		accelerationY:   prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_y"}, []string{"mac"}),
		accelerationZ:   prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_acceleration_z"}, []string{"mac"}),
		powerVoltage:    prometheus.NewGaugeVec(prometheus.GaugeOpts{Name: "ruuvi_battery_voltage"}, []string{"mac"}),
		movementCounter: prometheus.NewCounterVec(prometheus.CounterOpts{Name: "ruuvi_movement_count"}, []string{"mac"}),
		measurementCounter: prometheus.NewHistogramVec(prometheus.HistogramOpts{Name: "ruuvi_measurement_counter",
			Buckets: []float64{1, 2, 3, 5, 10}}, []string{"mac"}),
	}
	registry := prometheus.NewRegistry()

	registry.MustRegister(gauges.temperature, gauges.humidity, gauges.pressure, gauges.accelerationX,
		gauges.accelerationY, gauges.accelerationZ, gauges.powerVoltage, gauges.movementCounter, gauges.measurementCounter)

	go func() {
		http.Handle("/metrics", promhttp.HandlerFor(registry, promhttp.HandlerOpts{}))
		log.Fatal(http.ListenAndServe(":10000", nil))
	}()

	a, err := api.GetAdapter(adapter.GetDefaultAdapterID())
	if err != nil {
		log.Println(err)
	}

	discoverEvents, _, err := api.Discover(a, nil)
	if err != nil {
		log.Println(err)
	}

	signals := make(chan os.Signal)
	signal.Notify(signals, os.Interrupt, syscall.SIGTERM)

outer:
	for {
		select {
		case received := <-signals:
			log.Println("Termination signal detected")
			log.Println(received.String())
			for dev := range cancelMap {
				close(cancelMap[dev])
				delete(cancelMap, dev)
			}
			//cancel()
			log.Println("Terminated")
			break outer


		case event, opened := <-discoverEvents:
			if !opened {
				log.Println("Event channel closed")
				break outer
			}

			if event.Type == adapter.DeviceRemoved {
				log.Println("removed", event.Path)
			} else if event.Type == adapter.DeviceAdded {
				log.Println("deviceAdded", event.Path)
				cancelCh := make(chan struct{})
				cancelMap[event.Path] = cancelCh
				go deviceHandler(a, event.Path, &gauges, cancelCh)
			}
		}
	}

	log.Println("outer stopped")

	a.Close()
	log.Println("Adapter close")

	err = bluez.CloseConnections()
	if err != nil {
		log.Fatalln(err)
	}
	//err = api.Exit()
	log.Println("API exit")
	os.Exit(0)
}
