```
GOOS=linux GOARM=6 GOARCH=arm go build -o ruuvi_exporter_server cmd/main.go
scp ruuvi_exporter_server pi@freezer-gateway-1.local:~/.
scp ruuvi_exporter.service pi@freezer-gateway-1.local:~/. 
```

```
sudo mv ruuvi_exporter.service /etc/systemd/system/
sudo chown root:root /etc/systemd/system/ruuvi_exporter.service
sudo systemctl daemon-reload
sudo systemctl status ruuvi_exporter
sudo systemctl enable ruuvi_exporter
sudo systemctl start ruuvi_exporter
sudo systemctl status ruuvi_exporter
```


```
sudo apt-get update 
sudo apt-get install bluez libbluetooth-dev
sudo apt-get dist-upgrade

wget https://dl.google.com/go/go1.14.3.linux-armv6l.tar.gz
sudo tar -C /usr/local -xzf go1.14.3.linux-armv6l.tar.gz
rm go1.14.3.linux-armv6l.tar.gz
echo 'PATH=$PATH:/usr/local/go/bin' >> ~/.profile 
echo 'GOPATH=$HOME/golang' >> ~/.profile

cd ~/ruuvi_exporter

go mod download

go build cmd/main.go

sudo ./main
```