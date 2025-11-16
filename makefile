
build:
	arduino-cli compile --fqbn esp32:esp32:esp32c3 .

compile-upload:
	mkdir -p build
	arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --build-path ./build . && arduino-cli upload --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --port $$(arduino-cli board list | grep ESP32 | cut -d' ' -f1) --input-dir ./build .

list-usb:
	arduino-cli board list
