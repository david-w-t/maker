# Arduino Uno WIFI Rev. 2

Steps to prepare Arduino IDE:
1. menu-Tools-Board-Board Manager...Install 'Arduino megaAVR Boards'.
2. menu-Sketch-Include Library-Manage Libraries...Install 'WiFiNINA'.
3. Steps for updating the board firmware:
	1. Download the nightly build of Arduino IDE: https://downloads.arduino.cc/arduino-nightly-windows.zip and use it in the following steps.
	2. menu-File-Examples-WiFiNINA-Tools-FirmwareUpdater...Upload this sketch to the board.
	3. menu-Tools-WiFi101 / WiFiNINA Firmware Updater...Choose com port for board, Select the latest firmware, Update Firmware.
