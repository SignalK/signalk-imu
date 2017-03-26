## Signalk IMU

This project provides the software for an IMU based on the Ultimate Sensor Fusion Solution provides at https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution/?pt=ac_prod_search.

It uses both the above IMU board and the Butterfly STM32L433 Development Board available at https://www.tindie.com/products/TleraCorp/butterfly-stm32l433-development-board/

Kris Winer has done extensive research into IMU solutions, see https://github.com/kriswiner. He may produce a ready-to-use IMU if there is a demand.

This IMU provides:
* Magnetic heading
* Rate of turn 
* Attitude (yaw, roll, pitch)
* Temperature (at least the temp on the board anyway)
* Atmospheric Pressure

Access to raw sensor data is possible with other software, see kriswiner github.

### Assembly

Obtain one of each of the above. Connect the two boards by wiring the pins as follows :
 
|Butterfly | IMU |
|---------------|-------|
| VDD (3.3V)| VDD |
| GND | GND |
| SDA (pin20) | SDA|
| SCL (pin 21) | SCL |
| pin D7 | INT |

Try to keep the wires away from the IMU board, as they may cause magnetic interference. By placing the IMU at the head fo the Butterfly board (usb is the tail) and rotating 90deg I found most wires could be led straight back, and the INT wire taken around the side. 

I used hotmelt glue to secure both boards to a small plastic backing board. (Just use the hotmelt at the corners)

### Loading software

Obtain and install the Arduino IDE, as documented at https://github.com/GrumpyOldPizza/arduino-STM32L4

Clone this repository and open the EM7180_MPU9250_BMP280_M24512DFC_IMU.ino file in the Arduino IDE.

Plug the IMU in using the USB port, and in the Tools menu select  
* Board > Butterfly-L443C
* USB Type > Serial
* Port > select the one showing your IMU

Using the Upload Button (top left) compile and load  the IMU, waiting for the 'OK'

From Tools> Serial Monitor, open the Console, select '115200 Baud' if required. 

You should see signalk data rolling up the screen.

### Operation

The IMU is self-calibrating, and will do so over time, however this can be improved by waving the IMU around in the air near its final mounting position. Try to make the IMU rotate through every possible orientation, I find that rolling the usb cable slowly in my fingers, while turning the IMU through 360 degrees in horizontal, vertical and 45degs works.

The calibration can be saved by clicking the BTN button, if you press the wrong one it will reset, and you need to do the calibration again. You can press the button again at any time to save a better calibration.

After saving the calibration the board will reload it next boot, providing immediate calibration.

Since the board is magnetically sensitive (like all compasses) mount it somewhere on the boat where its is not near any magnetic materials, wiring, or other interference. Also avoid mounting it where vibration occurs as it will reduce the effectiveness of the accelerometer. If you can avoid it, try mounting on some vibration absorbing material.

## Output
```
{
  "context": "vessels.self",
  "updates": [
    {
      "values": [
        {
          "path": "navigation.headingMagnetic",
          "value": -2.11
        },
        {
          "path": "navigation.rateOfTurn",
          "value": 0
        },
        {
          "path": "navigation.navigation.attitude",
          "value": {
            "roll": -3.12,
            "pitch": 0.02,
            "yaw": -2.11
          }
        },
        {
          "path": "environment.inside.temperature",
          "value": -246.87
        },
        {
          "path": "environment.outside.pressure",
          "value": 100795
        }
      ],
      "source": {
        "label": "pesky.IMU"
      }
    }
  ]
}
```
