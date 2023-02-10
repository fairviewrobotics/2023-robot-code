package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDSubsystem : SubsystemBase() {
    private var ds = DriverStation.isTeleop()
    var led = AddressableLED(0)
    var ledBuffer = AddressableLEDBuffer(24)
    var runningLights : Boolean = false
    var autoBalancing : Boolean = false
    //TODO: Remove the autobalancing var later and replace with var from button if autobalancing
    var gyroUpOrDown : Boolean = false
    var gyroFlat : Boolean = false
    var rainbow : Boolean = false
    //TODO: above is not needed
    //TODO: Remember, the lights are RBG not RGB so G and B must be switched to get the correct color
    init {
        led.setLength(ledBuffer.length)
        led.setData(ledBuffer)
        led.start()
    }
    var time = 0
    override fun periodic() {
        time += 1
        println(time)
        if (time >= 96) {
            time = 0
        }
        if (autoBalancing == false) {
            if (runningLights == false) {
                offLeds()
            } else if (runningLights == true) {
                if (DriverStation.isAutonomous()) {
                    if (autoBalancing == true) {
                        balanceColor()
                    } else {
                        autoColor()
                    }
                } else {
                    if (autoBalancing == true) {
                        balanceColor()
                    } else if (rainbow == false) {
                        teleop()
                    } else if (rainbow == true) {
                        rainbowColor()
                    }
                }
            }
        }
    }
    fun runningLightsOn() {
        runningLights = true
    }
    fun runningLightsOff() {
        runningLights = false
    }
    fun rainbowYes() {
        rainbow = true
    }
    fun rainbowNo() {
        rainbow = false
    }
    fun offLeds() {
        for (i in 0..ledBuffer.length-1) {
            ledBuffer.setRGB(i, 0, 0, 0)
        }
        led.setData(ledBuffer)
    }
    fun autoColor() {
        if (DriverStation.isAutonomous()) {
            for (i in 0..ledBuffer.length - 1) {
                ledBuffer.setRGB(i, 0, 255, 0)
            }
            led.setData(ledBuffer)
        }
    }
    fun teleop() {
        if (DriverStation.isTeleop()) {
            if (gyroUpOrDown == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 255, 0, 200)

                }
                led.setData(ledBuffer)
            } else if (gyroFlat == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 95, 150, 0)
                }
                led.setData(ledBuffer)
            } else {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 0, 0, 0)
                }
                led.setData(ledBuffer)
            }
        }
        //connect to network tables on strategy app
        //if strategy app says pickup cone, shine yellow solid lights
    }
    fun balanceColor() {
            if (gyroUpOrDown == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 255, 0, 0)
                }
                led.setData(ledBuffer)
            } else if (gyroFlat == true) {
                for (i in 0..ledBuffer.length-1) {
                    ledBuffer.setRGB(i, 0, 0, 255)
                }
                led.setData(ledBuffer)
            } else {
                for (i in 0..ledBuffer.length-1) {
                    ledBuffer.setRGB(i, 0, 0, 0)
                }
                led.setData(ledBuffer)
            }
    }
    fun rainbowColor() {
        ledBuffer.setRGB(0, 255, 0, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(1, 200, 0, 10)
        led.setData(ledBuffer)
        ledBuffer.setRGB(2, 150, 0, 20)
        led.setData(ledBuffer)
        ledBuffer.setRGB(3, 200, 0, 50)
        led.setData(ledBuffer)
        ledBuffer.setRGB(4, 255, 0, 77)
        led.setData(ledBuffer)
        ledBuffer.setRGB(5, 200, 0, 115)
        led.setData(ledBuffer)
        ledBuffer.setRGB(6, 200, 0, 175)
        led.setData(ledBuffer)
        ledBuffer.setRGB(7, 255, 0, 205)
        led.setData(ledBuffer)
        ledBuffer.setRGB(8, 175, 0, 200)
        led.setData(ledBuffer)
        ledBuffer.setRGB(9, 75, 0, 215)
        led.setData(ledBuffer)
        ledBuffer.setRGB(10, 0, 0, 255)
        led.setData(ledBuffer)
        ledBuffer.setRGB(11, 0, 100, 210)
        led.setData(ledBuffer)
        ledBuffer.setRGB(12, 0, 175, 170)
        led.setData(ledBuffer)
        ledBuffer.setRGB(13, 0, 255, 150)
        led.setData(ledBuffer)
        ledBuffer.setRGB(14, 0, 175, 100)
        led.setData(ledBuffer)
        ledBuffer.setRGB(15, 0, 200, 50)
        led.setData(ledBuffer)
        ledBuffer.setRGB(16, 0, 255, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(17, 50, 125, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(18, 100, 160, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(19, 148, 211, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(20, 115, 185, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(21, 90, 155, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(22, 75, 120, 0)
        led.setData(ledBuffer)
        ledBuffer.setRGB(23, 10, 50, 0)
        led.setData(ledBuffer)
    }
    fun flashOfConeColor() {
        if (DriverStation.isTeleop()) {
                if (time >= 1) {
                    ledBuffer.setRGB(time / 4, 255, 0, 200)
                    led.setData(ledBuffer)

                    for (i in 0..ledBuffer.length - 1) {
                        ledBuffer.setRGB(i, 0, 0, 0)
                    }
                    led.setData(ledBuffer)


            }

        }
    }
}