package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer

class LEDSubsystemHSV : SubsystemBase() {
    private var ds = DriverStation.isTeleop()
    var led = AddressableLED(0)
    var ledBuffer = AddressableLEDBuffer(20)//51
    var rainbowFirstPixelHue: Int = 0
    //TODO: Remember, the lights are RBG not RGB so G and B must be switched to get the correct color
    init {
        led.setLength(ledBuffer.length)
        led.setData(ledBuffer)
        led.start()
    }
    var time = 20
    var cone = true
    override fun periodic() {
        time += 1
        println(time)
        if (time >= 40) {//102
            time = 0
        }
        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomous()) {
                autoColor()
            } else if (DriverStation.isTeleop()) {
                teleop()
            }
        } else if (DriverStation.isDisabled()) {
            teleop()
        }
    }

    fun offLeds() {
        for (i in 0..ledBuffer.length-1) {
            ledBuffer.setHSV(i, 0, 0, 0)
        }
        led.setData(ledBuffer)
    }
    fun autoColor() {
        if (DriverStation.isAutonomous()) {
            if (time >= 1) {
                ledBuffer.setHSV(time / 4, 115, 230, 255)
                led.setData(ledBuffer)
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 0, 0, 0)
                }
                led.setData(ledBuffer)
            }
        }
    }
    fun teleop() {
            if (cone == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 25, 255, 255)
                }
                led.setData(ledBuffer)
            } else if (cone == false) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 144, 255, 255)
                }
                led.setData(ledBuffer)
            } else {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 0, 255, 255)
                }
                led.setData(ledBuffer)
            }
        //connect to network tables on strategy app
        //if strategy app says pickup cone, shine yellow solid lights
    }
    fun rainbowSystem() {
        for (i in 0..ledBuffer.length - 1) {
            var hue: Int = (rainbowFirstPixelHue + (i * 180 / ledBuffer.length)) % 180
            ledBuffer.setHSV(i, hue, 255, 128)
        }
        rainbowFirstPixelHue += 3
        rainbowFirstPixelHue %= 180
    }
    fun rainbow() {
        rainbowSystem()
        led.setData(ledBuffer)
    }
}