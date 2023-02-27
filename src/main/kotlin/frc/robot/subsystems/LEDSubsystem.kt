package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDSubsystem : SubsystemBase() {
    private var ds = DriverStation.isTeleop()
    var led = AddressableLED(0)
    var ledBuffer = AddressableLEDBuffer(24)
    var coneNeeded : Boolean = false
    var cubeNeeded : Boolean = false
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
        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomous()) {
                autoColor()
            } else if (DriverStation.isTeleop()) {
                teleop()
            }
        } else if (DriverStation.isDisabled()) {
            flashOfColor()
        }
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
            if (coneNeeded == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 255, 0, 200)

                }
                led.setData(ledBuffer)
            } else if (cubeNeeded == true) {
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
    fun flashOfColor() {
        if (DriverStation.isTeleop()) {
            if (time >= 1) {
                ledBuffer.setRGB(time / 4, 255, 0, 0)
                led.setData(ledBuffer)
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 0, 0, 0)
                }
                led.setData(ledBuffer)

                ledBuffer.setRGB(time / 8, 0, 0, 255)
                led.setData(ledBuffer)
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setRGB(i, 0, 0, 0)
                }
                led.setData(ledBuffer)
            }
        }
    }
}