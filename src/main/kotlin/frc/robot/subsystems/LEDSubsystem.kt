package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase

enum class LEDSubsystemState {
    GYRO_UP_OR_DOWN,
    GYRO_FLAT,
    RAINBOW,
    CUBE,
    CONE,
    NULL
}

class LEDSubsystem(val port: Int) : SubsystemBase() {
    var led = AddressableLED(port)
    var ledBuffer = AddressableLEDBuffer(24)
    var state = LEDSubsystemState.NULL
    init {
        led.setLength(ledBuffer.length)
        led.setData(ledBuffer)
        led.start()
    }
    var time = 0
    override fun periodic() {
        time += 1
        println(time)
        if (time >= 102) {
            time = 0
        }
        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomous()) {
                autoColor()
            } else if (DriverStation.isTeleop()) {
                teleop()
            }
        } else if (DriverStation.isDisabled()) {
            rainbow()
        }
        else if(state == LEDSubsystemState.CUBE)
        {
            cube()
        }
        else if(state == LEDSubsystemState.CONE)
        {
            cone()
        }
        else
        {
            red()
        }

    }

    fun setLEDState(sstate: LEDSubsystemState) {
        state = sstate
    }

    fun red() {
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
        if (DriverStation.isTeleop()) {
            if (coneNeeded == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 25, 255, 255)
                }
                led.setData(ledBuffer)
            } else if (cubeNeeded == true) {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 144, 255, 255)
                }
                led.setData(ledBuffer)
            } else {
                for (i in 0..ledBuffer.length - 1) {
                    ledBuffer.setHSV(i, 0, 0, 0)
                }
                led.setData(ledBuffer)
            }
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
