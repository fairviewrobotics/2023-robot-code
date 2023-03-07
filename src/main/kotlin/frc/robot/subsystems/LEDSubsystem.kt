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
        if (state == LEDSubsystemState.GYRO_UP_OR_DOWN)
        {
            gyroUpOrDown()
        }
        else if(state == LEDSubsystemState.GYRO_FLAT)
        {
            gyroFlat()
        }
        else if(state == LEDSubsystemState.RAINBOW)
        {
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
            ledBuffer.setRGB(i, 179, 20, 20)
        }
        led.setData(ledBuffer)
    }

    private fun gyroFlat() {
        for (i in 0..ledBuffer.length - 1) {
            ledBuffer.setRGB(i, 95, 150, 0)
        }
        led.setData(ledBuffer)
    }

    private fun gyroUpOrDown() {
        for (i in 0..ledBuffer.length - 1) {
            ledBuffer.setRGB(i, 255, 0, 200)

        }
        led.setData(ledBuffer)
    }


    fun rainbow() {

        for (i in 0..23) {
            led.setData(ledBuffer)
            ledBuffer.setHSV(i, ((i+time) % 23)*180/23, 255, 255)
        }
    }
    fun cone() {
        for (i in 0..22 step 2) {
            led.setData(ledBuffer)
            ledBuffer.setHSV((i + time) % 23, 26, 255, 255)
            led.setData(ledBuffer)
            ledBuffer.setHSV((i+1+time) % 23, 166, 255, 255)
        }

        led.setData(ledBuffer)
        ledBuffer.setHSV(23, 26, 255, 255)
    }

    fun cube(){
        for (i in 0..22 step 2) { //(289/365)*180
            //set the cube color
            led.setData(ledBuffer)
            ledBuffer.setHSV((i+time) % 23, 143, 255, 255)
            //set that random other color on the other blocks
            led.setData(ledBuffer)
            ledBuffer.setHSV((i+1+time) % 23, 157, 255, 255)
        }
        //set the one obnoxious non-even one
        led.setData(ledBuffer)
        ledBuffer.setHSV(23, 143, 255, 255)
    }

}