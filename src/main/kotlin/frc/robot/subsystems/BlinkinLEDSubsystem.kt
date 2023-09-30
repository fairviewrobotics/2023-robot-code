package frc.robot.subsystems


import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.SubsystemBase

class BlinkinLEDSubsystem : SubsystemBase(){

    val BlinkinDriver = Spark(9)

    var LEDNT = NetworkTableInstance.getDefault().getTable("LEDs").getDoubleTopic("LEDValue").publish()


    override fun periodic() {
        super.periodic()
        LEDNT.set(BlinkinDriver.get())
    }

    fun setLED(value: Double) {
        BlinkinDriver.set(value)
    }


}