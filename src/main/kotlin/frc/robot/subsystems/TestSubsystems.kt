package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase

class SparkMaxSubsystem(val port: Int) : SubsystemBase() {
    val x = CANSparkMax(port, CANSparkMaxLowLevel.MotorType.kBrushless)
}

class DigitalInputSubsystem(val port: Int) : SubsystemBase() {
    val x = DigitalInput(port)
}

class DummySubsystem() : SubsystemBase() {

}