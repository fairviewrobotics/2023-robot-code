package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

class SparkMaxSubsystem(x: CANSparkMax): SubsystemBase() {
    val controller = x
}