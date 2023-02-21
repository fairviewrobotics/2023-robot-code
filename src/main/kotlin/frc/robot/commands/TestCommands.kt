package frc.robot.commands

import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.SparkMaxSubsystem

/**
 * Rezero the swerve modules
 */

class RezeroingCommand(val quad: String, val controller: XboxController, val turning: SparkMaxSubsystem, val driving: SparkMaxSubsystem) : CommandBase() {
    val absoluteEncoder = turning.x.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val absoluteEncoderTopic = NetworkTableInstance.getDefault().getTable("Rezeroing").getDoubleTopic(quad).publish()

    init {
        absoluteEncoder.inverted = DrivetrainConstants.turningEncoderReversed
        absoluteEncoder.positionConversionFactor = DrivetrainConstants.turningEncoderPositionFactor
        absoluteEncoder.velocityConversionFactor = DrivetrainConstants.turningEncoderVelocityFactor

        addRequirements(turning)
        addRequirements(driving)

        turning.x.setSmartCurrentLimit(20)
        driving.x.setSmartCurrentLimit(40)
        turning.x.burnFlash()
        driving.x.burnFlash()
    }

    override fun execute() {
        super.execute()

        if (controller.aButton) {
            driving.x.setVoltage(1.0)
        } else {
            driving.x.setVoltage(0.0)
        }

        if (controller.bButton) {
            turning.x.setVoltage(6.0)
        } else {
            turning.x.setVoltage(0.0)
        }

        absoluteEncoderTopic.set(absoluteEncoder.position)
    }
}