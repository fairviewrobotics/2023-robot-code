package frc.robot.commands

import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
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

/**
 * For quickly testing a single motor
 */
class QuickSpin(val controller: XboxController, val motor: SparkMaxSubsystem, val currentLimit: Int): CommandBase() {
    val voltageTopic = NetworkTableInstance.getDefault().getTable("QuickSpin").getDoubleTopic("Voltage").publish()
    val currentTopic = NetworkTableInstance.getDefault().getTable("QuickSpin").getDoubleTopic("Current").publish()
    val powerTopic = NetworkTableInstance.getDefault().getTable("QuickSpin").getDoubleTopic("Power").publish()
    init {
        addRequirements(motor)
        motor.x.setSmartCurrentLimit(currentLimit)
        motor.x.burnFlash()
    }

    override fun execute() {
        motor.x.setVoltage(controller.leftY * 12.0)
        voltageTopic.set(motor.x.appliedOutput / motor.x.outputCurrent)
        currentTopic.set(motor.x.outputCurrent)
        powerTopic.set(motor.x.appliedOutput)
    }

    override fun end(interrupted: Boolean) {
        motor.x.setVoltage(0.0)
    }
}

class AbsoluteEncoderSetting(val controller: XboxController, val motor: SparkMaxSubsystem, val currentLimit: Int) : CommandBase() {
    init {
        addRequirements(motor)
        motor.x.setSmartCurrentLimit(currentLimit)
        motor.x.burnFlash()
        motor.x.encoder.positionConversionFactor = 1.0
        motor.x.encoder.velocityConversionFactor = 1.0
    }

    val positionRaw = NetworkTableInstance.getDefault().getTable("EncoderSetting").getDoubleTopic("RawPosition").publish()
    val positionTrue = NetworkTableInstance.getDefault().getTable("EncoderSetting").getDoubleTopic("TruePosition").publish()

    val position get() = 1.0
    override fun execute() {
        motor.x.setVoltage((controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0)

        positionRaw.set(motor.x.encoder.position)
        positionTrue.set(position)
    }
}

class ElevatorSetting(val motor: SparkMaxSubsystem, val controller: XboxController, val bottomBreaker: DigitalInput, val topBreaker: DigitalInput) : CommandBase() {
    init {
        addRequirements(motor)
        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        motor.x.encoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityMultiplier

    }

    object Telemetry {
        val position = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getDoubleTopic("Position").publish()
        val topBreaker = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getBooleanTopic("Top Hit").publish()
        val bottomBreaker = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getBooleanTopic("Bottom Hit").publish()
    }

    val topHit get() = !topBreaker.get()
    val bottomHit get() = !bottomBreaker.get()

    override fun execute() {
        var elevatorVoltage = (controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0

        if (topHit && elevatorVoltage > 0.0) {
            elevatorVoltage = 0.0
            //motor.x.encoder.position = ArmConstants.elevatorMaxHeight
        }

        if (bottomHit && elevatorVoltage < 0.0) {
            elevatorVoltage = 0.0
            motor.x.encoder.position = ArmConstants.elevatorMinHeight
        }

        motor.x.setVoltage(elevatorVoltage)
        Telemetry.position.set(motor.x.encoder.position)
        Telemetry.topBreaker.set(topHit)
        Telemetry.bottomBreaker.set(bottomHit)
    }
}