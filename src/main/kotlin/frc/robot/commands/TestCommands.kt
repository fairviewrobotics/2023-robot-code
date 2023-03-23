package frc.robot.commands

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.DigitalInputSubsystem
import frc.robot.subsystems.SparkMaxSubsystem

///**
// * Rezero the swerve modules
// */
//
//class RezeroingCommand(val quad: String, val controller: XboxController, val turning: SparkMaxSubsystem, val driving: SparkMaxSubsystem) : CommandBase() {
//    val absoluteEncoder = turning.x.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
//    val absoluteEncoderTopic = NetworkTableInstance.getDefault().getTable("Rezeroing").getDoubleTopic(quad).publish()
//
//    init {
//        absoluteEncoder.inverted = DrivetrainConstants.turningEncoderReversed
//        absoluteEncoder.positionConversionFactor = DrivetrainConstants.turningEncoderPositionFactor
//        absoluteEncoder.velocityConversionFactor = DrivetrainConstants.turningEncoderVelocityFactor
//
//        addRequirements(turning)
//        addRequirements(driving)
//
//        turning.x.setSmartCurrentLimit(20)
//        driving.x.setSmartCurrentLimit(40)
//        turning.x.burnFlash()
//        driving.x.burnFlash()
//    }
//
//    override fun execute() {
//        super.execute()
//
//        if (controller.aButton) {
//            driving.x.setVoltage(1.0)
//        } else {
//            driving.x.setVoltage(0.0)
//        }
//
//        if (controller.bButton) {
//            turning.x.setVoltage(6.0)
//        } else {
//            turning.x.setVoltage(0.0)
//        }
//
//        absoluteEncoderTopic.set(absoluteEncoder.position)
//    }
//}
//
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

//class AbsoluteEncoderSetting(val controller: XboxController, val motor: SparkMaxSubsystem, val currentLimit: Int) : CommandBase() {
//    init {
//        addRequirements(motor)
//        motor.x.setSmartCurrentLimit(currentLimit)
//        motor.x.burnFlash()
//        motor.x.encoder.positionConversionFactor = 1.0
//        motor.x.encoder.velocityConversionFactor = 1.0
//    }
//
//    val positionRaw = NetworkTableInstance.getDefault().getTable("EncoderSetting").getDoubleTopic("RawPosition").publish()
//    val positionTrue = NetworkTableInstance.getDefault().getTable("EncoderSetting").getDoubleTopic("TruePosition").publish()
//
//    val position get() = 1.0
//    override fun execute() {
//        motor.x.setVoltage((controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0)
//
//        positionRaw.set(motor.x.encoder.position)
//        positionTrue.set(position)
//    }
//}
//
//class ElevatorSetting(val motor: SparkMaxSubsystem, val controller: XboxController, val bottomBreaker: DigitalInput, val topBreaker: DigitalInput) : CommandBase() {
//    init {
//        addRequirements(motor)
//        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
//        motor.x.encoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityConversionFactor
//
//    }
//
//    object Telemetry {
//        val position = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getDoubleTopic("Position").publish()
//        val topBreaker = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getBooleanTopic("Top Hit").publish()
//        val bottomBreaker = NetworkTableInstance.getDefault().getTable("ElevatorSetting").getBooleanTopic("Bottom Hit").publish()
//    }
//
//    val topHit get() = !topBreaker.get()
//    val bottomHit get() = !bottomBreaker.get()
//
//    override fun execute() {
//        var elevatorVoltage = (controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0
//
//        if (topHit && elevatorVoltage > 0.0) {
//            elevatorVoltage = 0.0
//            //motor.x.encoder.position = ArmConstants.elevatorMaxHeight
//        }
//
//        if (bottomHit && elevatorVoltage < 0.0) {
//            elevatorVoltage = 0.0
//            motor.x.encoder.position = ArmConstants.elevatorMinHeight
//        }
//
//        motor.x.setVoltage(elevatorVoltage)
//        Telemetry.position.set(motor.x.encoder.position)
//        Telemetry.topBreaker.set(topHit)
//        Telemetry.bottomBreaker.set(bottomHit)
//    }
//}
//
//
//class DigitalInputTest(val input: DigitalInputSubsystem) :CommandBase() {
//    init {
//        addRequirements(input)
//    }
//
//    val DigitalInput = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("DigitalInput").getEntry(false)
//
//    override fun execute()
//    {
//        DigitalInput.set(input.x.get())
//    }
//
//}
//
//class EncoderAndLinebreakerTest(val bottomBreaker: DigitalInputSubsystem, val topBreaker: DigitalInputSubsystem, val motor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
//    val convertedPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Position").getEntry(0.0)
//    val bottomHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("BottomHit").getEntry(false)
//    val topHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("TopHit").getEntry(false)
//
//    init {
//        addRequirements(bottomBreaker)
//        addRequirements(topBreaker)
//        addRequirements(motor)
//
//        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
//        motor.x.inverted = ArmConstants.elevatorMotorInverted
//    }
//
//    override fun execute() {
//        if (controller.xButtonPressed) {
//            motor.x.encoder.position = 0.0
//        }
//
//        var up = controller.rightTriggerAxis;
//        var down = controller.leftTriggerAxis;
//
//        val top = !topBreaker.x.get()
//        val bottom = !bottomBreaker.x.get()
//
//        if (top) {
//            motor.x.encoder.position = ArmConstants.elevatorMaxHeight
//            up = 0.0
//        }
//
//        if (bottom) {
//            motor.x.encoder.position = ArmConstants.elevatorMinHeight
//            down = 0.0
//        }
//
//        motor.x.set((up - down));
//
//        convertedPosition.set(motor.x.encoder.position)
//        topHit.set(top)
//        bottomHit.set(bottom)
//    }
//}
//
//class PIDElevatorTuning(val bottomBreaker: DigitalInputSubsystem, val topBreaker: DigitalInputSubsystem, val motor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
//    val convertedPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Position").getEntry(0.0)
//    val bottomHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("BottomHit").getEntry(false)
//    val topHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("TopHit").getEntry(false)
//    val motorVelocity = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("motorVelocity").getEntry(0.0)
//    val outputNT = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Output").getEntry(0.0)
//    val errorNT = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Error").getEntry(0.0)
//
//    val pid = ProfiledPIDController(3.0,0.01,0.05, TrapezoidProfile.Constraints(5.5,100.0))
//    var position = motor.x.encoder.position
//    init {
//        addRequirements(bottomBreaker)
//        addRequirements(topBreaker)
//        addRequirements(motor)
//
//        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
//        motor.x.inverted = ArmConstants.elevatorMotorInverted
//        pid.setTolerance(0.025)
//    }
//
//    override fun execute() {
//        if (controller.xButtonPressed) {
//            motor.x.encoder.position = 0.0
//        }
//
//        if (controller.aButtonPressed) {
//            position = 0.0
//        }
//
//        if (controller.yButtonPressed) {
//            position = 0.5
//        }
//
//        if (controller.xButtonPressed) {
//            position = ArmConstants.elevatorMaxHeight
//        }
//
//        val top = !topBreaker.x.get()
//        val bottom = !bottomBreaker.x.get()
//
//        val setpoint = position
//        val measurement = motor.x.encoder.position
//
//        var output = pid.calculate(measurement, setpoint)
//        if (top) {
//            if (output > 0.0) {
//                output = 0.0
//            }
//
//            motor.x.encoder.position = ArmConstants.elevatorMaxHeight
//        }
//
//        if (bottom) {
//            if (output < 0.0) {
//                output = 0.0
//            }
//
//            motor.x.encoder.position = ArmConstants.elevatorMinHeight
//        }
//
//        motor.x.set(output)
//
//        convertedPosition.set(motor.x.encoder.position)
//        topHit.set(top)
//        bottomHit.set(bottom)
//        outputNT.set(output)
//        motorVelocity.set(motor.x.encoder.velocity)
//        errorNT.set(pid.positionError)
//    }
//}
//
//
//
//class IntakeSpinTest(val leftMotor: SparkMaxSubsystem, val rightMotor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
//    init {
//        addRequirements(leftMotor)
//        addRequirements(rightMotor)
//
//        leftMotor.x.inverted = ArmConstants.intakeMotorInverted
//        rightMotor.x.inverted = ArmConstants.intakeMotorInverted
//    }
//
//    override fun execute() {
//        val leftMotorSpeed =
//            if (controller.leftBumperPressed) {
//                controller.leftTriggerAxis
//            } else {
//                -controller.leftTriggerAxis
//            }
//        val rightMotorSpeed =
//            if (controller.rightBumperPressed) {
//                controller.rightTriggerAxis
//            } else {
//                -controller.rightTriggerAxis
//            }
//
//        leftMotor.x.set(leftMotorSpeed)
//        rightMotor.x.set(rightMotorSpeed)
//    }
//}
//
//class moveSlowly(val motor: CANSparkMax, val controller: XboxController) : CommandBase()
//{
//    init {
//        motor.setSmartCurrentLimit(40)
//    }
//
//    override fun execute()
//    {
//        motor.setVoltage(controller.rightX * 12.0)
//
//    }
//}