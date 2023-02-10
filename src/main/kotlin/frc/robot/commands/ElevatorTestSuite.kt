package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.DigitalInputSubsystem
import frc.robot.subsystems.SparkMaxSubsystem

class EncoderConversion(val controller: XboxController, val motor: SparkMaxSubsystem) : CommandBase() {
    val rawPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("RawPosition").getEntry(0.0)
    val convertedPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("ConvertedPosition").getEntry(0.0)

    val reduction = 1.0 / 48.0
    val sprocketDiameter = Units.inchesToMeters(2.144)
    val conversion = 0.003010870139
    init {
        addRequirements(motor)

        motor.x.encoder.position = 0.0
        motor.x.encoder.positionConversionFactor = 1.0
        motor.x.encoder.velocityConversionFactor = 1.0
    }

    override fun execute() {
        if (controller.xButtonPressed) {
            motor.x.encoder.position = 0.0
        }

        motor.x.set((controller.rightTriggerAxis - controller.leftTriggerAxis));

        rawPosition.set(motor.x.encoder.position)
        convertedPosition.set(motor.x.encoder.position * conversion)
    }
}

class DigitalInputTest(val input: DigitalInputSubsystem) :CommandBase() {
    init {
        addRequirements(input)
    }

    val DigitalInput = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("DigitalInput").getEntry(false)

    override fun execute()
    {
        DigitalInput.set(input.x.get())
    }

}

class EncoderAndLinebreakerTest(val bottomBreaker: DigitalInputSubsystem, val topBreaker: DigitalInputSubsystem, val motor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
    val convertedPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Position").getEntry(0.0)
    val bottomHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("BottomHit").getEntry(false)
    val topHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("TopHit").getEntry(false)

    init {
        addRequirements(bottomBreaker)
        addRequirements(topBreaker)
        addRequirements(motor)

        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        motor.x.inverted = ArmConstants.elevatorMotorInverted
    }

    override fun execute() {
        if (controller.xButtonPressed) {
            motor.x.encoder.position = 0.0
        }

        var up = controller.rightTriggerAxis;
        var down = controller.leftTriggerAxis;

        val top = !topBreaker.x.get()
        val bottom = !bottomBreaker.x.get()

        if (top) {
            motor.x.encoder.position = ArmConstants.elevatorMaxHeight
            up = 0.0
        }

        if (bottom) {
            motor.x.encoder.position = ArmConstants.elevatorMinHeight
            down = 0.0
        }

        motor.x.set((up - down));

        convertedPosition.set(motor.x.encoder.position)
        topHit.set(top)
        bottomHit.set(bottom)
    }
}

class PIDElevatorTuning(val bottomBreaker: DigitalInputSubsystem, val topBreaker: DigitalInputSubsystem, val motor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
    val convertedPosition = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Position").getEntry(0.0)
    val bottomHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("BottomHit").getEntry(false)
    val topHit = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getBooleanTopic("TopHit").getEntry(false)

    val outputNT = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Output").getEntry(0.0)
    val errorNT = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("Error").getEntry(0.0)

    val pid = PIDController(3.0,0.01,0.05)
    var position = motor.x.encoder.position
    init {
        addRequirements(bottomBreaker)
        addRequirements(topBreaker)
        addRequirements(motor)

        motor.x.encoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        motor.x.inverted = ArmConstants.elevatorMotorInverted

        pid.setTolerance(0.025)
    }

    override fun execute() {
        if (controller.xButtonPressed) {
            motor.x.encoder.position = 0.0
        }

        if (controller.aButtonPressed) {
            position = 0.0
        }

        if (controller.yButtonPressed) {
            position = 0.5
        }

        if (controller.xButtonPressed) {
            position = ArmConstants.elevatorMaxHeight
        }

        val top = !topBreaker.x.get()
        val bottom = !bottomBreaker.x.get()

        val setpoint = position
        val measurement = motor.x.encoder.position

        var output = pid.calculate(measurement, setpoint)
        if (top) {
            if (output > 0.0) {
                output = 0.0
            }

            motor.x.encoder.position = ArmConstants.elevatorMaxHeight
        }

        if (bottom) {
            if (output < 0.0) {
                output = 0.0
            }

            motor.x.encoder.position = ArmConstants.elevatorMinHeight
        }

        motor.x.set(output)

        convertedPosition.set(motor.x.encoder.position)
        topHit.set(top)
        bottomHit.set(bottom)
        outputNT.set(output)
        errorNT.set(pid.positionError)
    }
}



class IntakeSpinTest(val leftMotor: SparkMaxSubsystem, val rightMotor: SparkMaxSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(leftMotor)
        addRequirements(rightMotor)

        leftMotor.x.inverted = ArmConstants.intakeMotorInverted
        rightMotor.x.inverted = ArmConstants.intakeMotorInverted
    }

    override fun execute() {
        val leftMotorSpeed =
            if (controller.leftBumperPressed) {
                controller.leftTriggerAxis
            } else {
                -controller.leftTriggerAxis
            }
        val rightMotorSpeed =
            if (controller.rightBumperPressed) {
                controller.rightTriggerAxis
            } else {
                -controller.rightTriggerAxis
            }

        leftMotor.x.set(leftMotorSpeed)
        rightMotor.x.set(rightMotorSpeed)
    }
}