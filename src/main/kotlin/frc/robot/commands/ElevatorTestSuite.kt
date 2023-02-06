package frc.robot.commands

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

        if (top)
            up = 0.0

        if (bottom)
            down = 0.0

        motor.x.set((up - down));

        convertedPosition.set(motor.x.encoder.position)
        topHit.set(top)
        bottomHit.set(bottom)
    }
}