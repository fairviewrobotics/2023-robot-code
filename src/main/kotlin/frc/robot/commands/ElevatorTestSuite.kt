package frc.robot.commands

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
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

class absoluteEncoderTest(val controller: XboxController, val motor: SparkMaxSubsystem) : CommandBase()
{
    val encoderPos = NetworkTableInstance.getDefault().getTable("ElevatorTestSuite").getDoubleTopic("encoderpos").getEntry(0.0)
    val elbowEncoder = motor.x.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    init {
        addRequirements(motor)
    }

    override fun execute() {

        //motor.x.set((controller.rightTriggerAxis - controller.leftTriggerAxis));

        encoderPos.set(elbowEncoder.position)
    }
}

class elbowPIDTest(val motor: SparkMaxSubsystem ) : CommandBase()
{
    val setpoint = 0.0;
    val kp = 1.0;
    val ki = 0.0;
    val kd = 0.0;
    val pid = PIDController(kp, ki, kd)

    //FIXME recalculate values we used wrong mass
    val ks = 0.0; //what's a ks?
    val kg = 3.42;
    val kv = 1.25;
    val ka = 0.19;
    var feedForward = ArmFeedforward(ks,kg, kv, ka)

    init{
        pid.setpoint = setpoint
        addRequirements(motor)
    }
    override fun execute()
    {
        motor.x.setVoltage(pid.calculate(motor.x.encoder.position, setpoint)+feedForward.calculate(setpoint, 0.0))

    }


}
