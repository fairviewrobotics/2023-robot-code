package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxLimitSwitch
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants

class ArmSubsystem(val elbowMotorID: Int, val elevatorMotorID: Int) :SubsystemBase() {
    val elevatorMotor = CANSparkMax(elevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(elbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)


    val elevatorEncoder = elevatorMotor.getEncoder()
    val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    val reverseLimit = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    var elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elevatorPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ntDesiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ntDesiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()

    var desiredElevatorState = 0.0
    var desiredElbowState = 0.0

    var inst = NetworkTableInstance.getDefault()
    var table = inst.getTable("Arm")

    val elbowPid = PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD)
    val elevatorPid = ProfiledPIDController(
        ArmConstants.elevatorP,
        ArmConstants.elevatorI,
        ArmConstants.elevatorD, ArmConstants.elevatorTrapezoidConstraints, 0.02)




    init {
        elbowEncoder.setPositionConversionFactor(ArmConstants.elbowEncoderPosMultiplier)
        elbowEncoder.setVelocityConversionFactor(ArmConstants.elbowEncoderVelocityMultiplier)
        elevatorEncoder.setPositionConversionFactor(ArmConstants.elevatorEncoderPositionConversionFactor)
        elevatorEncoder.setVelocityConversionFactor(ArmConstants.elevatorEncoderVelocityMultiplier)
    }
    fun SetDesired(ElbowPos: Double, ElevatorPos: Double)
    {
        desiredElbowState = ElbowPos

        desiredElevatorState = ElevatorPos
    }

    override fun periodic() {
        super.periodic()
        /** be setting the ntvalue for elbow position **/
        elevatorMotor.setVoltage((elevatorPid.calculate(elevatorEncoder.position, desiredElevatorState)))
        elbowMotor.setVoltage((elbowPid.calculate(elevatorEncoder.position, desiredElbowState))+ ArmConstants.elbowFeedForward.calculate(desiredElbowState,2.5))

        //This part is the network table bit
        elbowPos.set(elbowEncoder.position)
        elevatorVelocity.set(elbowEncoder.velocity)
        elevatorPos.set(elevatorEncoder.position)
        elevatorVelocity.set(elevatorEncoder.velocity)
        elbowPos.set(elbowEncoder.position)
        ntDesiredElbow.set(desiredElbowState)
        ntDesiredElevator.set(desiredElevatorState)


        //these two ifs set the encoder when it hits a limit switch

        if(reverseLimit.isPressed)
        {
            elevatorEncoder.position = ArmConstants.elevatorMinHeight
        }

        if(forwardLimit.isPressed)
        {
            elevatorEncoder.position = ArmConstants.elevatorMaxHeight
        }
    }
}