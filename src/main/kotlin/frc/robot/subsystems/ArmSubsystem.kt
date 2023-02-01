package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxLimitSwitch
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.ArmConstants

class ArmSubsystem(val ElbowMotorID: Int, val ElevatorMotorID: Int) :SubsystemBase() {
    val ElevatorMotor = CANSparkMax(ElevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val ElbowMotor = CANSparkMax(ElbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

    val ElevatorEncoder = ElevatorMotor.getEncoder()
    val ElbowEncoder = ElbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val ForwardLimit = ElevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    val ReverseLimit = ElbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    var ElbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ElbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ElevatorPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ElevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var NtDesiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var NtDesiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()

    var Inst = NetworkTableInstance.getDefault()
    var Table = Inst.getTable("Arm")

    //FF vals
    val ElbowkA = 0.0
    val ElbowkG = 0.0
    val ElbowkS = 0.0
    val ElbowkV = 0.0

    val ElevatorkA = 0.0
    val ElevatorkV = 0.0
    val ElevatorkS = 0.0
    val ElevatorkG = 0.0

    var ElbowFeedForward = ArmFeedforward(ElbowkS, ElbowkG, ElbowkV, ElbowkA)
    var ElevatorFeedForward = ElevatorFeedforward(ElevatorkS, ElevatorkG, ElevatorkV, ElevatorkA)


    //PID vals
    //PID stuff
    //also needs tuning
    val ElevatorP = 0.1
    val ElevatorI = 0.1
    val ElevatorD = 0.1
    val ElbowP = 0.1
    val ElbowI = 0.1
    val ElbowD = 0.1
    val ElbowPid = PIDController(ElbowP, ElbowI, ElbowD)
    val ElevatorPid = PIDController(ElevatorP,ElevatorI,ElevatorD)




    init {
        ElbowEncoder.setPositionConversionFactor(ArmConstants.ElbowEncoderPosMultiplier)
        ElbowEncoder.setVelocityConversionFactor(ArmConstants.ElbowEncoderVelocityMultiplier)
        ElevatorEncoder.setPositionConversionFactor(ArmConstants.ElevatorEncoderPosMultiplier)
        ElevatorEncoder.setVelocityConversionFactor(ArmConstants.ElevatorEncoderVelocityMultiplier)
    }
    fun SetDesired(ElbowPos: Double, ElevatorPos: Double)
    {
            ArmConstants.ElbowDesiredPos = ElbowPos

            ArmConstants.ElevatorDesiredPos = ElevatorPos
    }

    override fun periodic() {
        super.periodic()
        /** be setting the ntvalue for elbow position **/
            ElevatorMotor.setVoltage((ElevatorPid.calculate(ElevatorEncoder.position, ArmConstants.ElevatorDesiredPos)*ArmConstants.ElevatorSpeedMultiplier)+ElevatorFeedForward.calculate(5.0))

            ElbowMotor.setVoltage((ElbowPid.calculate(ElevatorEncoder.position, ArmConstants.ElbowDesiredPos)*ArmConstants.ElbowSpeedMultiplier)+ElbowFeedForward.calculate(5.0,5.0))

        //This part is the network table bit
        //TODO get speeds
        ElbowPos.set(ElbowEncoder.position)
        ElevatorVelocity.set(ElbowEncoder.velocity)
        ElevatorPos.set(ElevatorEncoder.position)
        ElevatorVelocity.set(ElevatorEncoder.velocity)
        ElbowPos.set(ElbowEncoder.position)
        NtDesiredElbow.set(ArmConstants.ElbowDesiredPos)
        NtDesiredElevator.set(ArmConstants.ElevatorDesiredPos)


        //these two ifs set the encoder when it hits a limit switch

        if(ReverseLimit.isPressed)
        {
            ElevatorEncoder.position = ArmConstants.ElevatorBottomPosEnocder
        }

        if(ForwardLimit.isPressed)
        {
            ElevatorEncoder.position = ArmConstants.ElevatorTopPosEnocder
        }
    }
}