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

class ArmSubsystem(val elbowMotorID: Int, val elevatorMotorID: Int) :SubsystemBase() {
    val elevatorMotor = CANSparkMax(elevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(elbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

    val elevatorEncoder = elevatorMotor.getEncoder()
    val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    val reverseLimit = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    var elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elbowSpeed = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elevatorPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var elevatorSpeed = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ntDesiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var ntDesiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()

    var inst = NetworkTableInstance.getDefault()
    var table = inst.getTable("Arm")

    //FF vals
    val elbowkA = 0.0
    val elbowkG = 0.0
    val elbowkS = 0.0
    val elbowkV = 0.0

    val elevatorkA = 0.0
    val elevatorkV = 0.0
    val elevatorkS = 0.0
    val elevatorkG = 0.0

    var elbowFeedForward = ArmFeedforward(elbowkS, elbowkG, elbowkV, elbowkA)
    var elevatorFeedForward = ElevatorFeedforward(elevatorkS, elevatorkG, elevatorkV, elevatorkA)


    //PID vals
    //PID stuff
    //also needs tuning
    val elevatorP = 0.1
    val elevatorI = 0.1
    val elevatorD = 0.1
    val elbowP = 0.1
    val elbowI = 0.1
    val elbowD = 0.1
    val elbowPid = PIDController(elbowP, elbowI, elbowD)
    val elevatorPid = PIDController(elevatorP,elevatorI,elevatorD)




    init {
    }
    fun setDesired(elbowPos: Double, elevatorPos: Double)
    {
            ArmConstants.elbowDesiredPos = elbowPos

            ArmConstants.elevatorDesiredPos = elevatorPos
    }

    override fun periodic() {
        super.periodic()
        /** be setting the ntvalue for elbow position **/
            elevatorMotor.setVoltage((elevatorPid.calculate(elevatorEncoder.position, ArmConstants.elevatorDesiredPos)*ArmConstants.elevatorSpeedMultiplier)+elevatorFeedForward.calculate(5.0))

            elbowMotor.setVoltage((elbowPid.calculate(elevatorEncoder.position, ArmConstants.elbowDesiredPos)*ArmConstants.elbowSpeedMultiplier)+elbowFeedForward.calculate(5.0,5.0))

        //This part is the network table bit
        //TODO get speeds
        elbowPos.set(elbowEncoder.position)
        //elbowspeed.set(elbowEncoder.speed)
        elevatorPos.set(elevatorEncoder.position)
        //elevatorSpeed.set(elevatorEncoder.speed)
        elbowPos.set(elbowEncoder.position)
        ntDesiredElbow.set(ArmConstants.elbowDesiredPos)
        ntDesiredElevator.set(ArmConstants.elevatorDesiredPos)


        //these two ifs set the encoder when it hits a limit switch

        if(reverseLimit.isPressed)
        {
            elevatorEncoder.position = ArmConstants.elevatorBottomPosEnocder
        }

        if(forwardLimit.isPressed)
        {
            elevatorEncoder.position = ArmConstants.elevatorTopPosEnocder
        }
    }
}