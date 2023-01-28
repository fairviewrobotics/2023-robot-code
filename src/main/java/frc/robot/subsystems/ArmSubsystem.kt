package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxLimitSwitch
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.ArmConstants

class ArmSubsystem(val elbowMotor: CANSparkMax, val elevatorMotor: CANSparkMax) :SubsystemBase() {
    val elevatorEncoder = elevatorMotor.getEncoder()
    val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    val reverseLimit = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    var elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
    var inst = NetworkTableInstance.getDefault()
    var table = inst.getTable("Arm")





    init {
        elevatorEncoder.position = 0.0
    }
    fun setDesired(elbowPos: Double, elevatorPos: Double)
    {
        if(elbowPos <= ArmConstants.elbowThreshold)
        {
            ArmConstants.elbowDesiredPos = elbowPos
        }
        if(elevatorPos <= ArmConstants.elevatorThreshold)
        {
            ArmConstants.elevatorDesiredPos = elevatorPos
        }
    }

    override fun periodic() {
        super.periodic()
        /** be setting the ntvalue for elbow position **/
        if(elevatorEncoder.position < ArmConstants.elevatorThreshold) {
            elevatorMotor.setVoltage((ArmConstants.elevatorPid.calculate(elevatorEncoder.position, ArmConstants.elevatorDesiredPos)*ArmConstants.elevatorSpeedMultiplier)+ArmConstants.elevatorfeedforward.calculate(ArmConstants.elevatorDesiredPos))
        }

        if(elbowEncoder.position < ArmConstants.elbowThreshold) {
            elbowMotor.setVoltage((ArmConstants.elbowPid.calculate(elevatorEncoder.position, ArmConstants.elbowDesiredPos)*ArmConstants.elbowSpeedMultiplier)+ArmConstants.elevatorfeedforward.calculate(ArmConstants.elbowDesiredPos))
        }

        //This part is the network table bit
        elbowPos.set(elbowEncoder.position)
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