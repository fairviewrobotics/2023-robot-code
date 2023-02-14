package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase

class IntakeSubsystem(intakeMotorLID: Int,
                      intakeMotorRID: Int,
                      pitchMotorID: Int,
) : SubsystemBase() {

    val intakeMotorR = CANSparkMax(intakeMotorRID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeMotorL = CANSparkMax(intakeMotorLID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val pitchMotor = CANSparkMax(pitchMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    var intakeMotors = MotorControllerGroup(intakeMotorL, intakeMotorR)

    val pitchEncoder = pitchMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    //var elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPositions")
    // var desiredIntakePitch = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("IntakePositions").publish()
    var desiredIntakeSpeed = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("IntakePositions").publish()
    var desiredIntakePitch = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("IntakePositions").publish()
    var intakePosition = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("IntakePositions").publish()



    val pitchFFS = 0.0
    val pitchFFG = 0.0
    val pitchFFV = 0.0
    val pitchFFA = 0.0
    val FFVelocity = 0.0

    var intakeFeedforward = ArmFeedforward(pitchFFS,pitchFFG, pitchFFV, pitchFFA)

    val pitchP = 0.0
    val pitchI = 0.0
    val pitchD = 0.0

    val intakePID = PIDController(pitchP,pitchI,pitchD)

    val pitchEncoderPositionConversionFactor = 0.0
    val pitchEncoderVelocityConversionFactor = 0.0

    // limits for the position of the intake pitch
    val maxIntakePosition = 2*Math.PI
    val minIntakePosition = Math.PI

    var targetIntakeSpeed = 0.0
    var targetIntagePitch = 0.0

    // when elbow pointing down --> 3pi/2
    // when intake perpendicular to elbow --> 3pi/2
    // offsets in revolutions
    val elbowPosOffset = 0.0
    val intakePosOffset = 0.0

    //------------------------------------------------------------------------------------------------------------------------------------------------

    init {
        intakeMotorR.inverted = true
        pitchEncoder.positionConversionFactor = pitchEncoderPositionConversionFactor
        pitchEncoder.velocityConversionFactor = pitchEncoderVelocityConversionFactor
    }

    // radian pitch input
    // targetPitch between pi and 2pi
    fun setTargets(intakeSpeed: Double, intakePitch: Double){
        targetIntakeSpeed = intakeSpeed
        if (intakePitch <= minIntakePosition){
            targetIntagePitch = minIntakePosition
        }

        else if (intakePitch >= maxIntakePosition){
            targetIntagePitch = maxIntakePosition
        }
        else{
            targetIntagePitch = (intakePitch + intakePosOffset)/(2*Math.PI)
        }
    }

    //returns revs
    fun getLevelPitch(elbowPos: Double) : Double{

        // convert revs to radians and add radian offset
        val calcElbowPos = (elbowPos+elbowPosOffset)*(2*Math.PI)

        // pitch of intake to be parallel with the ground
        val levelPitch = (3*Math.PI) - calcElbowPos

        return levelPitch/(2*Math.PI) - intakePosOffset
    }

    override fun periodic() {
        intakeMotors.set(targetIntakeSpeed)
        pitchMotor.setVoltage(intakePID.calculate(pitchEncoder.position, targetIntagePitch) + intakeFeedforward.calculate(targetIntagePitch, FFVelocity))

        intakePosition.set(pitchEncoder.position)
        desiredIntakePitch.set(targetIntagePitch)
        desiredIntakeSpeed.set(targetIntakeSpeed)
    }

}