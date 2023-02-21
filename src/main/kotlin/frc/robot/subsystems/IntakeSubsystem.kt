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
import frc.robot.constants.IntakeConstants

/** Subsystem for the intake.
 * @param intakeMotorLID The CAN id for the left intake motor.
 * @param intakeMotorRID The CAN id for the right intake motor.
 * @param pitchMotorID The CAN id for the pitch motor.
 *
 * Warning: This subsystem has not been tested and/or tuned.
 * **/
class IntakeSubsystem(intakeMotorLID: Int,
                      intakeMotorRID: Int,
                      pitchMotorID: Int,
) : SubsystemBase() {

    val intakeMotorR = CANSparkMax(intakeMotorRID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeMotorL = CANSparkMax(intakeMotorLID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val pitchMotor = CANSparkMax(pitchMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    var intakeMotors = MotorControllerGroup(intakeMotorL, intakeMotorR)

    val pitchEncoder = pitchMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val elbowPos = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()

    // Telemetry
    // the Ts stand for telemetry, this is horrible, I know.
    val TabsoluteIntakePosition = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("RelativeIntakePosition").publish()
    val TrelativeIntakePosition = NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("AbsoluteIntakePosition").publish()
    val TintakeVoltage =  NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("IntakeVoltage").publish()
    val TpitchVoltage =  NetworkTableInstance.getDefault().getTable("Intake").getDoubleTopic("PitchVoltage").publish()
    init {
        intakeMotorR.inverted = IntakeConstants.intakeMotorRInverted
        intakeMotorL.inverted = IntakeConstants.intakeMotorLInverted
        pitchEncoder.positionConversionFactor = IntakeConstants.pitchEncoderPositionConversionFactor
        pitchEncoder.velocityConversionFactor = IntakeConstants.pitchEncoderVelocityConversionFactor

        intakeMotorR.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        intakeMotorL.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        pitchMotor.setSmartCurrentLimit(IntakeConstants.pitchMotorCurrentLimit)

        intakeMotorR.burnFlash()
        intakeMotorL.burnFlash()
        pitchMotor.burnFlash()
    }

    /*// radian pitch input
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
*/

    /** This is the pitch without taking consideration to the position of the elbow.
     * 0 degrees means the intake is parallel to the rest of the arm. **/
    val relativePitch get() = 2.0

    /** This is the pitch with taking consideration to the position of the elbow.
     * 0 degrees means the intake is parallel to the ground.
      */
    val absolutePitch get() = 2.0

    /** This value sets the voltage for the intake motors. Positive value will spin the wheels inward
     * and pick objects up.
     */
    var intakeVoltage = 0.0
        set(x: Double) {
            field = x
            intakeMotors.setVoltage(x)
        }

    /** This value sets the voltage for the pitch motors. Positive values will tilt the intake up,
     * negative values will tilt the intake down.
     */
    var pitchVoltage = 0.0
        set(x: Double) {
            field = x
            pitchMotor.setVoltage(x)
        }
}