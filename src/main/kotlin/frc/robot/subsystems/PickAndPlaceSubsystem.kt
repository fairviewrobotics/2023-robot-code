package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.SetPickAndPlacePosition
import frc.robot.constants.ArmConstants
import frc.robot.constants.IntakeConstants

class PickAndPlaceSubsystem() : SubsystemBase(){

    val elevatorMotor = CANSparkMax(ArmConstants.elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(ArmConstants.elbowMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val wristMotor = CANSparkMax(ArmConstants.wristMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeOneMotor = CANSparkMax(ArmConstants.intakeMotorOneId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeTwoMotor = CANSparkMax(ArmConstants.intakeMotorTwoId, CANSparkMaxLowLevel.MotorType.kBrushless)

    val elevatorEncoder = elevatorMotor.encoder
    val sprocketEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val axleEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    val forwardLimit = DigitalInput(ArmConstants.topBreakerId)
    val reverseLimit = DigitalInput(ArmConstants.bottomBreakerId)


    var elevatorZeroed = false

    //Is the carriage at the top of the elevator?

    val topHit get() = !forwardLimit.get()

    // Is the carriage at the bottom of the elevator?
    val bottomHit get() = !reverseLimit.get()

    object Telemetry {
        var elbowPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
        var elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVelocity").publish()
        var elevatorPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPosition").publish()
        var elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVelocity").publish()

        val elevatorVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVoltage").publish()
        val bottomHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("BottomHit").publish()
        val topHit = NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("TopHit").publish()
        val wristPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPosition").publish()
        val wristVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristVoltage").publish()
        val elbowVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVolts").publish()
        val intakeVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("IntakeVolts").publish()

        val elevatorZeroed =  NetworkTableInstance.getDefault().getTable("Arm").getBooleanTopic("ElevatorZeroed").publish()

        var table = NetworkTableInstance.getDefault().getTable("NtPnP positions")

        var elevatorValue = table.getDoubleTopic("Elevator").subscribe(0.0)
        var elbowValue = table.getDoubleTopic("Elbow").subscribe(0.0)
        var wristValue = table.getDoubleTopic("Wrist").subscribe(0.0)

    }


    init {
        //intake
        elbowMotor.setSmartCurrentLimit(38)
        wristMotor.setSmartCurrentLimit(38)
        elbowMotor.inverted = ArmConstants.elbowMotorInverted
        wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
        //intakeOneMotor.idleMode = CANSparkMax.IdleMode.kBrake
        //intakeTwoMotor.idleMode = CANSparkMax.IdleMode.kBrake
        intakeTwoMotor.inverted = false
        elbowMotor.idleMode = CANSparkMax.IdleMode.kBrake
        elevatorMotor.idleMode = CANSparkMax.IdleMode.kBrake

        //intakeOneMotor.inverted = IntakeConstants.intakeMotorRInverted
        //intakeTwoMotor.inverted = IntakeConstants.intakeMotorLInverted


        //intakeOneMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        //intakeTwoMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorsCurrentLimit)
        wristMotor.setSmartCurrentLimit(IntakeConstants.pitchMotorCurrentLimit)

        //intakeOneMotor.burnFlash()
        //intakeTwoMotor.burnFlash()
        wristMotor.burnFlash()
        //everything else:
        // TODO: Elevator conversion factors have been tuned
        elbowEncoder.positionConversionFactor = 2.0 * Math.PI
        elbowEncoder.velocityConversionFactor = (2.0 *Math.PI)/60

        elevatorEncoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        elevatorEncoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityConversionFactor

        wristEncoder.positionConversionFactor = 2.0 * Math.PI
        wristEncoder.velocityConversionFactor = (2.0 * Math.PI)/ 60.0
        // TODO: wristEncoder position and velocity conversion factor
    }


    val elbowPositionFirst get() = Rotation2d(elbowEncoder.position).plus(Rotation2d(absoluteWristPosition))

    val elbowPositionRadians get() = elbowPositionFirst.minus(Rotation2d(ArmConstants.elbowEncoderPosOffset)).radians
    val elevatorPositionMeters get() = elevatorEncoder.position

    //intake
    val absoluteWristPosition get() = Rotation2d(wristEncoder.position).minus(Rotation2d(ArmConstants.wristEncoderPosOffset)).radians


    /** This is the pitch with taking consideration to the position of the elbow.
     * 0 degrees means the intake is parallel to the ground.
     */
    //val relativeWristPosition get() = Rotation2d(wristEncoder.position).minus(Rotation2d(ArmConstants.wristEncoderPosOffset)).radians

    /** This value sets the voltage for the intake motors. Positive value will spin the wheels inward
     * and pick objects up.
     */
    var intakesVoltage = 0.0
        set(x: Double) {
            field = x
            intakeOneMotor.setVoltage(x)
            intakeTwoMotor.setVoltage(x)
        }

    var wristVoltage = 0.0
        set(x: Double) {
            field = x
            wristMotor.setVoltage(x)
        }

    var elbowVoltage = 0.0
        set(x: Double) {
            field = x
            elbowMotor.setVoltage(x)
        }

    var elevatorVoltage = 0.0
        set(x: Double) {
            field = x
            Telemetry.elevatorVoltage.set(elevatorVoltage)
            if (!elevatorZeroed) {
                elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage)
            } else if (bottomHit && x < 0.0) {
                elevatorMotor.setVoltage(0.0)
            } else if (topHit && x > 0.0) {
                elevatorMotor.setVoltage(0.0)
            } else {
                elevatorMotor.setVoltage(x)
            }
        }

    override fun periodic() {
        super.periodic()
        if (!elevatorZeroed) {
            elevatorMotor.setVoltage(ArmConstants.elevatorZeroingVoltage)
        }
        // These are the voltages we set and will be sent to the elevator and elbow.
        if(bottomHit)
        {
            elevatorEncoder.position = ArmConstants.elevatorMinHeight
            elevatorZeroed = true
        }
        else if(topHit)
        {
            elevatorEncoder.position = ArmConstants.elevatorMaxHeight
        }

        // Telemetry setting.
        Telemetry.elevatorZeroed.set(elevatorZeroed)
        Telemetry.elbowPosition.set(elbowPositionRadians)
        Telemetry.elevatorPosition.set(elevatorEncoder.position)
        Telemetry.elbowVelocity.set(sprocketEncoder.velocity)
        Telemetry.topHit.set(topHit)
        Telemetry.bottomHit.set(bottomHit)
        Telemetry.wristVoltage.set(wristVoltage)
        Telemetry.elbowVoltage.set(elbowVoltage)

        Telemetry.intakeVoltage.set(intakesVoltage)
        Telemetry.elevatorPosition.set(elevatorPositionMeters)
        Telemetry.elevatorVelocity.set(elevatorEncoder.velocity)

    }
//    fun NTPnP(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
//
//        return SetPickAndPlacePosition(
//            true,
//            pnp,
//            {
//                Telemetry.elevatorValue.get()
//            }, // elevator
//            { Telemetry.elbowValue.get() }, // elbow
//            { Telemetry.wristValue.get() }, // wrist
//            { controller.leftTriggerAxis * 12.0 } // intake
//        )
//    }

}