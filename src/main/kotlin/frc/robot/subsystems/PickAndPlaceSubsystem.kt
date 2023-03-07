package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.IntakeConstants

class PickAndPlaceSubsystem(elevatorMotorId : Int,
                            elbowMotorId : Int,
                            wristMotorId : Int,
                            intakeMotorOneId : Int,
                            intakeMotorTwoId: Int,
                            topBreakerID: Int,
                            bottomBreakerID: Int) : SubsystemBase(){

    val elevatorMotor = CANSparkMax(elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(elbowMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val wristMotor = CANSparkMax(wristMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeOneMotor = CANSparkMax(intakeMotorOneId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeTwoMotor = CANSparkMax(intakeMotorTwoId, CANSparkMaxLowLevel.MotorType.kBrushless)

    val elevatorEncoder = elevatorMotor.encoder
    val sprocketEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val axleEncoder = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    val forwardLimit = DigitalInput(topBreakerID);
    val reverseLimit = DigitalInput(bottomBreakerID);


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

    }


    init {
        //intake
        elbowMotor.setSmartCurrentLimit(20)
        wristMotor.setSmartCurrentLimit(20)
        elbowMotor.inverted = ArmConstants.elbowMotorInverted
        wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
        //intakeOneMotor.idleMode = CANSparkMax.IdleMode.kBrake
        //intakeTwoMotor.idleMode = CANSparkMax.IdleMode.kBrake
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
        // TODO: Elevator conversion factors have been tuned, but the elbow conversion factors have not.
        sprocketEncoder.positionConversionFactor = 2.0 * Math.PI
        sprocketEncoder.velocityConversionFactor = 2.0 * Math.PI / 60.0

        elevatorEncoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        elevatorEncoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityConversionFactor

        axleEncoder.positionConversionFactor = 2.0 *Math.PI
        axleEncoder.velocityConversionFactor = 2.0 *Math.PI / 60.0
        // TODO: wristEncoder position and velocity conversion factor
    }

    val elbowPositionRadians get() = 1.0
    val elevatorPositionMeters get() = elevatorEncoder.position

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
            if (!elevatorZeroed) {
                elevatorMotor.setVoltage(-3.0)
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
            elevatorMotor.setVoltage(-3.0)
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
        Telemetry.elevatorVoltage.set(elevatorVoltage)
        Telemetry.intakeVoltage.set(intakesVoltage)
        Telemetry.elevatorPosition.set(elevatorPositionMeters)
        Telemetry.elevatorVelocity.set(elevatorEncoder.velocity)
    }

}