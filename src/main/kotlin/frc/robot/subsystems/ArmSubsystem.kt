package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants

/**
 * The subsystem for the arm. The arm consists of the motor driving the elevator, the motor driving the elbow, and the
 * two linebreakers at the top and bottom of elevator.
 */
class ArmSubsystem(topBreakerID: Int, bottomBreakerID: Int, elbowMotorID: Int, elevatorMotorID: Int) :SubsystemBase() {
    /** The elevator is zeroed if it has reached the bottom of its path (tripping the bottom breaker) at least once.
     * Since the elevator is only controlled through setting a desired position, this is required to ensure positions are
     * correctly reached.
     */
    var elevatorZeroed = false

    // Standard motor, encoder, and hardware declarations.
    val elbowMotor = CANSparkMax(elbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elevatorMotor = CANSparkMax(elevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elevatorEncoder = elevatorMotor.getEncoder()
    val elbowEncoder = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val forwardLimit = DigitalInput(topBreakerID);
    val reverseLimit = DigitalInput(bottomBreakerID);

    /**
     * Used to get the current position of the elevator. This unit is in meters, and should be interpreted as how far
     * along the carriage is on its path. 0.0 means the elevator is at the bottom.
     */
    val elevatorPositionMeters get() = elevatorEncoder.position

    /**
     * Used to get the current position of the elbow. This unit is in radians, and should be interpreted as how rotated
     * the elbow is. 0.0 means the intake is parallel with the carriage. Looking from the right, a positive measurement
     * means the intake is further away from the ground than it is at 0.0, and a negative measurement means the intake
     * is closer to the ground than it is at 0.0.
     */
    val elbowPositionRadians get() = elbowEncoder.position + ArmConstants.elbowEncoderPosOffset


    /**
     * Is the carriage at the top of the elevator?
     */
    val topHit get() = !forwardLimit.get()

    /**
     * Is the carriage at the bottom of the elevator?
     */
    val bottomHit get() = !reverseLimit.get()

    /** Telemetry object to make code more concise and resolve some name conflicts. **/
    object Telemetry {
        var elbowPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPosition").publish()
        var elbowVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVelocity").publish()
        var elevatorPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPosition").publish()
        var elevatorVelocity = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVelocity").publish()
        var desiredElevatorPosition = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElevator").publish()
        var desiredElbowPosition= NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElbow").publish()

        val elevatorVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVoltage").publish()
    }

    /**
     * These are the desired positions for the elevator and elbow, and are set through setDesired().
     */
    var desiredElevatorPositionMeters = 0.0
    var desiredElbowPositionRadians = 0.0

    /**
     * PIDs and feedforward for elevator and elbow.
     */
    val elbowPid = PIDController(ArmConstants.elbowP, ArmConstants.elbowI, ArmConstants.elbowD)
    var elevatorPid = ProfiledPIDController(
        ArmConstants.elevatorP,
        ArmConstants.elevatorI,
        ArmConstants.elevatorD, ArmConstants.elevatorTrapezoidConstraints, 0.02)

    init {
        // TODO: Elevator conversion factors have been tuned, but the elbow conversion factors have not.
        elbowEncoder.positionConversionFactor = ArmConstants.elbowEncoderPosMultiplier
        elbowEncoder.velocityConversionFactor = ArmConstants.elbowEncoderVelocityMultiplier
        elevatorEncoder.positionConversionFactor = ArmConstants.elevatorEncoderPositionConversionFactor
        elevatorEncoder.velocityConversionFactor = ArmConstants.elevatorEncoderVelocityMultiplier

        // TODO: The=is may or may not need to be checked.
        elevatorMotor.inverted = ArmConstants.elevatorMotorInverted
    }
    /**
     * Sets the desired position
     *
     * The periodic function moves forever towards the position, so this is how you make it do something favorable.
     * This basically tells the PID that it needs to go to elevatorPos
     *
     * @param elbowPositionRadians the new desired position for the elbow. 0 radians is parallel with the ground,
     * positive radians will make the intake farther from the ground.
     * @param elevatorPositionMeters the new desired position for the elevator. 0 meters is bottom of the elevator
     */
    fun setDesired(elbowPositionRadians: Double, elevatorPositionMeters: Double) {
        // We use a coerceIn to ensure the desired positions are not set out of the acceptable range. We don't want the
        // arm smashing into the elevator, for example.
        desiredElbowPositionRadians = elbowPositionRadians.coerceIn(ArmConstants.elbowMinRotation, ArmConstants.elbowMaxRotation)
        desiredElevatorPositionMeters = elevatorPositionMeters.coerceIn(ArmConstants.elevatorMinHeight, ArmConstants.elevatorMaxHeight)
    }
    /**
     * This will either move the elevator and elbow to its desired states, or, if the elevator is not zeroed, move the
     * elevator slowly down until it is trips the bottom breaker.
     */
    override fun periodic() {
        super.periodic()
        // These are the voltages we set and will be sent to the elevator and elbow.
        var elevatorVoltage: Double
        var elbowVoltage: Double

        // IF the elevator is not zeroed, just run the elevator slowly and don't mess with the elbow at all.
        if (!elevatorZeroed) {
            elevatorVoltage = -3.0
            elbowVoltage = 0.0

            if (bottomHit) {
                elevatorZeroed = true
            }
        } else {
            // IF the elevator is zeroed, then proceed as normal. Move the elevator and elbow to the desired states.
            elevatorVoltage = elevatorPid.calculate(elevatorPositionMeters, desiredElevatorPositionMeters)
            //elbowVoltage =
            //    elbowPid.calculate(elbowPositionRadians, desiredElbowPositionRadians) + elbowFeedforward.calculate(
            //        desiredElbowPositionRadians,
            //        Math.PI
            //    )
        }

        // Here we do the checks for the bottom and top linebreakers, and constrain voltage, zero elevator, and reset
        // elevator position accordingly.
        if (bottomHit) {
            if (elevatorVoltage < 0.0) {
                elevatorVoltage = 0.0
            }

            elevatorEncoder.position = ArmConstants.elevatorMinHeight
            elevatorZeroed = true
        }

        if (topHit) {
            if (elevatorVoltage > 0.0) {
                elevatorVoltage = 0.0
            }

            elevatorEncoder.position = ArmConstants.elevatorMaxHeight
        }
        elevatorMotor.setVoltage(elevatorVoltage)
        //elbowMotor.setVoltage(elbowVoltage)

        // Telemetry setting.
        Telemetry.elevatorVoltage.set(elevatorVoltage)
        Telemetry.elbowPosition.set(elbowPositionRadians)
        Telemetry.elbowVelocity.set(elbowEncoder.velocity)
        Telemetry.elevatorPosition.set(elevatorPositionMeters)
        Telemetry.elevatorVelocity.set(elevatorEncoder.velocity)
        Telemetry.desiredElevatorPosition.set(desiredElevatorPositionMeters)
        Telemetry.desiredElbowPosition.set(desiredElbowPositionRadians)
    }

}