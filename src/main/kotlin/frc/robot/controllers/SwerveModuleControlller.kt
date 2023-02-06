package frc.robot.controllers

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.constants.DrivetrainConstants

// NOTE: use encoders from cansparkmax or specify port?
// TODO: convert encoder rpm/rev counts to meters per second, degree, etc.
class SwerveModuleControlller(val drivingPort: Int, val turningPort: Int, val chassisAngularOffset: Double) {
    val drivingMotor = CANSparkMax(drivingPort, CANSparkMaxLowLevel.MotorType.kBrushless)
    val turningMotor = CANSparkMax(turningPort, CANSparkMaxLowLevel.MotorType.kBrushless)

    val drivingEncoder = drivingMotor.encoder
    val turningEncoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    val drivingPID = drivingMotor.pidController
    val turningPID = turningMotor.pidController

    var m_desiredState = SwerveModuleState(0.0, Rotation2d())

    init {
        // Reset settings to a known state, in case SparkMAXs were swapped out.
        drivingMotor.restoreFactoryDefaults()
        turningMotor.restoreFactoryDefaults()

        // Measurement for PIDs
        drivingPID.setFeedbackDevice(drivingEncoder)
        turningPID.setFeedbackDevice(turningEncoder)

        // Converting revolutions and RPMs to meters and meters/second.
        drivingEncoder.positionConversionFactor = DrivetrainConstants.drivingEncoderPositionFactor
        drivingEncoder.velocityConversionFactor = DrivetrainConstants.drivingEncoderVelocityFactor

        turningEncoder.positionConversionFactor = DrivetrainConstants.turningEncoderPositionFactor
        turningEncoder.velocityConversionFactor = DrivetrainConstants.turningEncoderVelocityFactor

        // Output shaft rotates in the opposite direction of the steering motor in the module, this we need to invert.
        turningEncoder.inverted = DrivetrainConstants.turningEncoderReversed

        // Enable PID wrap around - going through 0 to get to the setpoint. Going from 350 degrees to 10 degrees will only
        // take 20 degrees of movement as opposed to 340.
        turningPID.positionPIDWrappingEnabled = true
        turningPID.positionPIDWrappingMinInput = DrivetrainConstants.turningEncoderPositionPIDMinInput
        turningPID.positionPIDWrappingMaxInput = DrivetrainConstants.turningEncoderPositionPIDMaxInput

        // Set the PID gains. TODO: Needs to be tuned.
        drivingPID.p = DrivetrainConstants.drivingP
        drivingPID.i = DrivetrainConstants.drivingI
        drivingPID.d = DrivetrainConstants.drivingD
        drivingPID.ff = DrivetrainConstants.drivingFF
        drivingPID.setOutputRange(DrivetrainConstants.drivingMinOutput, DrivetrainConstants.drivingMaxOutput)

        // Set the pid gains. TODO: Needs to be tuned.
        turningPID.p = DrivetrainConstants.turningP
        turningPID.i = DrivetrainConstants.turningI
        turningPID.d = DrivetrainConstants.turningD
        turningPID.ff = DrivetrainConstants.turningFF
        turningPID.setOutputRange(DrivetrainConstants.turningMinOutput, DrivetrainConstants.turningMaxOutput)

        // Idle mode can either be brake (brings motors to quick stop) or coast (motors spin down at own, natural rate)
        drivingMotor.idleMode = DrivetrainConstants.drivingMotorIdleMode
        turningMotor.idleMode = DrivetrainConstants.turningMotorIdleMode

        // Setting a current limit that controllers will try to adjust for. NEOs have low internal resistance,
        // large spikes in voltage can damage motor and controller.
        drivingMotor.setSmartCurrentLimit(DrivetrainConstants.drivingMotorCurrentLimit)
        turningMotor.setSmartCurrentLimit(DrivetrainConstants.turningMotorCurrentLimit)

        // Ensures there will be no wacky turning movement on startup
        m_desiredState.angle = Rotation2d(turningEncoder.position)
        drivingEncoder.position = 0.0;

        // Saves these settings to the motor controller itself, in case of brownout.
        drivingMotor.burnFlash()
        turningMotor.burnFlash()
    }

    /** Returns state (speed and angle) for the module. **/
    val state: SwerveModuleState get() = SwerveModuleState(drivingEncoder.velocity, Rotation2d(turningEncoder.position - chassisAngularOffset))
    /** Returns **/
    val position: SwerveModulePosition get() = SwerveModulePosition(drivingEncoder.position, Rotation2d(turningEncoder.position - chassisAngularOffset))

    /** Sets desired state (speed and angle) for the module. **/
    fun setDesiredState(newState: SwerveModuleState) {
        // Apply chassis angular offset to the directed state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = newState.speedMetersPerSecond
        correctedDesiredState.angle = newState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset))

        // Optimizing helps avoid spinning further than 90 degrees.
        val optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(turningEncoder.position))

        // Setpoints for PIDs
        drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        turningPID.setReference(optimizedDesiredState.angle.radians, CANSparkMax.ControlType.kPosition)

        m_desiredState = newState
    }

    /** Zeroes all the SwerveModule encoders. **/
    fun resetEncoders() {
        drivingEncoder.position = 0.0
    }
}