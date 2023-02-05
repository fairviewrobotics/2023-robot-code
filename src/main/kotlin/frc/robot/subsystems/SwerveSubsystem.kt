package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.DrivetrainConstants
import frc.robot.RobotContainer
import frc.robot.controllers.SwerveModuleControlller
import kotlin.math.IEEErem


class SwerveSubsystem() : SubsystemBase() {
    val frontLeft = SwerveModuleControlller(
        DrivetrainConstants.frontLeftDrivingPort,
        DrivetrainConstants.frontLeftTurningPort,
        DrivetrainConstants.frontLeftChassisAngularOffset
    )

    val frontRight = SwerveModuleControlller(
        DrivetrainConstants.frontRightDrivingPort,
        DrivetrainConstants.frontRightTurningPort,
        DrivetrainConstants.frontRightChassisAngularOffset
    )

    val rearLeft = SwerveModuleControlller(
        DrivetrainConstants.rearLeftDrivingPort,
        DrivetrainConstants.rearLeftTurningPort,
        DrivetrainConstants.rearLeftChassisAngularOffset
    )

    val rearRight = SwerveModuleControlller(
        DrivetrainConstants.rearRightDrivingPort,
        DrivetrainConstants.rearRightTurningPort,
        DrivetrainConstants.rearRightChassisAngularOffset
    )

    val gyro = AHRS()

    val heading: Double get() = (Units.degreesToRadians(gyro.angle.IEEErem(360.0))) * -1

    //val headingInDegrees = Units.radiansToDegrees(heading)



    val odometry = SwerveDriveOdometry(
        DrivetrainConstants.driveKinematics,
        Rotation2d.fromRadians(heading),
        arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position)
    )

    val setpointsTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Setpoints").getEntry(doubleArrayOf(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
    val actualTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Actual").getEntry(doubleArrayOf(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
    val poseTelemetry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Pose").getEntry(doubleArrayOf(pose.x, pose.y, pose.rotation.radians))
    val gyroHeading = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("GyroHeading").getEntry((heading))

    override fun periodic() {
        odometry.update(
            Rotation2d.fromRadians(heading),
            arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position)
        )

        actualTelemetry.set(doubleArrayOf(
            frontLeft.position.angle.radians, frontLeft.state.speedMetersPerSecond,
            frontRight.position.angle.radians, frontRight.state.speedMetersPerSecond,
            rearLeft.position.angle.radians, rearLeft.state.speedMetersPerSecond,
            rearRight.position.angle.radians, rearRight.state.speedMetersPerSecond))


        setpointsTelemetry.set(doubleArrayOf(
            frontLeft.m_desiredState.angle.radians, frontLeft.m_desiredState.speedMetersPerSecond,
            frontRight.m_desiredState.angle.radians, frontRight.m_desiredState.speedMetersPerSecond,
            rearLeft.m_desiredState.angle.radians, rearLeft.m_desiredState.speedMetersPerSecond,
            rearRight.m_desiredState.angle.radians, rearRight.m_desiredState.speedMetersPerSecond))

        poseTelemetry.set(doubleArrayOf(pose.x, pose.y, pose.rotation.radians))

        gyroHeading.set(heading)



    }

    val pose: Pose2d get() = odometry.poseMeters
    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromRadians(heading),
            arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position),
            pose
        )
    }

    fun drive(forwardMetersPerSecond: Double, sidewaysMetersPerSecond: Double, radiansPerSecond: Double, fieldRelative: Boolean) {

        val radiansDesired = NetworkTableInstance.getDefault()
        val swerveModuleStates = if (fieldRelative) {
            DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardMetersPerSecond,
                    sidewaysMetersPerSecond,
                    radiansPerSecond,
                    Rotation2d.fromRadians(heading)))
        } else {
            DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                ChassisSpeeds(
                    forwardMetersPerSecond,
                    sidewaysMetersPerSecond,
                    radiansPerSecond))
        }



        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeedMetersPerSecond)

        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        rearLeft.setDesiredState(swerveModuleStates[2])
        rearRight.setDesiredState(swerveModuleStates[3])
    }



    fun setX() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
    }

    fun setZero() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
    }

    fun zeroGyro() {
        gyro.reset()
        resetOdometry(Pose2d(0.0, 0.0, Rotation2d(0.0)))
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeedMetersPerSecond)

        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        rearLeft.setDesiredState(desiredStates[2])
        rearRight.setDesiredState(desiredStates[3])
    }

    fun resetEncoders() {
        frontLeft.resetEncoders()
        frontRight.resetEncoders()
        rearLeft.resetEncoders()
        rearRight.resetEncoders()
    }


    fun turnRate(): Double {
        val coefficient = if (DrivetrainConstants.gyroReversed) {
            -1.0
        } else {
            1.0
        }
        return gyro.rate * coefficient
    }

}