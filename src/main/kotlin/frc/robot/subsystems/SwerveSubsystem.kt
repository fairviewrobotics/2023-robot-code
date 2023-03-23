package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.DoubleArrayEntry
import edu.wpi.first.networktables.DoubleEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.VisionUtils
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.VisionConstants
import frc.robot.controllers.SwerveModuleControlller
import frc.robot.utils.NetworkTableUtils
import frc.robot.utils.SwerveUtils
import kotlin.math.*


class SwerveSubsystem: SubsystemBase() {
    //Defining Motors
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

    //Gyro
    val gyro = AHRS()

    //Slew Rate Constants
    var currentRotation = 0.0
    var currentTranslationDirection = 0.0
    var currentTranslationMagnitude = 0.0

    //Slew Rate Limiters
    val magnitudeLimiter = SlewRateLimiter(DrivetrainConstants.magnitudeSlewRate)
    val rotationLimiter = SlewRateLimiter(DrivetrainConstants.rotationalSlewRate)

    //Slew Rate Time
    var previousTime = WPIUtilJNI.now() * 1e-6

    //Limelight Network Table
    val limelightTable = NetworkTableUtils("limelight")

    //Convert Gyro angle to radians(-2pi to 2pi)
    val heading: Double get() = (Units.degreesToRadians(-1 * (gyro.angle + 180.0).IEEErem(360.0)))

    //Swerve Odometry
    val odometry = SwerveDriveOdometry(
        DrivetrainConstants.driveKinematics,
        Rotation2d.fromRadians(heading),
        arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position)
    )

    //Network Tables Telemetry
    val setpointsTelemetry: DoubleArrayEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Setpoints").getEntry(doubleArrayOf(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
    val actualTelemetry: DoubleArrayEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Actual").getEntry(doubleArrayOf(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
    val poseTelemetry: DoubleArrayEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleArrayTopic("Pose").getEntry(doubleArrayOf(pose.x, pose.y, pose.rotation.radians))
    val gyroHeading: DoubleEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("GyroHeading").getEntry((heading))
    val frontrightpos: DoubleEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("frpos").getEntry((frontRight.position.angle.radians))
    val frontleftpos: DoubleEntry = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("flpos").getEntry((frontLeft.position.angle.radians))

    //Periodic
    override fun periodic() {
        //Update odometry
        odometry.update(
            Rotation2d.fromRadians(heading),
            arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position)
        )

        //Coen's Vision Lineup Thing:
        // find the botpose network table id thingy, construct a pose2d, feed it into resetodometry
//        val botpose: DoubleArray = limelightTable.getDoubleArray("botpose", DoubleArray(0))
//        if (!botpose.contentEquals(DoubleArray(0))) {
//            val pose = Pose2d(Translation2d(botpose[0], botpose[2]), Rotation2d(botpose[3], botpose[5]))
//            resetOdometry(pose)
//        }

        frontrightpos.set(frontRight.position.angle.radians)
        frontleftpos.set(frontLeft.position.angle.radians)

        //Set Network Tables Telemetry
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

    //Define robot pose
    val pose: Pose2d get() = odometry.poseMeters * 1.0

    //Reset odometry function
    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromRadians(heading),
            arrayOf(frontLeft.position, frontRight.position, rearLeft.position, rearRight.position),
            pose
        )
    }

    //Drive function - slew rate limited to prevent shearing of wheels
    fun drive(forwardMetersPerSecond: Double, sidewaysMetersPerSecond: Double, radiansPerSecond: Double, fieldRelative: Boolean, rateLimit: Boolean) {

        // forward is xspeed, sideways is yspeed
        var xSpeedCommanded: Double
        var ySpeedCommanded: Double

        if (rateLimit) {
            val inputTranslationDirection = atan2(sidewaysMetersPerSecond, forwardMetersPerSecond)
            val inputTranslationMagnitude = sqrt(forwardMetersPerSecond.pow(2.0) + sidewaysMetersPerSecond.pow(2.0))

            var directionSlewRate: Double
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = abs(DrivetrainConstants.directionSlewRate / currentTranslationMagnitude)
            } else {
                directionSlewRate = 500.0 // super high number means slew is instantaneous
            }

            val currentTime = WPIUtilJNI.now() * 1e-6
            val elapsedTime = currentTime - previousTime

            val angleDifference =
                SwerveUtils.AngleDifference(inputTranslationDirection, currentTranslationDirection)
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                )
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude)
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) { // small number avoids floating-point errors
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0)
                } else {
                    currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI)
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude)
                }
            } else {
                currentTranslationDirection = SwerveUtils.StepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                )
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude)
            }

            previousTime = currentTime

            xSpeedCommanded = currentTranslationMagnitude * cos(currentTranslationDirection)
            ySpeedCommanded = currentTranslationMagnitude * sin(currentTranslationDirection)
            currentRotation = rotationLimiter.calculate(radiansPerSecond)
        } else {
            xSpeedCommanded = forwardMetersPerSecond
            ySpeedCommanded = sidewaysMetersPerSecond
            currentRotation = radiansPerSecond
        }

        val xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond
        val ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.maxSpeedMetersPerSecond
        val rotationDelivered = currentRotation * DrivetrainConstants.maxAngularSpeed


        val swerveModuleStates = if (fieldRelative) {
            DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotationDelivered,
                    Rotation2d.fromRadians(heading)
                )
            )
        } else {
            DrivetrainConstants.driveKinematics.toSwerveModuleStates(
                ChassisSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotationDelivered
                )
            )
        }



        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeedMetersPerSecond)

        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        rearLeft.setDesiredState(swerveModuleStates[2])
        rearRight.setDesiredState(swerveModuleStates[3])


    }


    //Sets the wheels to an X configuration
    fun setX() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)))
    }

    //Sets the wheels to a zeroed configuration
    fun setZero() {
        frontLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        frontRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        rearLeft.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
        rearRight.setDesiredState(SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)))
    }

    //Resets Gyro
    fun zeroGyro() {
        gyro.reset()
    }



    //Resets Gyro and odometry
    fun zeroGyroAndOdometry() {
        gyro.reset()
        resetOdometry(Pose2d(0.0, 0.0, Rotation2d(0.0)))
    }

    //Sets states of swerve modules
    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeedMetersPerSecond)

        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        rearLeft.setDesiredState(desiredStates[2])
        rearRight.setDesiredState(desiredStates[3])
    }

    //Resets Swerve encoders
    fun resetEncoders() {
        frontLeft.resetEncoders()
        frontRight.resetEncoders()
        rearLeft.resetEncoders()
        rearRight.resetEncoders()
    }

    //Returns turn rate of the robot
    fun turnRate(): Double {
        val coefficient = if (DrivetrainConstants.gyroReversed) {
            -1.0
        } else {
            1.0
        }
        return gyro.rate * coefficient
    }

}