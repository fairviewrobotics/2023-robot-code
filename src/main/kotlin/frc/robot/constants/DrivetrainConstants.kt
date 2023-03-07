package frc.robot.constants

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance


object DrivetrainConstants {


    const val maxSpeedMetersPerSecond = 2.0
    const val maxAngularSpeed = Math.PI

    const val directionSlewRate = 4.0 // rads/sec   - turning
    const val magnitudeSlewRate = 5.0 // percent/second (1 = 100%)   - forward/backward/traverse
    const val rotationalSlewRate = 12.0 // percent/second (1 = 100%)   - rotation

    val directionSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("DirectionalSlewRate").getEntry((directionSlewRate))
    val magnitudeSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("MagnitudeSlewRate").getEntry((directionSlewRate))
    val rotationSlewNet = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("RotationSlewRate").getEntry((directionSlewRate))



    const val drivingSpeedScalar = -1.0
    const val rotationSpeedScalar = -1.0

    // TUNED
    val trackWidth = Units.inchesToMeters(26.5)
    val wheelBase = Units.inchesToMeters(26.5)

    val driveKinematics = SwerveDriveKinematics(
        Translation2d(wheelBase / 2, trackWidth / 2),
        Translation2d(wheelBase / 2, -trackWidth / 2),
        Translation2d(-wheelBase / 2, trackWidth / 2),
        Translation2d(-wheelBase / 2, -trackWidth / 2)
    )

    // FIXME: Right now 0degrees is in the Y-positive direction, when normally 0 rad is in the X-positive direction. Could this be an issue?
    // TUNED
    const val frontLeftChassisAngularOffset = 5.772
    const val frontRightChassisAngularOffset = 6.09
    const val rearLeftChassisAngularOffset = .871 + (Math.PI)
    const val rearRightChassisAngularOffset =  3.650

    /**
     *     const val frontLeftChassisAngularOffset = 5.772 - (Math.PI/2) -(Math.PI/2)
     *     const val frontRightChassisAngularOffset = 6.09 - (Math.PI)+(0.0)
     *     const val rearLeftChassisAngularOffset = .871
     *     const val rearRightChassisAngularOffset =  3.650 + (Math.PI/2) +(Math.PI/2)
     */
    // SPARK MAX CAN ID
    // TUNED
    const val frontLeftDrivingPort = 5
    const val rearLeftDrivingPort = 1
    const val frontRightDrivingPort = 7
    const val rearRightDrivingPort = 3

    const val frontLeftTurningPort = 6
    const val rearLeftTurningPort = 2
    const val frontRightTurningPort = 8
    const val rearRightTurningPort = 4

    const val gyroReversed = false // TUNED
    const val turningEncoderReversed = true

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    const val drivingMotorPinionTeeth = 13

    // Calculations required for driving motor conversion factors and feed forward.
    const val freeSpeedRpm = 5676.0 // from NEO datasheet, do not tune.
    const val drivingMotorFreeSpeedRps = freeSpeedRpm / 60.0
    const val wheelDiameterMeters = 0.0762
    const val wheelCircumferenceMeters = wheelDiameterMeters * Math.PI
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    const val drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15)
    const val driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)/ drivingMotorReduction

    // Conversion factors (Revs -> Meters, RPM -> M/S).
    const val drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI)/ drivingMotorReduction // meters
    const val drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI) / drivingMotorReduction) / 60.0 // meters per second

    const val turningEncoderPositionFactor = (2 * Math.PI) // radians
    const val turningEncoderVelocityFactor = (2 * Math.PI) / 60.0 // radians per second

    const val turningEncoderPositionPIDMinInput = 0.0 // radians
    const val turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor // radians

    // PIDs for driving and turning. minAndMaxOutputs are for SparkMax -1,1 setting, not voltage. TODO: These values must be tuned.
    const val drivingP = 0.06
    const val drivingI = 0.0
    const val drivingD = 0.0
    const val drivingFF = 1.0 / driveWheelFreeSpeedRps
    const val drivingMinOutput = -1.0
    const val drivingMaxOutput = 1.0

    const val turningP = 0.6
    const val turningI = 0.0
    const val turningD = 0.0
    const val turningFF = 0.0
    const val turningMinOutput = -1.0
    const val turningMaxOutput = 1.0

    // Idle mode for driving and turning motor
    val drivingMotorIdleMode = CANSparkMax.IdleMode.kBrake
    val turningMotorIdleMode = CANSparkMax.IdleMode.kBrake

    // Current limits for motors, set using smartcurrentlimits in swervemodulecontroller
    const val drivingMotorCurrentLimit = 40 // amps
    const val turningMotorCurrentLimit = 20 // amps
}