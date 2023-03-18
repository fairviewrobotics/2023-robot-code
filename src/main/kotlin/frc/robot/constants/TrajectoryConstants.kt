package frc.robot.constants

import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrapezoidProfile


object TrajectoryConstants {

    val kMaxSpeedMetersPerSecond:Double = 1.5
    val kMaxAccelerationMetersPerSecondSquared:Double = 1.5
    val kMaxAngularSpeedRadiansPerSecond:Double = 1000 * Math.PI
    val kMaxAngularSpeedRadiansPerSecondSquared:Double = 1000 * Math.PI

    val kPXController:Double = 0.05
    val kPYController:Double = 0.05
    val kPThetaController:Double = 2.0 // Old values: 0.075
    val kDThetaController:Double = 0.0 // Old values: 0.4

    // Constraint for the motion profiled robot angle controller
    var kThetaControllerConstraints:TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )

    var config: TrajectoryConfig = TrajectoryConfig(
        kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(DrivetrainConstants.driveKinematics)


}


