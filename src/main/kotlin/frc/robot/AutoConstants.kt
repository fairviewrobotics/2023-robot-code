package frc.robot

import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units


object AutoConstants {

    val kMaxSpeedMetersPerSecond:Double = 3.0
    val kMaxAccelerationMetersPerSecondSquared:Double = 3.0
    val kMaxAngularSpeedRadiansPerSecond:Double = Math.PI
    val kMaxAngularSpeedRadiansPerSecondSquared:Double = Math.PI

    val kPXController:Double = 1.0
    val kPYController:Double = 1.0
    val kPThetaController:Double = 1.0

    // Constraint for the motion profiled robot angle controller
    var kThetaControllerConstraints:TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )
}


