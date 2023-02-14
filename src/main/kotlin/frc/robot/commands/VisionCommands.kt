package frc.robot.commands

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveSubsystem

class GoToAprilTag(driveSubsystem: SwerveSubsystem, limelight: LimelightSubsystem): CommandBase() {
    val targetpose = limelight.getTargetPose()


    val trajectory: PathPlannerTrajectory

    println("Hi")

//    limelight.setEntry("pipeline", 0)
    val current = PathPoint(driveSubsystem.pose.translation, driveSubsystem.pose.rotation)
    val target =
        PathPoint(Translation2d(targetpose[0], targetpose[2]-1), Rotation2d(targetpose[3], targetpose[5]))

    trajectory = PathPlanner.generatePath(PathConstraints(5.0, 1.0), current, target)

    return driveUtils.trajectoryDrivePathPlanner(trajectory, false)

}

fun LineUpHorizontal(driveSubsystem: SwerveSubsystem, limelight: LimelightSubsystem) : Command {
    val lineupPID = PIDController(DrivetrainConstants.lineupP, DrivetrainConstants.lineupI, DrivetrainConstants.lineupD)
    lineupPID.setTolerance(1.4)

    if (limelight.canSeeTarget()) {
        val out = lineupPID.calculate(limelight.getAprilTagOffseta(), 0.0) * 10
        val mps = clamp(out, -5.0, 5.0)

        return UnlimitedDrive(driveSubsystem, { 0.0 }, { mps }, { 0.0 }, false)

    }
}


