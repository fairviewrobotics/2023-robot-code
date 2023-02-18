package frc.robot.commands

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.LimelightHelpers
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveSubsystem

class GoToAprilTag(driveSubsystem: SwerveSubsystem, limelight: LimelightSubsystem): CommandBase() {
/*    val targetpose = limelight.getTargetPose()
    val trajectory: PathPlannerTrajectory

    override fun execute() {
        super.execute()
    }



    println("Hi")

//    limelight.setEntry("pipeline", 0)
    val current = PathPoint(driveSubsystem.pose.translation, driveSubsystem.pose.rotation)
    val target =
        PathPoint(Translation2d(targetpose[0], targetpose[2]-1), Rotation2d(targetpose[3], targetpose[5]))

    trajectory = PathPlanner.generatePath(PathConstraints(5.0, 1.0), current, target)

    return driveUtils.trajectoryDrivePathPlanner(trajectory, false)*/

}
class LineUpHorizontal(val driveSubsystem: SwerveSubsystem, val limelight: LimelightSubsystem) : CommandBase() {
    val lineupPID = PIDController(DrivetrainConstants.lineupP, DrivetrainConstants.lineupI, DrivetrainConstants.lineupD)

    init {
        lineupPID.setTolerance(1.0)
    }

    override fun execute() {
        val out = lineupPID.calculate(limelight.getAprilTagOffset(), 0.0)

        driveSubsystem.drive(0.0, out, 0.0, false, false)
    }

    override fun isFinished(): Boolean {
        return lineupPID.atSetpoint()
    }
}


