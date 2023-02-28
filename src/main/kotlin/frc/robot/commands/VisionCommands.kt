package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import frc.robot.VisionUtils
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem
import com.fasterxml.jackson.databind.ext.SqlBlobSerializer

class GoToAprilTag(driveSubsystem: SwerveSubsystem): CommandBase() {
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

fun SetPipeline(pipeline: VisionConstants.Pipelines) : Command {
    return RunCommand({
        VisionUtils.setPipelineIndex("", pipeline.ordinal)
    })
}
class RumbleCheck(val controller: XboxController, val check: () -> Boolean): CommandBase() {
  override fun execute(){
    controller.setRumble(RumbleType.kBothRumble, (if (check()) 1.0 else 0.0));
  }
  override fun isFinished(): Boolean {
    return !check()
  }
}

class Align(val driveSubsystem: SwerveSubsystem) : CommandBase() {
    val lineupPID = ProfiledPIDController(VisionConstants.lineupP, VisionConstants.lineupI, VisionConstants.lineupD, TrapezoidProfile.Constraints(1.0, 100.0))

    init {
        lineupPID.setTolerance(1.0)
    }

    override fun execute() {
        val out = lineupPID.calculate(VisionUtils.getTX("") - VisionConstants.alignmentTxOffset, 0.0)

        driveSubsystem.drive(0.0, out, 0.0, false, false)
    }

    override fun isFinished(): Boolean {
        return !VisionUtils.getTV("") || lineupPID.atSetpoint()
    }
}

fun AlignToAprilTag(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup{
  return(SequentialCommandGroup(
    SetPipeline(VisionConstants.Pipelines.APRILTAG),
    RumbleCheck(controller, {VisionUtils.getTV("")}),
    Align(driveSubsystem)
  ))
}
fun AlignToCone(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup{
  return(SequentialCommandGroup(
    SetPipeline(VisionConstants.Pipelines.CONE),
    RumbleCheck(controller, {VisionUtils.getTV("")}),
    Align(driveSubsystem)
  ))
}
fun AlignToRetroreflective(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup{
  return(SequentialCommandGroup(
    SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
    RumbleCheck(controller, {VisionUtils.getTV("")}),
    Align(driveSubsystem)
  ))
}

