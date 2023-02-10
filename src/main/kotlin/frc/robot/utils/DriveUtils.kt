package frc.robot.utils

import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.SwerveSubsystem

class DriveUtils(private val swerveSubsystem: SwerveSubsystem) {
    fun trajectoryDrive(trajectory: Trajectory) : Command {
        val thetaController = ProfiledPIDController(
            TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController, TrajectoryConstants.kThetaControllerConstraints
        )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        val thetaControllerError = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("TurningError").getEntry(0.0)

        thetaControllerError.set(thetaController.positionError)

        val swerveControllerCommand = SwerveControllerCommand(
            trajectory,
            this.swerveSubsystem::pose,
            DrivetrainConstants.driveKinematics,

            // Position controllers
            PIDController(TrajectoryConstants.kPXController, 0.0, 0.0),
            PIDController(TrajectoryConstants.kPYController, 0.0, 0.0),
            thetaController,
            this.swerveSubsystem::setModuleStates,
            this.swerveSubsystem
        )

        // Reset odometry to the starting pose of the trajectory.

        println("trajectoryRunning")

        // Run path following command, then stop at the end.
        return ParallelCommandGroup(
            SequentialCommandGroup(
                swerveControllerCommand,
                RunCommand({
                    this.swerveSubsystem.drive(0.0,0.0,0.0,false, false)
                })
            ),

            RunCommand({
                println("-------------------------")
//                println(thetaController.velocityError)
//                println(thetaController.positionError)
//                println(thetaController.setpoint.position)
//                println(thetaController.setpoint.velocity)
//                println(swerveSubsystem.heading)
//            println(swerveSuqbsystem.pose.x)
                println(this.swerveSubsystem.pose.y)
                thetaControllerError.set(thetaController.positionError)

                println("-------------------------")
            })
        )
    }


    fun trajectoryDrivePathPlanner(trajectory: PathPlannerTrajectory, isFirstPath: Boolean) : Command {
        val thetaController = PIDController(
            TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController
        )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        val thetaControllerError = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("TurningError").getEntry(0.0)

        thetaControllerError.set(thetaController.positionError)

        val swerveControllerCommand = PPSwerveControllerCommand(
            trajectory,
            this.swerveSubsystem::pose,
            DrivetrainConstants.driveKinematics,

            // Position controllers
            PIDController(TrajectoryConstants.kPXController, 0.0, 0.0),
            PIDController(TrajectoryConstants.kPYController, 0.0, 0.0),
            thetaController,
            this.swerveSubsystem::setModuleStates,
            true,
            this.swerveSubsystem
        )

        // Reset odometry to the starting pose of the trajectory.

        // Run path following command, then stop at the end.
        return SequentialCommandGroup(
            InstantCommand({
                // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    this.swerveSubsystem.resetOdometry(trajectory.initialHolonomicPose)
                }
            }),
            swerveControllerCommand,
            RunCommand({
                this.swerveSubsystem.drive(0.0,0.0,0.0,
                    true, false)
            })
        )
    }
}