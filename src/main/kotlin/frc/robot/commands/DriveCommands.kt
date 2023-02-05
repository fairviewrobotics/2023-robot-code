package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.robot.AutoConstants
import frc.robot.DrivetrainConstants
import frc.robot.subsystems.SwerveSubsystem

class StandardDrive(val swervesubsystem: SwerveSubsystem, val forward: () -> Double, val sideways: () -> Double, val radians: () -> Double, val fieldRelative: Boolean) : CommandBase() {
    var radianLimiter: SlewRateLimiter = SlewRateLimiter(1.0)
    init {
        addRequirements(swervesubsystem)
    }

    override fun execute() {
        val forwardDesired = MathUtil.applyDeadband(forward(), 0.06)
        val sidewaysDesired = MathUtil.applyDeadband(sideways(), 0.06)
        val radiansDesired = radianLimiter.calculate(MathUtil.applyDeadband(radians(), 0.06))

        swervesubsystem.drive(forwardDesired, sidewaysDesired, radiansDesired, fieldRelative)
    }

    override fun end(interrupted: Boolean) {
        //swervesubsystem.drive(0.0,0.0,0.0,true)

    }
}

class UnlimitedDrive(val swervesubsystem: SwerveSubsystem, val forward: () -> Double, val sideways: () -> Double, val radians: () -> Double, val fieldRelative: Boolean) : CommandBase() {

    init {
        addRequirements(swervesubsystem)
    }

    override fun execute() {
        val forwardDesired = MathUtil.applyDeadband(forward(), 0.06)
        val sidewaysDesired = MathUtil.applyDeadband(sideways(), 0.06)
        val radiansDesired = MathUtil.applyDeadband(radians(), 0.06)

        swervesubsystem.drive(forwardDesired, sidewaysDesired, -1 * radiansDesired, fieldRelative)
    }

    override fun end(interrupted: Boolean) {
        swervesubsystem.drive(0.0,0.0,0.0,true)
    }
}

class HolonomicDrive(val swervesubsystem: SwerveSubsystem, val trajectory: Trajectory): CommandBase() {
    init {
        addRequirements(swervesubsystem)
    }

    override fun execute() {
        var thetaController = ProfiledPIDController(
            AutoConstants.kPThetaController, 0.0, 0.0, AutoConstants.kThetaControllerConstraints
        )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        var swerveControllerCommand:SwerveControllerCommand = SwerveControllerCommand(
            trajectory,
            swervesubsystem::pose,
            DrivetrainConstants.driveKinematics,

            // Position controllers
            PIDController(AutoConstants.kPXController, 0.0, 0.0),
            PIDController(AutoConstants.kPYController, 0.0, 0.0),
            thetaController,
            swervesubsystem::setModuleStates,
            swervesubsystem
        )

        // Reset odometry to the starting pose of the trajectory.
        swervesubsystem.resetOdometry(trajectory.initialPose)

        // Run path following command, then stop at the end.
        swerveControllerCommand.andThen(
            RunCommand({
                swervesubsystem.drive(0.0,0.0,0.0,true)
            }, swervesubsystem)
        )

    }

    override fun end(interrupted: Boolean) {
        swervesubsystem.drive(0.0,0.0,0.0,true)
    }




}