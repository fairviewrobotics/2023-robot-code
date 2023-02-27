// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.commands.*
import frc.robot.constants.ArmConstants
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.IntakeConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.*
import java.nio.file.Path

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)
    val swerveSubsystem = SwerveSubsystem()
    val subsystem1 = DummySubsystem()
    val subsystem2 = DummySubsystem()

    val traj: PathPlannerTrajectory = PathPlanner.generatePath(
        PathConstraints(12.0,3.5),
        PathPoint(Translation2d(0.0,0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0), 0.0) ,
        PathPoint(Translation2d(-2.0,0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0))

    )




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()


    }
    
    private fun configureButtonBindings() {
        // Current Button Bindings:
        //
        // Primary Controller:
        // Swerve default = drive
        // X = set the wheels in an X configuration
        // B = zero the gyro
        // Y = zero the gyro and the pose
        // A = run trajectory
        // Right Bumper = Emergency Stop(driving)
        swerveSubsystem.defaultCommand = StandardDrive(
            swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
            true,
            true
        )

        // y - Reset gyro
        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(RunCommand({
            swerveSubsystem.zeroGyro()
        }, swerveSubsystem))

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(RunCommand({
            swerveSubsystem.setX()
        }, swerveSubsystem))

        subsystem1.defaultCommand = KCMTMasterElevatorCommand(subsystem1, ArmConstants.elevatorMotorId, ArmConstants.topLinebreakerId, ArmConstants.bottomLinebreakerId, primaryController)

        // SECONDARY
        subsystem2.defaultCommand = KCMTMasterArmCommand(subsystem2, IntakeConstants.wristMotorID, ArmConstants.elbowMotorId, secondaryController)
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    val autonomousCommand: Command = SequentialCommandGroup(
        RunCommand({swerveSubsystem.zeroGyroAndOdometry()}),
        RunCommand({
            //swerveSubsystem.drive(1.0, 0.0,0.0,true, true)
            TrajectoryDrivePathPlanner(swerveSubsystem, traj, false)
        }).withTimeout(10.0)
    )
}