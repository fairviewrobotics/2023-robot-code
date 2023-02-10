// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import frc.robot.subsystems.SwerveSubsystem

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.motorcontrol.Spark

import frc.robot.commands.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.DigitalInputSubsystem
import frc.robot.subsystems.SparkMaxSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    val top = DigitalInputSubsystem(1)
    val bottom = DigitalInputSubsystem(0)
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)

    val swerveSubsystem = SwerveSubsystem()

    //Right = pos y
    //Backward = pos x
    val traj: PathPlannerTrajectory = PathPlanner.generatePath(
        PathConstraints(12.0, 3.5),
        PathPoint(Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0), 0.0),
        PathPoint(Translation2d(-5.0, 0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)), // position, heading(direction of travel), holonomic rotation, velocity override
        PathPoint(Translation2d(-5.0, 0.5), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0.0)), // position, heading(direction of travel), holonomic rotation
        PathPoint(Translation2d(0.0, 0.5), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(90.0)),
        PathPoint(Translation2d(0.0, 1.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(90.0)),
        PathPoint(Translation2d(-5.0, 1.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(180.0)),
        PathPoint(Translation2d(-5.0, 1.5), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(180.0)),
        PathPoint(Translation2d(0.0, 1.5), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(180.0)),
        PathPoint(Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(180.0))// position, heading(direction of travel), holonomic rotation
    )

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
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
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
            true,
            true
        )


        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyro()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyroAndOdometry()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            RunCommand({
                swerveSubsystem.drive(0.0,0.0,0.0,true,false)
            }, swerveSubsystem)
        )


        JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
            TrajectoryDrivePathPlanner(swerveSubsystem, traj, false)
        )
    }
}