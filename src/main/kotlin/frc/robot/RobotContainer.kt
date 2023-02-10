// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import frc.robot.subsystems.SwerveSubsystem

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator

import frc.robot.commands.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.utils.DriveUtils

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    private val primaryController = XboxController(0)
    private val secondaryController = XboxController(1)

    private val swerveSubsystem = SwerveSubsystem()
    private val limelightSubsystem = LimelightSubsystem()

    private val driveUtils = DriveUtils(swerveSubsystem)


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
        swerveSubsystem.defaultCommand = UnlimitedDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
            true
        )

//        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
//            RunCommand({
//                swerveSubsystem.setX()
//            }, swerveSubsystem)
//        )

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            RunCommand({
                swerveSubsystem.setZero()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            RunCommand({
                LineUp(swerveSubsystem, limelightSubsystem)
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyro()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroOdometry()
            }, swerveSubsystem)
        )
//        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            RunCommand({
//                swerveSubsystem.resetReset()
//            }, swerveSubsystem)
//        )



        JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(driveUtils.trajectoryDrive(TrajectoryGenerator.generateTrajectory(
                swerveSubsystem.pose,
            listOf(Translation2d(0.5,0.0),Translation2d(0.5, 1.0), Translation2d(1.5,1.0), Translation2d(1.5,0.0)),
            Pose2d(0.2, 0.05, Rotation2d.fromRadians(Math.PI/2)),
            TrajectoryConstants.config
        )))

    }
}