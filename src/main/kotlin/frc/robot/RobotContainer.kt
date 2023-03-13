// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController.Axis
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
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
    val pickAndPlace = PickAndPlaceSubsystem()
    val swerveSubsystem = SwerveSubsystem()
    val trajectories = Trajectories(pickAndPlace, swerveSubsystem)

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()
    }
    
    private fun configureButtonBindings() {

       swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar  / 2.0},
            true,
            true)

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            StandardDrive(swerveSubsystem,
                { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
                { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
                { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
                true,
                true
        ))
//TODO: The alignment to apriltag needs to happen before the pick and place command in sequential command order
//TODO: The alignment to apriltag needs to change to pegs for certain cases, or that needs to be added in in a different way cause right now, it would only place for cube shelves

        //PRIMARY CONtROLLER:
        pickAndPlace.defaultCommand = Base(pickAndPlace)

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            })
        )
        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyroAndOdometry()
            })
        )
        Trigger {primaryController.leftTriggerAxis > 0.2} .whileTrue(
            LowPickCube(pickAndPlace)
        )
        Trigger {primaryController.rightTriggerAxis > 0.2} .whileTrue(
            LowPickCone(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            ShelfPick(pickAndPlace)

        )
        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            ChutePick(pickAndPlace)
        )

        //SECONDARY CONtROLLER
        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            MidPlaceCone(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            MidPlaceCube(pickAndPlace)
        )
        Trigger {secondaryController.leftTriggerAxis > 0.2} .whileTrue(
            HighPlaceCone(pickAndPlace)
        )
        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
            HighPlaceCube(pickAndPlace)
        )
    }
    val autonomousCommand: Command = RunCommand({trajectories.AutoBuilder()})
}
