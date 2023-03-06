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
    val pickAndPlace = PickAndPlaceSubsystem(ArmConstants.elevatorMotorId, ArmConstants.elbowMotorId, 11, 12, 13, 0, 1)
    val swerveSubsystem = SwerveSubsystem()

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()


    }
    
    private fun configureButtonBindings() {
       /*swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * 5.0 },
            { primaryController.leftX * 5.0 },
            { primaryController.rightX * 1.0},
            true,
            true)
*/
//        JoystickButton(primaryController, XboxController.Button.kA.value).onTrue(
//            AlignToAprilTag(swerveSubsystem, primaryController)
//        )
//TODO: The alignment to apriltag needs to happen before the pick and place command in sequential command order
//TODO: The alignment to apriltag needs to change to pegs for certain cases, or that needs to be added in in a different way cause right now, it would only place for cube shelves
//TODO: Robot shoots down, something is wrong that we aren't noticing
        pickAndPlace.defaultCommand = zeroVoltage(pickAndPlace)//change to base once done testing

        JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, primaryController)
            TestPickandPlace(pickAndPlace)
        )

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            LowPickCone(pickAndPlace)
        )

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            LowPickCube(pickAndPlace)
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            Base(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, primaryController)
            MidPlace(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, primaryController)
            HighPlace(pickAndPlace)
        )

    }

    val autonomousCommand: Command = RunCommand({})
}
