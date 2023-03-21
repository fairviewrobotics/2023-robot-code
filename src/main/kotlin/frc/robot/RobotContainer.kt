// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.wpilibj.XboxController.Axis
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.commands.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.*
import frc.robot.commands.AlignToAprilTag
import java.time.Instant

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {

    val primary = XboxController(0)
    val sparkMax = SparkMaxSubsystem(9)
    val swerveSubsystem = SwerveSubsystem()
    init {
        configureButtonBindings()

    }

    private fun configureButtonBindings() {

        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem, { primary.leftY }, { primary.leftX }, { primary.rightX }, true, true)
        JoystickButton(primary, XboxController.Button.kLeftBumper.value).whileTrue(RunCommand({ swerveSubsystem.zeroGyroAndOdometry() }))
        //sparkMax.defaultCommand = QuickSpin(primary, sparkMax, 25)
        JoystickButton(primary, XboxController.Button.kA.value).whileTrue(AlignToAprilTag(swerveSubsystem, primary))
        JoystickButton(primary, XboxController.Button.kB.value).whileTrue(AlignToRetroreflective(swerveSubsystem, primary))
        JoystickButton(primary, XboxController.Button.kX.value).whileTrue(AlignToCone(swerveSubsystem, primary))
        JoystickButton(primary, XboxController.Button.kY.value).whileTrue(AlignToCube(swerveSubsystem, primary))
    }

    val autonomousCommand: Command = InstantCommand({})
}
