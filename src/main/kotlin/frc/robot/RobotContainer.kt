// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.EncoderReadout
import frc.robot.commands.MotorTest
import frc.robot.commands.OpenLoopTest
import frc.robot.commands.PointInDirection
import frc.robot.controllers.SwerveModuleControlller
import frc.robot.subsystems.SparkMaxSubsystem
import frc.robot.subsystems.SwerveSubsystem
import com.kauailabs.navx.frc.AHRS

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
        swerveSubsystem.defaultCommand = RunCommand({
            swerveSubsystem.drive(
                MathUtil.applyDeadband(primaryController.leftY * DrivetrainConstants.drivingSpeedScalar, 0.06),
                MathUtil.applyDeadband(primaryController.leftX * DrivetrainConstants.drivingSpeedScalar, 0.06),
                MathUtil.applyDeadband(primaryController.rightX  * DrivetrainConstants.rotationSpeedScalar, 0.06),
                true
            )
        }, swerveSubsystem)

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            RunCommand({
                swerveSubsystem.setZero()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyro()
            }, swerveSubsystem)
        )

    }


}