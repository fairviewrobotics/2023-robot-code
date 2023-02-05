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
import frc.robot.controllers.SwerveModuleControlller


import frc.robot.subsystems.SwerveSubsystem
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.motorcontrol.Spark
import frc.robot.commands.*
import frc.robot.controllers.DigitalInputSubsystem
import frc.robot.controllers.SparkMaxSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)

    //val swerveSubsystem = SwerveSubsystem()
    //val simpleTest = SparkMaxSubsystem(2)
    val motor = SparkMaxSubsystem(2)
    val topLimit = DigitalInputSubsystem(0)
    val bottomLimit = DigitalInputSubsystem(1)
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
        motor.defaultCommand = EncoderAndLinebreakerTest(bottomLimit, topLimit, motor, primaryController)

        /*swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem, { primaryController.leftY }, { primaryController.leftX }, { primaryController.rightTriggerAxis }, true)

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
*/
    }


}