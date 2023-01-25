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




//    val fl = SparkMaxSubsystem(CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val fr = SparkMaxSubsystem(CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val rl = SparkMaxSubsystem(CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val rr = SparkMaxSubsystem(CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless))
//
//    val fll = SparkMaxSubsystem(CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val frr = SparkMaxSubsystem(CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val rll = SparkMaxSubsystem(CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless))
//    val rrr = SparkMaxSubsystem(CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless))

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
//        fl.defaultCommand = EncoderReadout("FrontLeft", fl, true, 2.862-(Math.PI/2))
//        fr.defaultCommand = EncoderReadout("FrontRight", fr, true, 4.285+(0.0))
//        rl.defaultCommand = EncoderReadout("RearLeft", rl, true, 0.871+(Math.PI))
//        rr.defaultCommand = EncoderReadout("RearRight", rr, true, 2.090+(Math.PI/2))
        swerveSubsystem.defaultCommand = PointInDirection(swerveSubsystem, primaryController)
        swerveSubsystem.defaultCommand = RunCommand({
            swerveSubsystem.drive(
                MathUtil.applyDeadband(primaryController.leftY, 0.06),
                MathUtil.applyDeadband(primaryController.leftX, 0.06),
                MathUtil.applyDeadband(primaryController.rightX, 0.06),
                false
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

    }


}