// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import frc.robot.subsystems.SwerveSubsystem
import com.kauailabs.navx.frc.AHRS

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand

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

    val swerveSubsystem = SwerveSubsystem()


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()


    }

    private fun myRunFunction(): Command {

        var trajectoryOne: Trajectory = TrajectoryGenerator.generateTrajectory(
            swerveSubsystem.pose,
            listOf(Translation2d(0.5,0.0),Translation2d(0.5, 1.0), Translation2d(1.5,1.0), Translation2d(1.5,0.0)),
            Pose2d(0.2, 0.05, Rotation2d.fromRadians(Math.PI/2)),
            AutoConstants.config
        )

        var thetaController = ProfiledPIDController(
            AutoConstants.kPThetaController, 0.0, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints
        )
        thetaController.enableContinuousInput(-Math.PI, Math.PI)

        var thetaControllerError = NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("TurningError").getEntry(0.0)

        thetaControllerError.set(thetaController.positionError)







        var swerveControllerCommand: SwerveControllerCommand = SwerveControllerCommand(
            trajectoryOne,
            swerveSubsystem::pose,
            DrivetrainConstants.driveKinematics,

            // Position controllers
            PIDController(AutoConstants.kPXController, 0.0, 0.0),
            PIDController(AutoConstants.kPYController, 0.0, 0.0),
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem
        )

        // Reset odometry to the starting pose of the trajectory.

        println("trajectoryRunning")

        // Run path following command, then stop at the end.
        return ParallelCommandGroup(
            SequentialCommandGroup(
                swerveControllerCommand,
                RunCommand({
                    swerveSubsystem.drive(0.0,0.0,0.0,false)
                })
            ),

            RunCommand({
                println("-------------------------")
//                println(thetaController.velocityError)
//                println(thetaController.positionError)
//                println(thetaController.setpoint.position)
//                println(thetaController.setpoint.velocity)
//                println(swerveSubsystem.heading)
                println(swerveSubsystem.pose.x)
                println(swerveSubsystem.pose.y)
                thetaControllerError.set(thetaController.positionError)

                println("-------------------------")
            })
        )
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

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyro()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(myRunFunction())


    }


}