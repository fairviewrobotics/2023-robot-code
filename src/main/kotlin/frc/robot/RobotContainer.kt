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


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.motorcontrol.Spark

import frc.robot.commands.*
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
//    val mArmSubsystem = ArmSubsystem(1,0,2)
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)
//    val topBreaker = DigitalInputSubsystem(0)
//    val bottomBreaker = DigitalInputSubsystem(1)
    val swerveSubsystem = SwerveSubsystem()
//    val elevatorMotor = SparkMaxSubsystem(9)

   // val topBreaker = DigitalInputSubsystem(1)
    //val bottomBreaker = DigitalInputSubsystem(0) //linebreak wiring: brown - 5v, blue - GND, black - output

    //val elevatorMotor = SparkMaxSubsystem(2)







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
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
            true,
            true
        )
    }

}
