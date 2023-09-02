// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.BooleanEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.wpilibj.XboxController.Axis
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.commands.*
import frc.robot.constants.CommandValues

import frc.robot.constants.DrivetrainConstants
import frc.robot.subsystems.*
import java.util.ConcurrentModificationException
import kotlin.math.abs

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
    val testTrajectories = AutoTrajectories(pickAndPlace, swerveSubsystem)
    var autoCommandChooser: SendableChooser<Command> = SendableChooser()


    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        configureAutoOptions()
        Discovery()
    }


    enum class Place(val x: Int) {
        HIGH(0),
        MID(1),
        LOW(2);

        companion object {
            fun fromInt(value: Int) = Place.values().first { it.x == value }
        }
    }

    enum class CommandSelector {
        LOWPICKCUBE,
        LOWPICKCONE,
        CHUTEPICK,
        FLOORPLACE,
        MIDPLACECUBE,
        HIGHPLACECUBE,
        MIDPLACECONE,
        HIGHPLACECONE,
        BASE
    }

    fun select(): CommandSelector {
        if (!CommandValues.pickup && CommandValues.floor){
            return CommandSelector.FLOORPLACE
        } else if (CommandValues.pickup && CommandValues.chute) {
            return CommandSelector.CHUTEPICK
        } else if (!CommandValues.pickup && CommandValues.cube && CommandValues.middlePlace) {
            return CommandSelector.MIDPLACECUBE
        } else if (!CommandValues.pickup && !CommandValues.cube && CommandValues.middlePlace) {
            return CommandSelector.MIDPLACECONE
        } else if (!CommandValues.pickup && CommandValues.cube && !CommandValues.middlePlace) {
            return CommandSelector.HIGHPLACECUBE
        } else if (!CommandValues.pickup && !CommandValues.cube && !CommandValues.middlePlace) {
            return CommandSelector.HIGHPLACECONE
        } else if (CommandValues.pickup && CommandValues.cube) {
            return CommandSelector.LOWPICKCUBE
        } else if (CommandValues.pickup && !CommandValues.cube) {
            return CommandSelector.LOWPICKCONE
        } else {
            return CommandSelector.BASE
        }
    }

    enum class VisionSelector {
        CHUTEVISION,
        RETROREFLECTIVE,
        NOVISION
    }

    fun selectVision(): VisionSelector {
        if (CommandValues.floor && !CommandValues.pickup){
            return VisionSelector.NOVISION
        } else if (CommandValues.vision && CommandValues.chute && CommandValues.pickup){
            //return VisionSelector.CHUTEVISION
            return VisionSelector.NOVISION
        } else if (CommandValues.vision && CommandValues.cone && !CommandValues.pickup) {
            return VisionSelector.RETROREFLECTIVE
        } else {
            return VisionSelector.NOVISION
        }
    }

    private fun Discovery() {
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar},
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar},
            true,
            true
        )

        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            StandardDrive(swerveSubsystem,
                { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar / 2.0 },
                true,
                true
            )
        )

        pickAndPlace.defaultCommand = Base(pickAndPlace)

        //PRIMARY CONTROLLER
        Trigger { primaryController.rightTriggerAxis > 0.2 }.whileTrue(


            SequentialCommandGroup(
                InstantCommand({
                    CommandValues.pickup = true
                }),

                SelectCommand(
                    mapOf(
                        VisionSelector.RETROREFLECTIVE to RetroreflectiveVision(swerveSubsystem, primaryController).withTimeout(3.0),
                        VisionSelector.CHUTEVISION to ChuteVision(swerveSubsystem, primaryController).withTimeout(6.0),
                        VisionSelector.NOVISION to InstantCommand({})
                    ),
                    this::selectVision
                ),
                ParallelCommandGroup(
                    StandardDrive(swerveSubsystem,
                        { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                        { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
                        { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar  / 2.0},
                        true,
                        true
                    ).repeatedly(),

                    SelectCommand(
                        mapOf(
                            CommandSelector.BASE to Base(pickAndPlace),
                            CommandSelector.CHUTEPICK to ChutePick(pickAndPlace),
                            CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace, primaryController),
                            CommandSelector.LOWPICKCONE to LowPickCone(pickAndPlace),
                            CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                            CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace, primaryController)
                        ),
                        this::select
                    )
                )
            )

        )

        Trigger { primaryController.leftTriggerAxis > 0.2 }.whileTrue(
            SequentialCommandGroup(
                InstantCommand({
                    CommandValues.pickup = false
                }),

                SelectCommand(
                    mapOf(
                        VisionSelector.RETROREFLECTIVE to RetroreflectiveVision(swerveSubsystem, primaryController).withTimeout(3.0),
                        VisionSelector.CHUTEVISION to ChuteVision(swerveSubsystem, primaryController).withTimeout(6.0),
                        VisionSelector.NOVISION to InstantCommand({})
                    ),
                    this::selectVision
                ),

                ParallelCommandGroup(
                    StandardDrive(swerveSubsystem,
                        { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                        { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
                        { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar / 2.0},
                        true,
                        true
                    ).repeatedly(),

                    SelectCommand(
                        mapOf(
                            CommandSelector.BASE to Base(pickAndPlace),
                            CommandSelector.CHUTEPICK to ChutePick(pickAndPlace),
                            CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace, primaryController),
                            CommandSelector.LOWPICKCONE to LowPickCone(pickAndPlace),
                            CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                            CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace, primaryController)
                        ),
                        this::select
                    )
                )
            )
        )

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            })
        )


        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyro()
            })
        )

        JoystickButton(primaryController, XboxController.Button.kA.value).onTrue(
            InstantCommand({
                CommandValues.fieldOriented = !CommandValues.fieldOriented
            })
        )

        //SECONDARY CONTROLLER

        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { -4.0 })
        )
        Trigger {secondaryController.leftTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { 2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        //TODO: This wasn't working
        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { 4.0 })
        )
//        JoystickButton(secondaryController, Axis.kLeftY.value ).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { secondaryController.leftY * -4.0 }, { 0.0 }, { 0.0 })
//        )
//        JoystickButton(secondaryController, Axis.kRightY.value).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { secondaryController.rightY * 4.0 }, { 0.0 })
//        )
//TODO: Test the following
        Trigger { abs( secondaryController.leftY ) > 0.1 }.whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { secondaryController.leftY * -4.0}, { 0.0 })
        )

        Trigger { abs( secondaryController.rightY ) > 0.1 }.whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { secondaryController.leftY * -4.0}, { 0.0 })
        )

        JoystickButton(secondaryController, XboxController.Button.kX.value).onTrue(
            InstantCommand({
                CommandValues.cube = !CommandValues.cube
                CommandValues.cone = !CommandValues.cone
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kA.value).onTrue(
            InstantCommand({
                CommandValues.floor = !CommandValues.floor
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kB.value).onTrue(
            InstantCommand({
                CommandValues.highPlace = !CommandValues.highPlace
                CommandValues.middlePlace = !CommandValues.middlePlace
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kY.value).onTrue(
            InstantCommand({
                CommandValues.ground = !CommandValues.ground
                CommandValues.chute = !CommandValues.chute
            })
        )

        POVButton(secondaryController, 180).onTrue(
            InstantCommand({
                CommandValues.vision = !CommandValues.vision
            })
        )



    }
    //AUTO CONFIGURATION


    private fun configureAutoOptions() {
        autoCommandChooser.setDefaultOption(
            "Center Place Cube High and Balance",
            SequentialCommandGroup(
                AutoPlaceCubeHigh(pickAndPlace).withTimeout(4.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.25, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.addOption(
            "Center Place Cone Mid and Balance",
            SequentialCommandGroup(
                AutoPlaceConeMid(pickAndPlace).withTimeout(4.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.25, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.addOption(
            "Just Place Cube High",
            SequentialCommandGroup(
                AutoPlaceCubeHigh(pickAndPlace).withTimeout(4.0),
                AutoBase(pickAndPlace)
            )
        )
        autoCommandChooser.addOption(
            "Just Place Cone Mid",
            SequentialCommandGroup(
                AutoPlaceConeMid(pickAndPlace).withTimeout(4.0),
                AutoBase(pickAndPlace)
            )
        )
        autoCommandChooser.addOption(
            "Red Left Place Cube High Leave",
            testTrajectories.RedBottom1()
        )
        autoCommandChooser.addOption(
            "Red Right Place Cube High Leave",
            testTrajectories.RedTop1()
        )
        autoCommandChooser.addOption(
            "Blue Left Place Cube High Leave",
            testTrajectories.BlueTop1()
        )
        autoCommandChooser.addOption(
            "Blue Right Place Cube High Leave",
            testTrajectories.BlueBottom1()
        )
        SmartDashboard.putData("Auto Mode", autoCommandChooser)
    }
    val autonomousCommand: Command get() = autoCommandChooser.selected

//        SequentialCommandGroup(
//            AutoPlaceConeMid(pickAndPlace).withTimeout(4.0),
//            RunCommand({swerveSubsystem.drive(-0.5,0.0,0.0,false, true)}, swerveSubsystem).withTimeout(1.0),
//            ParallelCommandGroup(
//                RunCommand({swerveSubsystem.drive(0.0,0.0,0.0,false, true)}, swerveSubsystem),
//                Base(pickAndPlace)
//            )
//        )
}