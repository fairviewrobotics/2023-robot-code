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
    //val trajectories = Trajectories(pickAndPlace, swerveSubsystem)
    val testTrajectories = AutoTrajectories(pickAndPlace, swerveSubsystem)

    // val ledsubsystem = yadda yadad

    //Starting Config: Cube, with Middle Place, and Ground Pickup
////    var cube: Boolean = true // Both
////    var middlePlace: Boolean = true // Place
////    var floor: Boolean = false // Place
////    var chute: Boolean = false // Pickup
//
//    // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
//    var ground: Boolean = true // Pickup
//    var cone: Boolean = true // Both
//    var highPlace: Boolean = false // Place




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // change this variable if you would like to use competition bindings (final tuning, driver practice), or test bindings
        // for (individual tuning and what not)
        //configureButtonBindings()
        Discovery()
        configureAutoOptions()
    }

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


    private fun Discovery() {
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar  / 2.0},
            true,
            true
        )

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            StandardDrive(swerveSubsystem,
                { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
                { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
                { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
                true,
                true
            )
        )

        pickAndPlace.defaultCommand = Base(pickAndPlace)

        //PRIMARY CONTROLLER
        Trigger { primaryController.leftTriggerAxis > 0.2 }.whileTrue(
            SequentialCommandGroup(
                InstantCommand({
                    CommandValues.pickup = true
                }),

                if (CommandValues.vision && CommandValues.chute)
                    AlignToAprilTag(swerveSubsystem,primaryController)
                else if (CommandValues.vision && CommandValues.cube)
                    AlignToCube(swerveSubsystem, primaryController)
                else if (CommandValues.vision && !CommandValues.cube)
                    AlignToCone(swerveSubsystem, primaryController)
                else
                    InstantCommand({}),

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

        Trigger { primaryController.rightTriggerAxis > 0.2 }.whileTrue(
            SequentialCommandGroup(
                InstantCommand({
                    CommandValues.pickup = false
                }),

                if (CommandValues.vision && CommandValues.cube)
                    AlignToAprilTag(swerveSubsystem,primaryController)
                else if (CommandValues.vision && !CommandValues.cube)
                    AlignToRetroreflective(swerveSubsystem, primaryController)
                else
                    InstantCommand({}),

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
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { 4.0 })
        )
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { -4.0 })
        )
        Trigger {secondaryController.leftTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { secondaryController.leftTriggerAxis * -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { secondaryController.rightTriggerAxis * 4.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, Axis.kLeftY.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { secondaryController.leftY * -4.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, Axis.kRightY.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { secondaryController.rightY * -4.0 }, { 0.0 })
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

    private fun configureButtonBindings() {
        //TESTING CONTROLS:

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
            )
        )


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
        //TODO: Tune
        Trigger {secondaryController.leftTriggerAxis > 0.2} .whileTrue(
            HighPlaceCone(pickAndPlace, primaryController)
        )
        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { 2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        //TODO: Tune
        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            MidPlaceCone(pickAndPlace, primaryController)
        )
        //TODO: Tune
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            LowPickCone(pickAndPlace)
        )
        //Ready
        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            ChutePick(pickAndPlace)
        )
        //TODO: Tune
        JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            FloorPlace(pickAndPlace, primaryController)
        )

    }

    //AUTO CONFIGURATION

    var autoCommandChooser: SendableChooser<Command> = SendableChooser()
    private fun configureAutoOptions() {

        autoCommandChooser.setDefaultOption(
            "Center Place Cube and Balance",
            SequentialCommandGroup(
                AutoPlaceHigh(pickAndPlace).withTimeout(4.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.2, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )

        autoCommandChooser.addOption(
            "Center Place Cone and Balance",
            SequentialCommandGroup(
                AutoPlaceConeHigh(pickAndPlace).withTimeout(5.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.2, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )


        SmartDashboard.putData("Auto Mode", autoCommandChooser)
    }

    val autonomousCommand: Command get() = autoCommandChooser.selected





}