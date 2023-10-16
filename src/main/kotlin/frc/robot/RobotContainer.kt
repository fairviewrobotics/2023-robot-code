// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.BooleanEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.GenericHID
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
    val ledSubsystem = BlinkinLEDSubsystem()
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
        UPRIGHTPICKCONE,
        FLOORPLACE,
        MIDPLACECUBE,
        HIGHPLACECUBE,
        MIDPLACECONE,
        HIGHPLACECONE,
        CHUTEPICKCUBE,
        CHUTEPICKCONE,
        SHELFPICKCUBE,
        SHELFPICKCONE,
        BASE
    }

    fun select(): CommandSelector {
        if (!CommandValues.pickup && CommandValues.floor){
            return CommandSelector.FLOORPLACE
        } else if (CommandValues.pickup && !CommandValues.chute && CommandValues.shelf && CommandValues.cube) {
            return CommandSelector.SHELFPICKCUBE
        } else if (CommandValues.pickup && !CommandValues.chute && CommandValues.shelf && !CommandValues.cube) {
            return CommandSelector.SHELFPICKCONE
        } else if (CommandValues.pickup && CommandValues.cube && CommandValues.chute) {
            return CommandSelector.CHUTEPICKCUBE
        } else if (CommandValues.pickup && !CommandValues.cube && CommandValues.chute) {
            return CommandSelector.CHUTEPICKCONE
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
            return CommandSelector.UPRIGHTPICKCONE
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

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            StandardDrive(swerveSubsystem,
                { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0 },
                { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar / 2.0 },
                true,
                true
            )
        )

        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar },
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar },
            true,
            true
        )


        pickAndPlace.defaultCommand = Base(pickAndPlace)
        ledSubsystem.defaultCommand = SetLEDValueConeCube(ledSubsystem)

        //PRIMARY CONTROLLER
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
//                    RumbleCheck(primaryController) {true},
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
                            CommandSelector.CHUTEPICKCUBE to ChutePickCube(pickAndPlace),
                            CommandSelector.CHUTEPICKCONE to ChutePickCone(pickAndPlace),
                            CommandSelector.SHELFPICKCUBE to ShelfPickCube(pickAndPlace),
                            CommandSelector.SHELFPICKCONE to ShelfPickCone(pickAndPlace),
                            CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace, primaryController),
                            CommandSelector.UPRIGHTPICKCONE to UprightPickCone(pickAndPlace, primaryController),
                            CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                            CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace, primaryController)
                        ),
                        this::select
                    )
                )
            )

        )

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
                            CommandSelector.CHUTEPICKCUBE to ChutePickCube(pickAndPlace),
                            CommandSelector.CHUTEPICKCONE to ChutePickCone(pickAndPlace),
                            CommandSelector.SHELFPICKCUBE to ShelfPickCube(pickAndPlace),
                            CommandSelector.SHELFPICKCONE to ShelfPickCone(pickAndPlace),
                            CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace, primaryController),
                            CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace, primaryController),
                            CommandSelector.UPRIGHTPICKCONE to UprightPickCone(pickAndPlace, primaryController),
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

//        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
//            RunCommand({
//                TelemetryPosition(pickAndPlace);
//            })
//        )

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
        Trigger {secondaryController.leftTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { 2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
//        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { 4.0 })
//        )
//        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { -4.0 })
//        )

//        JoystickButton(secondaryController, Axis.kLeftY.value ).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { secondaryController.leftY * -4.0 }, { 0.0 }, { 0.0 })
//        )
//        JoystickButton(secondaryController, Axis.kRightY.value).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { secondaryController.rightY * -4.0 }, { 0.0 })
//        )

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
        POVButton(secondaryController, 0).onTrue(
            InstantCommand({
                CommandValues.shelf = !CommandValues.shelf
                CommandValues.ground = !CommandValues.ground
            })
        )
    }



    //AUTO CONFIGURATION
    private fun configureAutoOptions() {
        autoCommandChooser.addOption(
            "Just Place Cube Mid",
            SequentialCommandGroup(
                AutoPlaceCubeMid(pickAndPlace).withTimeout(4.0),
                Base(pickAndPlace)
            )
        )
        autoCommandChooser.addOption(
            "Just Place Cube High",
            SequentialCommandGroup(
                AutoPlaceCubeHigh(pickAndPlace).withTimeout(4.0),
                Base(pickAndPlace)
            )
        )
        autoCommandChooser.addOption(
            "Just Place Cone Mid",
            SequentialCommandGroup(
                AutoPlaceConeMidGetThere(pickAndPlace).withTimeout(2.0),
                AutoPlaceConeMidPlace(pickAndPlace).withTimeout(1.0),
                Base(pickAndPlace)
            )
        )
        autoCommandChooser.addOption(
            "Just Place Cone High",
            SequentialCommandGroup(
                AutoPlaceConeHigh(pickAndPlace).withTimeout(4.0),
                Base(pickAndPlace)
            )
        )
        autoCommandChooser.setDefaultOption(
            "Center Cube Place Mid and Balance",
            SequentialCommandGroup(
                AutoPlaceConeMidGetThere(pickAndPlace).withTimeout(2.0),
                AutoPlaceConeMidPlace(pickAndPlace).withTimeout(1.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.30, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.setDefaultOption(
            "Center Cube Place High and Balance",
            SequentialCommandGroup(
                AutoPlaceCubeHigh(pickAndPlace).withTimeout(4.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.30, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.addOption(
            "Center Cone Place Mid and Balance",
            SequentialCommandGroup(
                AutoPlaceConeMidGetThere(pickAndPlace).withTimeout(2.0),
                AutoPlaceConeMidPlace(pickAndPlace).withTimeout(1.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.30, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.addOption(
            "Center Cone Place High and Balance",
            SequentialCommandGroup(
                AutoPlaceConeHigh(pickAndPlace).withTimeout(4.0),
                ParallelCommandGroup(
                    AutoBase(pickAndPlace),
                    RunCommand({swerveSubsystem.drive(-0.30, 0.0, 0.0, false, true)}, swerveSubsystem)
                ).withTimeout(2.0),
                ParallelCommandGroup(
                    Base(pickAndPlace),
                    Balancer(swerveSubsystem)
                )
            )
        )
        autoCommandChooser.addOption(
            "Blue Left Cone Far Place Mid Leave",
            testTrajectories.BlueLeftConeLeftPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Blue Left Cube Place High Leave",
            testTrajectories.BlueTop1()
        )
        autoCommandChooser.addOption(
            "Blue Left Cone Near Place Mid Leave",
            testTrajectories.BlueLeftConeRightPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Blue Right Cone Near Place Mid Leave",
            testTrajectories.BlueRightConeLeftPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Blue Right Cube Place High Leave",
            testTrajectories.BlueBottom1()
        )
        autoCommandChooser.addOption(
            "Blue Right Cone Far Place Mid Leave",
            testTrajectories.BlueRightConeRightPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Red Left Cone Far Place Mid Leave",
            testTrajectories.RedLeftConeLeftPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Red Left Cube Place High Leave",
            testTrajectories.RedBottom1()
        )
        autoCommandChooser.addOption(
            "Red Left Cone Near Place Mid Leave",
            testTrajectories.RedLeftConeRightPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Red Right Cone Near Place Mid Leave",
            testTrajectories.RedRightConeLeftPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Red Right Cube Place High Leave",
            testTrajectories.RedTop1()
        )
        autoCommandChooser.addOption(
            "Red Right Cone Far Place Mid Leave",
            testTrajectories.RedRightConeRightPlaceLeave()
        )
        autoCommandChooser.addOption(
            "Blue Left Cube Place High Leave and Balance",
            testTrajectories.BlueTop1Balance()
        )
        autoCommandChooser.addOption(
            "Blue Right Cube Place High Leave and Balance",
            testTrajectories.BlueBottom1Balance()
        )
        autoCommandChooser.addOption(
            "Red Left Cube Place High Leave and Balance",
            testTrajectories.RedBottom1Balance()
        )
        autoCommandChooser.addOption(
            "Red Right Cube Place High Leave and Balance",
            testTrajectories.RedTop1Balance()
        )
        autoCommandChooser.addOption(
            "Test Path",
            testTrajectories.TestPath()
        )
        SmartDashboard.putData("Auto Mode", autoCommandChooser)
    }
    val autonomousCommand: Command get() = autoCommandChooser.selected
}