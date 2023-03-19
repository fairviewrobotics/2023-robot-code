// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.BooleanEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.wpilibj.XboxController.Axis
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
    val trajectories = Trajectories(pickAndPlace, swerveSubsystem)
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
        //Venezuela()
        configureButtonBindings()
        //Discovery()
    }

    // Do not modify.
    enum class Piece(val x: Int) {
        CONE(0),
        CUBE(1);

        companion object {
            fun fromInt(value: Int) = Piece.values().first { it.x == value }
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


    var commandMapping: Command =
        SelectCommand(
            mapOf(
                CommandSelector.BASE to Base(pickAndPlace),
                CommandSelector.CHUTEPICK to ChutePick(pickAndPlace),
                CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace),
                CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace),
                CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace),
                CommandSelector.LOWPICKCONE to LowPickCone(pickAndPlace),
                CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace),
                CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace)
            ),
            this::select
        )



    /**
     * The Columbia control scheme.
     *
     * **DO NOT MODIFY!** This is an official control scheme and should not be modified unless debugging. To test your own
     * systems or controls, write another function.
     *
     * The primary controller has a high level control of the robot (Left joystick). They can drive the robot, and run
     * either a pick (X) or a place (Y) command. They can vary the speed of the robot's drivetrain (A).
     *
     * The secondary controller has the control to decide how a pick or place wants to be done. They can control whether
     * cones or cubes be picked up (A), where the cone or cube is placed (B), and whether to use vision (X).
     *
     * Should be used with a networktables utility to inspect the modes chosen.
     */
    private fun Columbia() {

        val nt = NetworkTableInstance.getDefault().getTable("DriverControl")

        var piece: Piece = Piece.CUBE
        var place: Place = Place.LOW
        var vision = false

        val pieceNT = nt.getIntegerTopic("Piece").publish()
        val placeNT = nt.getIntegerTopic("Place").publish()
        val visionNT = nt.getBooleanTopic("Vision").publish()

        val lockDrive: () -> Command = {
            StandardDrive(swerveSubsystem,
                { primaryController.leftY * DrivetrainConstants.maxSpeedMetersPerSecond / (if (primaryController.aButton) 1.0 else 2.0) },
                { 0.0 },
                { 0.0 },
                false,
                true)
        }

        pickAndPlace.defaultCommand = Base(pickAndPlace)

        val pickCommand: () -> Command = { ->
            val pnpCommand = if (piece == Piece.CONE) {
                LowPickCone(pickAndPlace)
            } else {
                LowPickCube(pickAndPlace)
            }

            val visionCommand = if (piece == Piece.CONE) {
                AlignToCone(swerveSubsystem, primaryController)
            } else {
                AlignToCube(swerveSubsystem, secondaryController)
            }

            ParallelCommandGroup(
                SequentialCommandGroup(
                    if (vision) visionCommand else InstantCommand({}),
                    lockDrive()
                ),
                pnpCommand
            )
        }

        val placeCommand: () -> Command = { ->
            val pnpCommand = if (piece == Piece.CONE) {
                when (place) {
                    Place.HIGH -> HighPlaceCone(pickAndPlace)
                    Place.MID -> MidPlaceCone(pickAndPlace)
                    Place.LOW -> FloorPlace(pickAndPlace)
                }
            } else {
                when (place) {
                    Place.HIGH -> HighPlaceCube(pickAndPlace)
                    Place.MID -> MidPlaceCube(pickAndPlace)
                    Place.LOW -> FloorPlace(pickAndPlace)
                }
            }

            val visionCommand = if (piece == Piece.CONE) {
                AlignToRetroreflective(swerveSubsystem, primaryController)
            } else {
                AlignToAprilTag(swerveSubsystem, secondaryController)
            }

            ParallelCommandGroup(
                SequentialCommandGroup(
                    if (vision) visionCommand else InstantCommand({}),
                    lockDrive()
                ),
                pnpCommand
            )
        }
        // Primary: Left joystick drives, A button enables faster speed, B button picks, Y button places, left and right triggers control intake speed.
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.maxSpeedMetersPerSecond / (if (primaryController.aButton) 1.0 else 2.0) },
            { primaryController.leftX * DrivetrainConstants.maxSpeedMetersPerSecond / (if (primaryController.aButton) 1.0 else 2.0) },
            { primaryController.rightX * DrivetrainConstants.maxAngularSpeed / (if (primaryController.aButton) 1.0 else 2.0)},
            true,
            true)

        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(pickCommand())
        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(placeCommand())

        // Secondary: A button changes piece mode, B button changes place mode, X button changes vision mode.
        JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(InstantCommand({
            piece = Piece.fromInt((piece.ordinal + 1) % (Piece.values().size))
            pieceNT.set(piece.ordinal.toLong())
        }))

        JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(InstantCommand({
            place = Place.fromInt((place.ordinal + 1) % (Place.values().size))
            placeNT.set(place.ordinal.toLong())
        }))

        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(InstantCommand({
            vision = !vision
            visionNT.set(vision)
        }))

    }

    // Modify.
    private fun Venezuela() {

        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 2.0 }, { 0.0 })
        )

        JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { -2.0 }, { 0.0 })
        )

        Trigger {primaryController.rightTriggerAxis > 0.2}.whileTrue(
            VoltageArm(pickAndPlace, {0.0}, {2.0}, {0.0}, {0.0})
        )

        Trigger {primaryController.leftTriggerAxis > 0.2}.whileTrue(
            VoltageArm(pickAndPlace, {0.0}, {-2.0}, {0.0}, {0.0})
        )
//        Trigger {primaryController.leftTriggerAxis > 0.2} .whileTrue(
//            VoltageArm(pickAndPlace, { primaryController.leftTriggerAxis * -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
//        )
//        Trigger {primaryController.rightTriggerAxis > 0.2} .whileTrue(
//            VoltageArm(pickAndPlace, { primaryController.rightTriggerAxis * 4.0 }, { 0.0 }, { 0.0 }, { 0.0 })
//        )
//        JoystickButton(primaryController, Axis.kLeftY.value).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { primaryController.leftY * -4.0 }, { 0.0 }, { 0.0 })
//        )
//        JoystickButton(primaryController, Axis.kRightY.value).whileTrue(
//            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { primaryController.rightY * -4.0 }, { 0.0 })
//        )
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
                RunCommand({
                    CommandValues.pickup = true
                }),
                SelectCommand(
                    mapOf(
                        CommandSelector.BASE to Base(pickAndPlace),
                        CommandSelector.CHUTEPICK to ChutePick(pickAndPlace),
                        CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace),
                        CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace),
                        CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace),
                        CommandSelector.LOWPICKCONE to LowPickCone(pickAndPlace),
                        CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                        CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace),
                        CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace)
                    ),
                    this::select
                )
            )
        )

        Trigger { primaryController.rightTriggerAxis > 0.2 }.whileTrue(
            SequentialCommandGroup(
                RunCommand({
                    CommandValues.pickup = false
                }),
                SelectCommand(
                    mapOf(
                        CommandSelector.BASE to Base(pickAndPlace),
                        CommandSelector.CHUTEPICK to ChutePick(pickAndPlace),
                        CommandSelector.FLOORPLACE to FloorPlace(pickAndPlace),
                        CommandSelector.HIGHPLACECONE to HighPlaceCone(pickAndPlace),
                        CommandSelector.HIGHPLACECUBE to HighPlaceCube(pickAndPlace),
                        CommandSelector.LOWPICKCONE to LowPickCone(pickAndPlace),
                        CommandSelector.LOWPICKCUBE to LowPickCube(pickAndPlace),
                        CommandSelector.MIDPLACECONE to MidPlaceCone(pickAndPlace),
                        CommandSelector.MIDPLACECUBE to MidPlaceCube(pickAndPlace)
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

        //SECONDARY CONTROLLER

        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { 4.0 })
        )
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { -4.0 })
        )
        Trigger {primaryController.leftTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { primaryController.leftTriggerAxis * -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        Trigger {primaryController.rightTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { primaryController.rightTriggerAxis * 4.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, Axis.kLeftY.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { primaryController.leftX * -4.0 }, { 0.0 }, { 0.0 })
        )
        JoystickButton(secondaryController, Axis.kRightY.value).whileTrue(
            VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { primaryController.rightY * -4.0 }, { 0.0 })
        )

        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                CommandValues.cube = !CommandValues.cube
                CommandValues.cone = !CommandValues.cone
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            RunCommand({
                CommandValues.floor = !CommandValues.floor
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            RunCommand({
                CommandValues.highPlace = !CommandValues.highPlace
                CommandValues.middlePlace = !CommandValues.middlePlace
            })
        )

        JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                CommandValues.ground = !CommandValues.ground
                CommandValues.chute = !CommandValues.chute
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
            HighPlaceCone(pickAndPlace)
        )
        Trigger {secondaryController.rightTriggerAxis > 0.2} .whileTrue(
            VoltageArm(pickAndPlace, { 2.0 }, { 0.0 }, { 0.0 }, { 0.0 })
        )
        //TODO: Tune
        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            MidPlaceCone(pickAndPlace)
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
            FloorPlace(pickAndPlace)
        )

    }

    val autonomousCommand: Command = RunCommand({})
}