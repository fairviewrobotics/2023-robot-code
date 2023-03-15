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

    //Starting Config: Cube, with Middle Place, and Ground Pickup
    var cube: Boolean = true // Both
    var middlePlace: Boolean = true // Place
    var floor: Boolean = false // Place
    var chute: Boolean = false // Pickup

    // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
    var ground: Boolean = true // Pickup
    var cone: Boolean = false // Both
    var highPlace: Boolean = false // Place




    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // change this variable if you would like to use competition bindings (final tuning, driver practice), or test bindings
        // for (individual tuning and what not)
        configureButtonBindings()
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
    private fun Challenger() {
        /**
        BUTTON BINDINGS:
        //PRIMARY CONtROLLER:
        //Left Trigger = Execute Pickup
        //Right Trigger = Execute Place
        //X = Set X
        //Y = Zero Gyro
        //B = Fast Drive
        //SECONDARY CONtROLLER
        //A = Set Middle Place
        //Y = Set High place
        //X = Set Cube
        //B = Set Cone
        //Bottom D-Pad = Set Floor Pickup
        //Top D-Pad = Set Chute Pickup
        //Left D-Pad = Set Ground Pickup
        //Voltage Controls:
        //Left Bumper = Intake In
        //Right Bumper = Intake Out
        //Left Trigger = Elevator Down
        //Right Trigger = Elevator Up
        //Left Joystick = Elbow(Arm)
        //Right Joystick = Wrist
         */
        //Network Tables
        val nt = NetworkTableInstance.getDefault().getTable("DriverControl")


        var cubeNT = nt.getBooleanTopic("Cube").publish()
        var middlePlaceNT = nt.getBooleanTopic("Middle Place").publish()
        var floorNT = nt.getBooleanTopic("Floor Pickup").publish()
        var chuteNT = nt.getBooleanTopic("Chute Pickup").publish()

        // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
        var groundNT = nt.getBooleanTopic("Ground Pickup").publish()
        var coneNT = nt.getBooleanTopic("Cone Pickup").publish()
        var highPlaceNT = nt.getBooleanTopic("High Place").publish()

        //Default Commands
        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar  / 2.0},
            true,
            true)

        pickAndPlace.defaultCommand = Base(pickAndPlace)

        //PRIMARY CONtROLLER:

        //Left Trigger = Execute Pickup
        //Right Trigger = Execute Place
        //X = Set X
        //Y = Zero Gyro
        //B = Fast Drive


        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            }, swerveSubsystem)
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyroAndOdometry()
            }, swerveSubsystem)
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
        POVButton(primaryController, 180).whileTrue(

        )

        //Pickup:
        Trigger {primaryController.leftTriggerAxis > 0.2} .whileTrue(
            ControllerCommands(true, cube, middlePlace, floor, chute, pickAndPlace)

        )
        //Place:
        Trigger {primaryController.rightTriggerAxis > 0.2} .whileTrue(
            ParallelCommandGroup(
                ControllerCommands(false, cube, middlePlace, floor, chute, pickAndPlace),
                RunCommand({
                    cube = true // Both
                    middlePlace = true // Place
                    floor = false // Place
                    chute = false // Pickup

                    // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
                    ground = true // Pickup
                    cone = false // Both
                    highPlace = false
                })
            )
        )


        //SECONDARY CONtROLLER

        //A = Set Middle Place
        //Y = Set High place
        //X = Set Cube
        //B = Set Cone
        //Bottom D-Pad = Set Floor Place
        //Top D-Pad = Set Chute Pickup
        //Left D-Pad = Set Ground Pickup

        //Voltage Controls
        //Left Bumper = Intake In
        //Right Bumper = Intake Out
        //Left Trigger = Elevator Down
        //Right Trigger = Elevator Up
        //Left Joystick = Elbow(Arm)
        //Right Joystick = Wrist

        //Set Middle Place
        JoystickButton(secondaryController, XboxController.Button.kA.value).whileTrue(
            RunCommand({
                middlePlace = true
                middlePlaceNT.set(true)

                highPlace = false
                highPlaceNT.set(false)
            })
        )

        //Set High Place
        JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                middlePlace = false
                middlePlaceNT.set(false)

                highPlace = true
                highPlaceNT.set(true)
            })
        )

        //Set Cube
        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                cube = true
                cubeNT.set(true)

                cone = false
                coneNT.set(false)
            })
        )

        //Set Cone
        JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            RunCommand({
                cube = false
                cubeNT.set(false)

                cone = true
                coneNT.set(true)
            })
        )

        //Set Floor Place
        POVButton(primaryController, 180).whileTrue(
            RunCommand({
                floor = true
                floorNT.set(true)

                middlePlace = false
                middlePlaceNT.set(false)

                highPlace = false
                highPlaceNT.set(false)
            })
        )

        //Set Chute Pickup
        POVButton(primaryController, 0).whileTrue(
            RunCommand({
                chute = true
                chuteNT.set(true)

                ground = false
                groundNT.set(false)

                cone = true
                coneNT.set(true)

                cube = false
                cubeNT.set(false)
            })
        )

        //Set Ground Pickup
        POVButton(primaryController, 270).whileTrue(
            RunCommand({
                chute = false
                chuteNT.set(false)

                ground = true
                groundNT.set(true)
            })
        )

        //Voltage Controls
        //Left Bumper = Intake In
        //Right Bumper = Intake Out
        //Left Trigger = Elevator Down
        //Right Trigger = Elevator Up
        //Left Joystick = Elbow(Arm)
        //Right Joystick = Wrist
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
    }

    private fun configureButtonBindings() {
        //TESTING CONTROLS:

        swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * DrivetrainConstants.drivingSpeedScalar / 2.0 },
            { primaryController.leftX * DrivetrainConstants.drivingSpeedScalar / 2.0},
            { primaryController.rightX * DrivetrainConstants.rotationSpeedScalar  / 2.0},
            true,
            true)


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
        Trigger {primaryController.leftTriggerAxis > 0.2} .whileTrue(
            HighPlaceCone(pickAndPlace)
        )
        Trigger {primaryController.rightTriggerAxis > 0.2} .whileTrue(
            ChutePick(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kLeftBumper.value).whileTrue(
            MidPlaceCone(pickAndPlace)

        )
        JoystickButton(primaryController, XboxController.Button.kRightBumper.value).whileTrue(
            LowPickCone(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kB.value).whileTrue(
            ShelfPick(pickAndPlace)
        )
        JoystickButton(primaryController, XboxController.Button.kA.value).whileTrue(
            FloorPlace(pickAndPlace)
            //LowPickConeSide(pickAndPlace)
        )
    }

    val autonomousCommand: Command = RunCommand({testTrajectories.RedCenter1Balance()})
}