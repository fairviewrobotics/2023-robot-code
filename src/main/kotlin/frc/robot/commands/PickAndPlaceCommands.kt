package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.RobotContainer
import frc.robot.constants.ArmConstants
import frc.robot.constants.CommandValues
import frc.robot.subsystems.PickAndPlaceSubsystem
import frc.robot.subsystems.SwerveSubsystem

class SetPickAndPlacePosition(val continuous: Boolean ,val subsystem: PickAndPlaceSubsystem,
                          val elevatorPos: Double,
                          val elbowRot: Double,
                          val wristRot: Double,
                          val intakeVolts: () -> Double,
                          ) : CommandBase() {
    var elevatorPid = ProfiledPIDController(
        ArmConstants.elevatorP,
        ArmConstants.elevatorI,
        ArmConstants.elevatorD,
        TrapezoidProfile.Constraints(2.5, 1.7)//3.0, 0.8
    )
    var elbowPid = ProfiledPIDController(
        ArmConstants.elbowP,
        ArmConstants.elbowI,
        ArmConstants.elbowD,
        TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 5) //I think might need to be slowed down, cause robot can drive while the elbow is moving as long as elevator is down
    )

    var wristPid = ProfiledPIDController(
        ArmConstants.wristP,
        ArmConstants.wristI,
        ArmConstants.wristD,
        TrapezoidProfile.Constraints(Math.PI * 3, Math.PI * 5)
    )
    val elbowFeedforward = ArmConstants.elbowFF

    val wristFeedforward = ArmConstants.wristFF

    init {
        addRequirements(subsystem)
    }//This might need to be above the variables

    override fun initialize() {
        elevatorPid.setTolerance(0.04)//might need changing
        wristPid.setTolerance(0.05)//might need changing
        elbowPid.setTolerance(0.06)//might need changing

        elevatorPid.reset(subsystem.elevatorPositionMeters)
        elbowPid.reset(subsystem.elbowPositionRadians)
        wristPid.reset(subsystem.absoluteWristPosition)

        elevatorPid.setGoal(elevatorPos.coerceIn(ArmConstants.elevatorMinHeight, ArmConstants.elevatorMaxHeight))
        elbowPid.setGoal(elbowRot.coerceIn(ArmConstants.elbowMinRotation, ArmConstants.elbowMaxRotation))
        wristPid.setGoal(wristRot.coerceIn(ArmConstants.wristMinRotation, ArmConstants.wristMaxRotation))
    }

    object Telemetry{
        val nt = NetworkTableInstance.getDefault().getTable("DriverControl")

        var cubeNT = nt.getBooleanTopic("Cube").publish() // Both
        var middlePlaceNT = nt.getBooleanTopic("Middle Place").publish() // Place
        var floorNT = nt.getBooleanTopic("Floor").publish() // Place
        var chuteNT = nt.getBooleanTopic("Chute Pickup").publish() // Pickup
        var pickupNT = nt.getBooleanTopic("Pickup").publish()

        // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
        var groundNT = nt.getBooleanTopic("Ground Pickup").publish() // Pickup
        var coneNT = nt.getBooleanTopic("Cone").publish() // Both
        var highPlaceNT = nt.getBooleanTopic("High Place").publish() // Place


        val desiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElbow").publish()
        val desiredWrist = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredWrist").publish()
        val desiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElevator").publish()
        val desiredIntake = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredIntake").publish()

        val elbowPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPidError").publish()
        val elevatorPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPidError").publish()
        val wristPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPidError").publish()

        val elbowPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPidError").publish()
        val elevatorPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPidError").publish()
        val wristPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPidError").publish()

        val elbowGoal = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("Elbow Goal").publish()


    }

    override fun execute() {

        // store a lastValue for each of these, compare, and then set/reset if they are changed

        subsystem.elevatorVoltage = elevatorPid.calculate(subsystem.elevatorPositionMeters)
        subsystem.elbowVoltage = elbowPid.calculate(subsystem.elbowPositionRadians) + elbowFeedforward.calculate(subsystem.elbowPositionRadians, 0.0)
        subsystem.wristVoltage = wristPid.calculate(subsystem.absoluteWristPosition) + wristFeedforward.calculate(subsystem.absoluteWristPosition, 0.0)
        subsystem.intakesVoltage = intakeVolts()


        // TODO: Add telemetry: desired states (from suppliers), pid errors, voltages
        Telemetry.desiredWrist.set(wristRot)
        Telemetry.desiredElbow.set(elbowRot)
        Telemetry.desiredElevator.set(elevatorPos)
        Telemetry.desiredIntake.set(intakeVolts())
        Telemetry.elbowPidPosError.set(elbowPid.positionError)
        Telemetry.elevatorPidPosError.set(elevatorPid.positionError)
        Telemetry.elbowPidVelocityError.set(elbowPid.velocityError)
        Telemetry.elevatorPidVelocityError.set(elevatorPid.velocityError)
        Telemetry.wristPidVelocityError.set(wristPid.velocityError)

        Telemetry.elbowGoal.set(elbowPid.goal.position)

        Telemetry.cubeNT.set(CommandValues.cube)
        Telemetry.coneNT.set(CommandValues.cone)
        Telemetry.floorNT.set(CommandValues.floor)
        Telemetry.chuteNT.set(CommandValues.chute)
        Telemetry.pickupNT.set(CommandValues.pickup)
        Telemetry.middlePlaceNT.set(CommandValues.middlePlace)
        Telemetry.highPlaceNT.set(CommandValues.highPlace)
        Telemetry.groundNT.set(CommandValues.ground)

    }

    override fun isFinished(): Boolean {
        return !continuous && (elbowPid.atGoal() && elevatorPid.atGoal() && wristPid.atGoal())
    }

    //TODO: Could be messing with it:
    override fun end(interrupted: Boolean) {
        // TODO: does zeroing the intake voltage cause the piece to drop?
        subsystem.elevatorVoltage = 0.0
        subsystem.wristVoltage = 0.0
        subsystem.elbowVoltage = 0.0
        subsystem.intakesVoltage = 0.0
    }
}

class zeroVoltage(val subsystem: PickAndPlaceSubsystem): CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.wristVoltage = 0.0
        subsystem.elevatorVoltage = 0.0
        subsystem.elbowVoltage = 0.0
        subsystem.intakesVoltage = 0.0
    }



}

//TODO: All values for functions must be tested with degrees, height, and vision on a field
//TODO:Test
fun Base(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.0, // elevator
            Math.toRadians(80.0), // elbow
            Math.toRadians(70.0), // wrist
            { 1.0 } // intake
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
        0.0, // elevator
            Math.toRadians(80.0), // elbow
            Math.toRadians(70.0), // wrist
            { 0.5 } // intake
    )
    )
}
//TODO:Test
fun FloorPlace(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
                false,
                pnp,
                0.0, // elevator
                Math.toRadians(70.0), // elbow
            Math.toRadians(-70.0), // wrist
            { 1.0 } // intake
            ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.0, // elevator
            Math.toRadians(70.0), // elbow
            Math.toRadians(-70.0), // wrist
            { -3.0 } // intake
        )
    )

    //println("running")
}
fun MidPlaceCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.2, // elevator
            Math.toRadians(60.0), // elbow
            0.0, // wrist
            { 2.0 } // intake
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.2, // elevator
            Math.toRadians(60.0), // elbow
            0.0, // wrist
            { -3.0 } // intake
        )
    )

    //println("running")
}
fun HighPlaceCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.6, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-5.0), // wrist
            { 2.0 }
        ),

        SetPickAndPlacePosition(
            true,
            pnp,
            0.6, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-5.0), // wrist
            { -3.0 }
        )
    )
}
//TODO:Tune
fun MidPlaceCone(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.0394, // elevator
            0.9527, // elbow
            0.908, // wrist
            { 0.5 } // intake
        ),

        SetPickAndPlacePosition(
            true,
            pnp,
            0.0394, // elevator
            0.9527, // elbow
            0.908, // wrist
            { -6.0 } // intake
        )
    )
}
//TODO:Tune
fun HighPlaceCone(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.88, // elevator
            Math.toRadians(55.0), // elbow
            Math.toRadians(55.0), // wrist
            { 1.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.88, // elevator
            Math.toRadians(33.0), // elbow
            Math.toRadians(-26.0), // wrist
            { 1.0 }
        ),

        SetPickAndPlacePosition(
            true,
            pnp,
            0.88, // elevator
            Math.toRadians(33.0), // elbow
            Math.toRadians(-26.0), // wrist
            { if (controller.rightTriggerAxis < 0.8) 0.0 else -5.0  }
        )
    )
}
//fun LowPickConeBackup(pnp: PickAndPlaceSubsystem): Command {
//    return SequentialCommandGroup(
//        SetPickAndPlacePosition(
//            true,
//            pnp,
//            0.4, // elevator
//            Math.toRadians(-31.0), // elbow
//            Math.toRadians(-60.0), // wrist
//            7.0 // intake
//        )
//    )
//}
//TODO: Tune
fun LowPickCone(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.55, // elevator
            Math.toRadians(0.0), // elbow
            Math.toRadians(0.0), // wrist
            { 0.0 } // intake
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.55, // elevator
            Math.toRadians(-80.0), // elbow
            Math.toRadians(0.0), // wrist
            { 7.0 } // intake
        )
    )
}
fun LowPickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            true,
            pnp,
            0.3, // elevator
            Math.toRadians(-28.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 5.0 } // intake
        )
    )
}
fun ChutePick(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
       SetPickAndPlacePosition(
            true,
            pnp,
            0.0, // elevator
            1.03475, // elbow
            0.92208, // wrist
           { 1.0 } // intake
        )
    )
}
//TODO:Tune
fun AutoPickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.3, // elevator
            Math.toRadians(-28.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 5.0 } // intake
        )
    )
}
fun AutoPlaceMid(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.2, // elevator
            Math.toRadians(60.0), // elbow
            0.0, // wrist
            { 1.0 } // intake
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.2, // elevator
            Math.toRadians(60.0), // elbow
            0.0, // wrist
            { -3.0 } // intake
        )
    )
}
//TODO:Test
fun AutoPlaceHigh(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.6, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-5.0), // wrist
            { 1.0 }
        ),

        SetPickAndPlacePosition(
            false,
            pnp,
            0.6, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-5.0), // wrist
            { -3.0 }
        )
    )
}

class VoltageArm(val subsystem: PickAndPlaceSubsystem,
                 val elevatorSupplier: ()->Double,
                 val elbowSupplier: ()->Double,
                 val wristSupplier:()->Double,
                 val intakeSupplier: ()->Double) : CommandBase() {
    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.elevatorVoltage = elevatorSupplier()
        subsystem.elbowVoltage = elbowSupplier()
        subsystem.wristVoltage = wristSupplier()
        subsystem.intakesVoltage = intakeSupplier()
    }

    override fun end(interrupted: Boolean) {

        subsystem.elevatorVoltage = 0.0
        subsystem.wristVoltage = 0.0
        subsystem.elbowVoltage = 0.0
        subsystem.intakesVoltage = 0.0
    }


}
