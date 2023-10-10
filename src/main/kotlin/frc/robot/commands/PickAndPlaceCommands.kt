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
                              val intakeTwoVolts: () -> Double,
                              val intakeOneVolts: () -> Double,
) : CommandBase() {
    var elevatorPid = ProfiledPIDController(
        ArmConstants.elevatorP,
        ArmConstants.elevatorI,
        ArmConstants.elevatorD,
        TrapezoidProfile.Constraints(2.5, 1.7)//2.5, 1.7
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

        elevatorPid.p = ArmConstants.elevatorP
        elbowPid.p = ArmConstants.elbowP
        wristPid.p = ArmConstants.wristP

        // store a lastValue for each of these, compare, and then set/reset if they are changed

        subsystem.elevatorVoltage = elevatorPid.calculate(subsystem.elevatorPositionMeters)
        subsystem.elbowVoltage = elbowPid.calculate(subsystem.elbowPositionRadians) + elbowFeedforward.calculate(subsystem.elbowPositionRadians, 0.0)
        subsystem.wristVoltage = wristPid.calculate(subsystem.absoluteWristPosition) + wristFeedforward.calculate(subsystem.absoluteWristPosition, 0.0)
        subsystem.intakeOnesVoltage = intakeOneVolts()
        subsystem.intakeTwosVoltage = intakeTwoVolts()


        // TODO: Add telemetry: desired states (from suppliers), pid errors, voltages
        Telemetry.desiredWrist.set(wristRot)
        Telemetry.desiredElbow.set(elbowRot)
        Telemetry.desiredElevator.set(elevatorPos)
        Telemetry.desiredIntake.set(intakeOneVolts())
        Telemetry.desiredIntake.set(intakeTwoVolts())
        Telemetry.elbowPidPosError.set(elbowPid.positionError)
        Telemetry.elevatorPidPosError.set(elevatorPid.positionError)
        Telemetry.elbowPidVelocityError.set(elbowPid.velocityError)
        Telemetry.elevatorPidVelocityError.set(elevatorPid.velocityError)
        Telemetry.wristPidVelocityError.set(wristPid.velocityError)

        Telemetry.elbowGoal.set(elbowPid.goal.position)



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
        subsystem.intakeOnesVoltage = 0.0
        subsystem.intakeTwosVoltage = 0.0
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
        subsystem.intakeOnesVoltage = 0.0
        subsystem.intakeTwosVoltage = 0.0
    }



}

//TODO: All values for functions must be tested with degrees, height, and vision on a field
//TODO:Test
fun Base(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(73.0), // elbow 60
            Math.toRadians(-10.0), // wrist -100
            { if (!CommandValues.cube) 1.0 else 0.0 }, //in
            { if (!CommandValues.cube) -1.2 else 0.0 } //in
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(120.0), // elbow 98
            Math.toRadians(-20.0), // wrist -100
            { if (!CommandValues.cube) 1.0 else 0.0 }, //in
            { if (!CommandValues.cube) -1.2 else 0.0 } //in
        )
    )
}
fun AutoBase(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(73.0), // elbow
            Math.toRadians(-10.0), // wrist
            { 0.5 }, //in
            { -0.5 } //in
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(120.0), // elbow
            Math.toRadians(-20.0), // wrist
            { 0.5 }, //in
            { -0.5 } //in
        )
    )
}
fun AutoBase2(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(73.0), // elbow
            Math.toRadians(-10.0), // wrist
            { 0.5 }, //in
            { -0.5 }// in
        )
    )
}
//TODO:Test
fun FloorPlace(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(68.0), // elbow
            Math.toRadians(-65.0), // wrist
            { if (!CommandValues.cube) 1.0 else 0.0 }, //in
            { if (!CommandValues.cube) -1.2 else 0.0 }// in
        ),
        SetPickAndPlacePosition(
                true,
        pnp,
        ArmConstants.elevatorMinHeight, // elevator
        Math.toRadians(68.0), // elbow
        Math.toRadians(-65.0), // wrist
        { if (controller.rightBumper) -6.0 else if (!CommandValues.cube) 1.0 else 0.0 }, //out then in
        { if (controller.rightBumper) 6.0 else if (!CommandValues.cube) -1.2 else 0.0 }// out then in
    )
    )

    //println("running")
}
fun MidPlaceCube(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.25, // elevator
            Math.toRadians(63.0), // elbow
            Math.toRadians(-45.0), // wrist
            { 0.0 }, //in????
            { 0.0 }// in
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.25, // elevator
            Math.toRadians(63.0), // elbow
            Math.toRadians(-45.0), // wrist
            { if (controller.rightBumper) -5.0 else 0.0  },
            { if (controller.rightBumper) 5.0 else 0.0  }// intake
        )
    )

    //println("running")
}
fun HighPlaceCube(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(80.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 0.0 },
            { 0.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.65, // elevator
            Math.toRadians(53.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 0.0 },
            { 0.0 }
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.65, // elevator
            Math.toRadians(53.0), // elbow
            Math.toRadians(-30.0), // wrist
            { if (controller.rightBumper) -5.0 else 0.0  },
            { if (controller.rightBumper) 5.0 else 0.0  }
        )
    )
}
//TODO:Tune
fun MidPlaceCone(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.37, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-50.0), // wrist
            { 5.0 },
            { -6.0 }// intake
        ),

        SetPickAndPlacePosition(
            true,
            pnp,
            0.37, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-50.0), // wrist
            { if (controller.rightBumper) -6.5 else 5.0  },
            { if (controller.rightBumper) 6.0 else -6.0  }// intake
        )
    )
}
//TODO:Tune
fun HighPlaceCone(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.15, // elevator
            Math.toRadians(70.0), // elbow
            Math.toRadians(10.0), // wrist
            { 3.0 },
            { -4.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.8, // elevator
            Math.toRadians(48.0), // elbow
            Math.toRadians(-40.0), // wrist
            { 4.0 },
            { -5.0 }
        ),

        SetPickAndPlacePosition(
            true,
            pnp,
            0.8, // elevator
            Math.toRadians(48.0), // elbow
            Math.toRadians(-40.0), // wrist
            { if (controller.rightBumper) -6.5 else 4.0  },
            { if (controller.rightBumper) 6.0 else -5.0  }
        )
    )
}
fun UprightPickCone(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(35.0), //elbow
            Math.toRadians(-45.0), // wrist
            { 1.0 },
            { -1.0  }
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.1, // elevator
            //if (controller.rightBumper) 0.261799 else 0.749066, // elbow
            Math.toRadians(-6.0), //elbow
            Math.toRadians(-45.0), // wrist
            { if (controller.rightBumper) -6.0 else 1.0  },
            { if (controller.rightBumper) 8.0 else -1.0  }
        )
    )
}
fun LowPickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.35, // elevator
            Math.toRadians(-20.0), // elbow
            Math.toRadians(-10.0), // wrist
            { 3.0 },
            { -3.0 }// intake
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.35, // elevator
            Math.toRadians(-35.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 3.0 },
            { -3.0 }// intake
        )
    )
}
fun ChutePickCone(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            true,
            pnp,
            ArmConstants.elevatorMinHeight
            , // elevator
            1.13475, // elbow
            0.83999, // wrist
            { 2.5 },
            { 2.5 }// intake
        )
    )
}
fun ChutePickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            true,
            pnp,
            ArmConstants.elevatorMinHeight
            , // elevator
            1.13475, // elbow
            0.83999, // wrist
            { 2.5 },
            { 2.5 }// intake
        )
    )
}
fun ShelfPickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            true,
            pnp,
            ArmConstants.elevatorMinHeight
            , // elevator
            Math.toRadians(-27.0), // elbow
            Math.toRadians(-27.0), // wrist
            { 2.5 },
            { 2.5 }// intake
        )
    )
}
fun ShelfPickCone(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            true,
            pnp,
            ArmConstants.elevatorMinHeight
            , // elevator
            Math.toRadians(-27.0), // elbow
            Math.toRadians(-27.0), // wrist
            { 2.5 },
            { 2.5 }// intake
        )
    )
}









fun AutoPickCube(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.35, // elevator
            Math.toRadians(-20.0), // elbow
            Math.toRadians(-10.0), // wrist
            { 3.0 },
            { -3.0 }// intake
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.35, // elevator
            Math.toRadians(-35.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 3.0 },
            { -3.0 }// intake
        )
    )
}
fun AutoPickCone(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            //if (controller.rightBumper) 0.261799 else 0.749066, // elbow
            Math.toRadians(35.0), //elbow
            Math.toRadians(-45.0), // wrist
            { 1.0 },
            { -1.0  }
        )
    )
}
fun AutoPlaceCubeMid(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.25, // elevator
            Math.toRadians(63.0), // elbow
            Math.toRadians(-45.0), // wrist
            { 0.0 }, //in????
            { 0.0 }// in
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.25, // elevator
            Math.toRadians(63.0), // elbow
            Math.toRadians(-45.0), // wrist
            { -6.0 },
            { 6.0 }// intake
        )
    )
}
//TODO:Test
fun AutoPlaceCubeHigh(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            ArmConstants.elevatorMinHeight, // elevator
            Math.toRadians(80.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 0.0 },
            { 0.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.65, // elevator
            Math.toRadians(53.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 0.0 },
            { 0.0 }
        ),
        SetPickAndPlacePosition(
            true,
            pnp,
            0.65, // elevator
            Math.toRadians(53.0), // elbow
            Math.toRadians(-30.0), // wrist
            { -6.0 },
            { 6.0 }
        )
    )
}
fun AutoPlaceConeMidGetThere(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.37, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-30.0), // wrist
            { 5.0 },
            { -6.0 }// intake
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.37, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-50.0), // wrist
            { 5.0 },
            { -6.0 }// intake
        )
    )
}
fun AutoPlaceConeMidPlace(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.37, // elevator
            Math.toRadians(60.0), // elbow
            Math.toRadians(-50.0), // wrist
            { -6.5 },
            { 6.0 }// intake
        )
    )
}

fun AutoPlaceConeHigh(pnp: PickAndPlaceSubsystem): Command {
    return SequentialCommandGroup(
        SetPickAndPlacePosition(
            false,
            pnp,
            0.15, // elevator
            Math.toRadians(70.0), // elbow
            Math.toRadians(10.0), // wrist
            { 3.0 },
            { -4.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.8, // elevator
            Math.toRadians(48.0), // elbow
            Math.toRadians(-40.0), // wrist
            { 4.0 },
            { -5.0 }
        ),
        SetPickAndPlacePosition(
            false,
            pnp,
            0.8, // elevator
            Math.toRadians(48.0), // elbow
            Math.toRadians(-40.0), // wrist
            { -6.5 },
            { 6.0 }
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
        subsystem.intakeOnesVoltage = intakeSupplier()
        subsystem.intakeTwosVoltage = intakeSupplier()
    }

    override fun end(interrupted: Boolean) {

        subsystem.elevatorVoltage = 0.0
        subsystem.wristVoltage = 0.0
        subsystem.elbowVoltage = 0.0
        subsystem.intakeOnesVoltage = 0.0
        subsystem.intakeTwosVoltage = 0.0
    }


}
