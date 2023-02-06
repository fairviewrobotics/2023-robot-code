package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSubsystem



class ArmSetPosition(val armSubsystem: ArmSubsystem, val elbowPosition: Double, val elevatorPosition: Double) : CommandBase() {
    override fun execute() {
        armSubsystem.SetDesired(elbowPosition, elevatorPosition)
    }
    override fun isFinished(): Boolean {
        return (armSubsystem.elevatorPid.atSetpoint() && armSubsystem.elbowPid.atSetpoint())
    //if both parts have reached their desired
    }

    override fun end(interrupted: Boolean) {
    }
}

fun GotoHome(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowBottomPos, ArmConstants.elevatorBottomPos)
}

fun GotoTopPeg(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopPegPos, ArmConstants.elevatorTopPegPos)
}

fun GotoMiddlePeg(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidPegPos, ArmConstants.elevatorMidPegPos)
}

fun GotoGround(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowGroundPos, ArmConstants.elevatorGroundPos)
}

fun GotoMiddleShelf(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidShelfPos, ArmConstants.elevatorMidShelfPos)
}

fun GotoTopShelf(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopShelfPos, ArmConstants.elevatorTopShelfPos)
}

fun GotoPickup(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowPickupPos, ArmConstants.elevatorPickupPos)
}