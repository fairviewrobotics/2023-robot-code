package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.ArmConstants
import frc.robot.subsystems.ArmSubsystem



class ArmSetPosition(val armSubsystem: ArmSubsystem, val elbowPosition: Double, val elevatorPosition: Double) :CommandBase()
{



    init {
        armSubsystem.SetDesired(elbowPosition, elevatorPosition)
    }

    override fun execute() {

    }


    
    override fun isFinished(): Boolean {
        return (armSubsystem.elevatorPid.atSetpoint() && armSubsystem.elbowPid.atSetpoint())
    //if both parts have reached their desired
    }

    override fun end(interrupted: Boolean) {
    }
}

fun GotoHome(armSubsystem: ArmSubsystem) {
    ArmSetPosition(armSubsystem, ArmConstants.elbowBottomPos, ArmConstants.elevatorBottomPos)
}

fun gotoTopPeg(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowTopPegPos, ArmConstants.elevatorTopPegPos)
}

fun gotoMiddlePeg(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowMidPegPos, ArmConstants.elevatorMidPegPos)
}

fun gotoGround(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowGroundPos, ArmConstants.elevatorGroundPos)
}

fun gotoMiddleShelf(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowMidShelfPos, ArmConstants.elevatorMidShelfPos)
}

fun gotoTopShelf(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowTopShelfPos, ArmConstants.elevatorTopShelfPos)
}

fun gotoPickup(armSubsystem: ArmSubsystem)
{
    ArmSetPosition(armSubsystem, ArmConstants.elbowPickupPos, ArmConstants.elevatorPickupPos)
}