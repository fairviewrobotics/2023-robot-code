package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.ArmConstants
import frc.robot.subsystems.ArmSubsystem



class ArmSetPosition(val ArmSubsystem: ArmSubsystem, val ElbowPosition: Double, val ElevatorPosition: Double) :CommandBase()
{



    init {
        ArmSubsystem.SetDesired(ElbowPosition, ElevatorPosition)
    }

    override fun execute() {

    }


    
    override fun isFinished(): Boolean {
        return (ArmSubsystem.ElevatorPid.atSetpoint() && ArmSubsystem.ElbowPid.atSetpoint())
    //if both parts have reached their desired
    }

    override fun end(interrupted: Boolean) {
    }
}

fun GotoHome(ArmSubsystem: ArmSubsystem) {
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowBottomPos, ArmConstants.ElevatorBottomPos)
}

fun GotoTopPeg(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowTopPegPos, ArmConstants.ElevatorTopPegPos)
}

fun GotoMiddlePeg(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowMidPegPos, ArmConstants.ElevatorMidPegPos)
}

fun GotoGround(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowGroundPos, ArmConstants.ElevatorGroundPos)
}

fun GotoMiddleShelf(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowMidShelfPos, ArmConstants.ElevatorMidShelfPos)
}

fun GotoTopShelf(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowTopShelfPos, ArmConstants.ElevatorTopShelfPos)
}

fun gotoPickup(ArmSubsystem: ArmSubsystem)
{
    ArmSetPosition(ArmSubsystem, ArmConstants.ElbowPickupPos, ArmConstants.ElevatorPickupPos)
}