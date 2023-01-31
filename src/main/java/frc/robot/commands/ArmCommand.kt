package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.ArmConstants
import frc.robot.subsystems.ArmSubsystem

class armSetPosition(val armSubsystem: ArmSubsystem, val elbowPosition: Double, val elevatorPosition: Double) :CommandBase()
{
    //These values are for the commands
    //knowing the mechanism, these will end up being really strange values for all the way down.
    //bottom
    val elevatorBottomPos = 0.0
    val elbowBottomPos = 0.0

    //top peg
    val elevatorTopPegPos = 3.0
    val elbowTopPegPos = 3.0

    //middle peg
    val elevatorMidPegPos = 2.0
    val elbowMidPegPos = 2.0

    //ground
    val elevatorGroundPos = 2.0
    val elbowGroundPos = 1.0

    //middle shelf
    val elevatorMidShelfPos = 2.0
    val elbowMidShelfPos = 2.0

    //top shelf
    val elevatorTopShelfPos = 3.0
    val elbowTopShelfPos = 3.0

    //pickup shelf position
    val elevatorPickupPos = 3.0
    val elbowPickupPos = 3.0


    init {
        armSubsystem.setDesired(elbowPosition, elevatorPosition)
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

fun gotoHome()
{
    return desired(ArmSubsystem, 1.0, 2.0)}

fun gotoTopPeg()
{
    gotoAnyPos(elbowTopPegPos, elevatorTopPegPos)
}

fun gotoMiddlePeg()
{
    gotoAnyPos(elbowMidPegPos, elevatorMidPegPos)
}

fun gotoGround()
{
    gotoAnyPos(elbowGroundPos, elevatorGroundPos)
}

fun gotoMiddleShelf()
{
    gotoAnyPos(elbowMidShelfPos, elevatorMidShelfPos)
}

fun gotoTopShelf()
{
    gotoAnyPos(elbowTopShelfPos, elevatorTopShelfPos)
}

fun gotoPickup()
{
    gotoAnyPos(elbowPickupPos, elevatorPickupPos)
}
class desired(arm, elbowpos, elevatorpos) {
    execute()
    isFinished()
}


fun gotoPickup() {
    return desired(arm, 1.0, 2.0)
}