package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.ArmConstants
import frc.robot.subsystems.ArmSubsystem

class armSetPosition(val armsubsystem: ArmSubsystem, val elbowPosition: Double, val elevatorPosition: Double) :CommandBase()
{
    init {
        armsubsystem.setDesired(elbowPosition, elevatorPosition)
    }

    override fun execute() {

    }

    override fun isFinished(): Boolean {
        return (ArmConstants.elevatorPid.atSetpoint() && ArmConstants.elbowPid.atSetpoint())
    //if both parts have reached their desired
    }

    override fun end(interrupted: Boolean) {
    }
}

class gotoHome(val armsubsystem: ArmSubsystem) :CommandBase() //this one goes into the folded-up possition (all the way down for both)
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowBottomPos, ArmConstants.elevatorBottomPos)
    }


}

class gotoTopPeg(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the top peg
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowTopPegPos, ArmConstants.elevatorTopPegPos)
    }


}

class gotoMiddlePeg(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the middle peg
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowMidPegPos, ArmConstants.elevatorMidPegPos)
    }


}

class gotoGround(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the ground.
{ //This can be used to pick up cones from the ground and put them on the bottom shelf, at least for now...


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowGroundPos, ArmConstants.elevatorGroundPos)
    }


}

class gotoMiddleShelf(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the middle shelf
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowMidShelfPos, ArmConstants.elevatorMidShelfPos)
    }


}

class gotoTopShelf(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the top shelf
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowTopShelfPos, ArmConstants.elevatorTopShelfPos)
    }


}

class gotoPickup(val armsubsystem: ArmSubsystem) :CommandBase() //This one goes to the pickup shelf
{


    override fun execute() {
        armSetPosition(armsubsystem, ArmConstants.elbowPickupPos, ArmConstants.elevatorPickupPos)
    }


}