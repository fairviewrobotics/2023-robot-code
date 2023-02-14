package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.*
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSubsystem



class ArmSetPosition(val armSubsystem: ArmSubsystem, /*val elbowPosition: Double,*/ val elevatorPosition: Double) : CommandBase() {

    init{

    }
    override fun initialize() {
        println("set")

        if(elevatorPosition > 0 || elevatorPosition < 0.85) {
            armSubsystem.SetDesired(/*elbowPosition,*/ elevatorPosition)
        }
    }
    override fun isFinished(): Boolean {
        if(armSubsystem.elevatorPid.atGoal() /*&& armSubsystem.elbowPid.atSetpoint()*/) {
            println(armSubsystem.desiredElevatorState)
            println(armSubsystem.elevatorPid.atSetpoint())
            println(armSubsystem.elevatorPid.setpoint.position)
            //armSubsystem.elevatorPid.setGoal(0.0)
            return true
        }
        else
        {
            return false
        }
    //if both parts have reached their desired
    }

    override fun end(interrupted: Boolean) {
        //armSubsystem.elevatorPid.reset(0.0,0.0)
        //armSubsystem.elevatorPid.p = ArmConstants.elevatorP
       // armSubsystem.elevatorPid.setConstraints(ArmConstants.elevatorTrapezoidConstraints)

    }
}

class ZeroElevator(val armSubsystem : ArmSubsystem): CommandBase()
{
    init{
    addRequirements(armSubsystem)
    }
    override fun initialize() {
        super.initialize()
        if(armSubsystem.elevatorZeroed == false) {
            println("first if")
            if (armSubsystem.reverseLimit.get()) {
                println("second if")
                armSubsystem.moveVoltage(-2.0) //positive voltage = up
            } else {
                armSubsystem.moveVoltage(0.0)
            }
        }
    }

    override fun isFinished(): Boolean {
        if(armSubsystem.elevatorZeroed == true)
        {
            println("zeroed true finish")
            return true
        }
        if(!armSubsystem.reverseLimit.get()) {
            println("zeroed hit finish")
            armSubsystem.moveVoltage(0.0)
            armSubsystem.elevatorEncoder.position = ArmConstants.elevatorBottomPos
            armSubsystem.elevatorPid.reset(0.0,0.0)
            armSubsystem.elevatorZeroed = true
            armSubsystem.elevatorPid.setGoal(armSubsystem.elevatorEncoder.position)
            return true
        }
        else
        {
            return false
        }
    }
}

class slowlyToTop(val armSubsystem: ArmSubsystem) :CommandBase()
{
    init{
        addRequirements(armSubsystem)
    }

    override fun initialize() {
        super.initialize()
        armSubsystem.moveVoltage(1.0)
    }

    //override fun isFinished(): Boolean {
        /*if(armSubsystem.elevatorZeroed == true)
        {
            println("zeroed true finish")
            return true
        }
        if(!armSubsystem.reverseLimit.get()) {
            println("zeroed hit finish")
            armSubsystem.moveVoltage(0.0)
            armSubsystem.elevatorEncoder.position = ArmConstants.elevatorBottomPos
            armSubsystem.elevatorZeroed = true
            return true
        }
        else
        {
            return false
        }*/
    //}
    override fun isFinished() : Boolean
    {
        return false
    }
}

fun gotoPos(armSubsystem: ArmSubsystem, elevatorPosition: Double, elevatorSecondPos: Double) : Command{
    return SequentialCommandGroup(
        ZeroElevator(armSubsystem),
        ArmSetPosition(armSubsystem,elevatorPosition),
        RunCommand({}).withTimeout(1.0),
        ArmSetPosition(armSubsystem,elevatorSecondPos),
        RunCommand({}).withTimeout(1.0),
       )
}

fun gotoTop(armSubsystem: ArmSubsystem): Command
{
    return SequentialCommandGroup(
        ZeroElevator(armSubsystem),
        slowlyToTop(armSubsystem)
    )
}
fun GotoHome(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowBottomPos /*,ArmConstants.elevatorBottomPos*/)
}

fun GotoTopPeg(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopPegPos/*, ArmConstants.elevatorTopPegPos*/)
}

fun GotoMiddlePeg(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidPegPos/*, ArmConstants.elevatorMidPegPos*/)
}

fun GotoGround(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowGroundPos/*, ArmConstants.elevatorGroundPos*/)
}

fun GotoMiddleShelf(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidShelfPos/*, ArmConstants.elevatorMidShelfPos*/)
}

fun GotoTopShelf(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopShelfPos/*, ArmConstants.elevatorTopShelfPos*/)
}

fun GotoPickup(armSubsystem: ArmSubsystem): Command {
    return ArmSetPosition(armSubsystem, ArmConstants.elbowPickupPos/*, ArmConstants.elevatorPickupPos*/)
}