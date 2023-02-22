package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.*
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSubsystem


/*
/**
 * Sets the position the arm should go to
 *
 * This calls the armSubsystem's SetDesired to the entered value
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem which controls the arm motors and stuff
 * @param[elevatorPosition : Double] the position you want the elevator to go to
 * @param[elbowPosition: Double] the position you want the elbow to go to
 */
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

/**
 * A class for setting the elevator encoder to zero using the linebreaks
 *
 * The elevator is slowly moved down until it hits the linebreak, after that it stops
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
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

/**
 * slowly moves the elevator up forever
 *
 * this class exists purely for debugging convenience and is dangerous because of its lack of safety.
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
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

/**
 * Moves elevator to a series of positions
 *
 * zeroes elevator then moves it to elevatorPosition then elevatorSecondPos
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 * @param[elevatorPosition: Double] the first position the elevator should go to
 * @param[elevatorSecondPos : Double] the second position the elevator should go to
 */
fun gotoPos(armSubsystem: ArmSubsystem, elevatorPosition: Double, elevatorSecondPos: Double) : Command{

    return SequentialCommandGroup(
        ZeroElevator(armSubsystem),
        ArmSetPosition(armSubsystem,elevatorPosition),
        RunCommand({}).withTimeout(1.0),
        ArmSetPosition(armSubsystem,elevatorSecondPos),
        RunCommand({}).withTimeout(1.0),
       )
}

/**
 * Not sure what this is here for, but probably goes up
 *
 * Looks like it zeroes the elevator then slowly moves up forever.
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun gotoTop(armSubsystem: ArmSubsystem): Command
{

    return SequentialCommandGroup(
        ZeroElevator(armSubsystem),
        slowlyToTop(armSubsystem)
    )
}

/**
 * Goes to the home position, also known as all the way down
 *
 * just calls ArmSetPosition to go to predetermined positions
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoHome(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowBottomPos /*,ArmConstants.elevatorBottomPos*/)
}

/**
 * Goes into the top peg position
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoTopPeg(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopPegPos/*, ArmConstants.elevatorTopPegPos*/)
}

/**
 * Goes into the middle peg position
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoMiddlePeg(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidPegPos/*, ArmConstants.elevatorMidPegPos*/)
}

/**
 * Goes to the ground
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoGround(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowGroundPos/*, ArmConstants.elevatorGroundPos*/)
}

/**
 * Goes into the middle shelf position
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoMiddleShelf(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowMidShelfPos/*, ArmConstants.elevatorMidShelfPos*/)
}

/**
 * Goes into the top shelf position
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoTopShelf(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowTopShelfPos/*, ArmConstants.elevatorTopShelfPos*/)
}

/**
 * Goes into the position for picking stuff up
 *
 * same process as GotoHome
 *
 * @param[armSubsystem: ArmSubsystem] the subsystem containing the arm
 */
fun GotoPickup(armSubsystem: ArmSubsystem): Command {

    return ArmSetPosition(armSubsystem, ArmConstants.elbowPickupPos/*, ArmConstants.elevatorPickupPos*/)
}*/