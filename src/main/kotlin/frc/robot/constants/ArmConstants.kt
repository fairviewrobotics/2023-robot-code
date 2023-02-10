package frc.robot.constants

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/
    val elbowmotorID = 9
    val elevatormotorID = 10


    //FF vals
    val elbowkA = 0.0
    val elbowkG = 0.0
    val elbowkS = 0.0
    val eElbowkV = 0.0

    var elbowFeedForward = ArmFeedforward(elbowkS, elbowkG, eElbowkV, elbowkA)

    //PID vals
    //PID stuff
    //also needs tuning
    val elevatorP = 0.1
    val elevatorI = 0.1
    val elevatorD = 0.1
    val elevatorTrapezoidConstraints = TrapezoidProfile.Constraints(3.0, 1.0)

    val elbowP = 0.1
    val elbowI = 0.1
    val elbowD = 0.1

    val elevatorMinHeight = 0.0
    val elevatorMaxHeight = 0.972


    //multipliers for unit conversion and stuff
    // these values obtained from tuning
    val elevatorEncoderVelocityMultiplier = 0.003010870139 / 60.0 //this should turn revs/min to meters/sec
    val elevatorEncoderPositionConversionFactor = 0.003010870139 //this should turn revs to meters

    val elevatorMotorInverted = false
    val intakeMotorInverted = false

    val elbowEncoderVelocityMultiplier = 2.0 //this should turn revs/min to radians/sec (Yay!)
    val elbowEncoderPosMultiplier = 2.0 //this should turn revs to radians (Yay again!)

    //all those random command values
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




}