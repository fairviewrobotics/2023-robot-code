package frc.robot

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.DoublePublisher


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/

    //PID stuff
    //also needs tuning
    val elevatorP = 0.1
    val elevatorI = 0.1
    val elevatorD = 0.1
    val elbowP = 0.1
    val elbowI = 0.1
    val elbowD = 0.1
    val elbowPid = PIDController(elbowP, elbowI, elbowD)
    val elevatorPid = PIDController(elevatorP,elevatorI,elevatorD)

    //arbitrary values that need tuning
    val elevatorSpeedMultiplier = 1.0 //these two vals for controlling speed
    val elbowSpeedMultiplier = 1.0

    val elevatorThreshold = 1.0
    val elbowThreshold = 1.0
    var elbowDesiredPos = 1.0 //position wanted for elbow
    var elevatorDesiredPos = 1.0 //Position wanted for elevator

    val elevatorBottomPosEnocder = 0.0 //the value when it's at the bottom for the encoder
    val elevatorTopPosEnocder = 3.0 //the value when it's at the top for the encoder

    //network tables


    //feedforward stuff
    //More arbitrary values!!!
    val elbowkA = 0.0
    val elbowkG = 0.0
    val elbowkS = 0.0
    val elbowkV = 0.0

    val elevatorkA = 0.0
    val elevatorkV = 0.0
    val elevatorkS = 0.0
    val elevatorkG = 0.0

    var elbowfeedforward = ArmFeedforward(elbowkS, elbowkG, elbowkV, elbowkA)
    var elevatorfeedforward = ElevatorFeedforward(elevatorkS, elevatorkG, elevatorkV, elevatorkA)

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

}