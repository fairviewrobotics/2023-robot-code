package frc.robot

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.DoublePublisher


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/
    val ElbowmotorID = 9
    val ElevatormotorID = 10


    //arbitrary values that need tuning
    val ElevatorSpeedMultiplier = 1.0 //these two vals for controlling speed
    val ElbowSpeedMultiplier = 1.0

    //these two vals are how far it's allowed to go.
    val ElevatorThreshold = 1.0
    val ElbowThreshold = 1.0

    var ElbowDesiredPos = 1.0 //position wanted for elbow
    var ElevatorDesiredPos = 1.0 //Position wanted for elevator

    val ElevatorBottomPosEnocder = 0.0 //the value when it's at the bottom for the encoder
    val ElevatorTopPosEnocder = 3.0 //the value when it's at the top for the encoder

    //multipliers for unit conversion and stuff
    val ElevatorEncoderVelocityMultiplier = 2.0 //this should turn revs/min to meters/sec
    val ElevatorEncoderPosMultiplier = 2.0 //this should turn revs to meters
    val ElbowEncoderVelocityMultiplier = 2.0 //this should turn revs/min to radians/sec (Yay!)
    val ElbowEncoderPosMultiplier = 2.0 //this should turn revs to radians (Yay again!)

    //all those random command values
    //knowing the mechanism, these will end up being really strange values for all the way down.
//bottom
    val ElevatorBottomPos = 0.0
    val ElbowBottomPos = 0.0

    //top peg
    val ElevatorTopPegPos = 3.0
    val ElbowTopPegPos = 3.0

    //middle peg
    val ElevatorMidPegPos = 2.0
    val ElbowMidPegPos = 2.0

    //ground
    val ElevatorGroundPos = 2.0
    val ElbowGroundPos = 1.0

    //middle shelf
    val ElevatorMidShelfPos = 2.0
    val ElbowMidShelfPos = 2.0

    //top shelf
    val ElevatorTopShelfPos = 3.0
    val ElbowTopShelfPos = 3.0

    //pickup shelf position
    val ElevatorPickupPos = 3.0
    val ElbowPickupPos = 3.0




}