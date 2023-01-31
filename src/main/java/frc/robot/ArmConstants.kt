package frc.robot

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.DoublePublisher


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/
    val elbowmotorID = 9
    val elevatormotorID = 10


    //arbitrary values that need tuning
    val elevatorSpeedMultiplier = 1.0 //these two vals for controlling speed
    val elbowSpeedMultiplier = 1.0

    //these two vals are how far it's allowed to go.
    val elevatorThreshold = 1.0
    val elbowThreshold = 1.0

    var elbowDesiredPos = 1.0 //position wanted for elbow
    var elevatorDesiredPos = 1.0 //Position wanted for elevator

    val elevatorBottomPosEnocder = 0.0 //the value when it's at the bottom for the encoder
    val elevatorTopPosEnocder = 3.0 //the value when it's at the top for the encoder




}