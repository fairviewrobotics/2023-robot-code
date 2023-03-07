package frc.robot.constants

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/
    val elbowMotorId = 10
    val elevatorMotorId = 9

    val topLinebreakerId = 0
    val bottomLinebreakerId = 1

    val elevatorMotorInverted = false
    val elbowMotorInverted = true
    val intakeMotorInverted = false

    //PID vals
    val elevatorP = 500.0 //500
    val elevatorI = 0.0
    val elevatorD = 0.0
    //val elevatorTrapezoidConstraints = TrapezoidProfile.Constraints(50.0, 30.0)
//tune:
    val elbowP = 7.0
    val elbowI = 0.0
    val elbowD = 0.0
    val elbowFF = ArmFeedforward(0.13, 2.07, 1.00)

    val wristP = 1.0
    val wristI = 0.0
    val wristD = 0.0

    val elevatorMinHeight = 0.0
    val elevatorMaxHeight = 0.9245

    val elbowMaxRotation = (1.0/2.0) * Math.PI
    val elbowMinRotation = -(1.0/2.0) * Math.PI

    val wristMaxRotation = (Math.PI/2.0)
    val wristMinRotation = -(Math.PI/2.0)

    // multipliers for unit conversion and stuff
    // these values obtained from tuning
    val elevatorEncoderVelocityConversionFactor = 0.003010870139 / 60.0 //this should turn revs/min to meters/sec
    val elevatorEncoderPositionConversionFactor = 0.003010870139 //this should turn revs to meters

     val elbowEncoderPosOffset = 0.0

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