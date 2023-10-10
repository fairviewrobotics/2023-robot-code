package frc.robot.constants

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.trajectory.TrapezoidProfile


object ArmConstants {
    /** make a network table value in table Arm and value ElbowPosition**/

    val elevatorMotorId = 19
    val elbowMotorId = 10
    val wristMotorId = 11
    val intakeMotorOneId = 12
    val intakeMotorTwoId = 13

    val topBreakerId = 0
    val bottomBreakerId = 1

    val topLinebreakerId = 0
    val bottomLinebreakerId = 1

    val elevatorMotorInverted = false
    val elbowMotorInverted = true
    val intakeMotorInverted = false

    //Need very small tune
    var elevatorP = 108.0 //108
    val elevatorI = 0.0
    val elevatorD = 0.0

    //val elevatorTrapezoidConstraints = TrapezoidProfile.Constraints(50.0, 30.0)
//tune:
    val elbowP = 5.1 //6
    val elbowI = 0.0
    val elbowD = 0.0

    //maybe change:
    val elbowFF = ArmFeedforward(0.13, 0.50, 0.00)//might need to change kg

    //tune:
    val wristP = 3.1 // 7
    val wristI = 0.0
    val wristD = 0.0


    var elevatorGoal = 0.0
    var elbowGoal = 0.0
    var wristGoal = 0.0

    //maybe change:
    val wristFF = ArmFeedforward(0.13, 0.27, 0.00)

    val elevatorMinHeight = 0.04
    val elevatorMaxHeight = 0.947

    val elbowMaxRotation = Math.toRadians(140.0)
    val elbowMinRotation = -(1.0 / 2.0) * Math.PI

    val wristMaxRotation = (15.0)//15
    val wristMinRotation = -(160.0)

    // multipliers for unit conversion and stuff
    // these values obtained from tuning
    val elevatorEncoderVelocityConversionFactor =
        (0.003010870139 * 2.4) / 60.0 //this should turn revs/min to meters/sec
    val elevatorEncoderPositionConversionFactor = 1.0 //this should turn revs to meters

    //could need small tuning:
    val elbowEncoderPosOffset = -1.4
    val wristEncoderPosOffset = 0.15

    val elevatorZeroingVoltage = -1.5 //-1.0
}
