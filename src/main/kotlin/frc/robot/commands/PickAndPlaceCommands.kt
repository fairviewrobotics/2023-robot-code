package frc.robot.commands

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.PickAndPlaceSubsystem

class SetPickAndPlacePosition(val continuous: Boolean ,val subsystem: PickAndPlaceSubsystem,
                          val elevatorSupplier: ()->Double,
                          val elbowSupplier: ()->Double,
                          val wristSupplier:()->Double,
                          val intakeSupplier: ()->Double,
                          ) : CommandBase() {

    var lastElevatorSupplier = elevatorSupplier()
    var lastElbowSupplier = elevatorSupplier()
    var lastWristSupplier = elevatorSupplier()

    var elevatorPid = ProfiledPIDController(
    ArmConstants.elevatorP,
    ArmConstants.elevatorI,
    ArmConstants.elevatorD, ArmConstants.elevatorTrapezoidConstraints, 0.02)

    var elbowPid = ProfiledPIDController(
        ArmConstants.elbowP,
        ArmConstants.elbowI,
        ArmConstants.elbowD, ArmConstants.elbowTrapezoidConstraints, 0.02)
    var wristPid = ProfiledPIDController(
        ArmConstants.wristP,
        ArmConstants.wristI,
        ArmConstants.wristD, ArmConstants.wristTrapezoidConstraints, 0.02)

    val elbowFeedforward = ArmConstants.elbowFF


    init{
        elbowPid.setGoal(elbowSupplier())
        elevatorPid.setGoal(elevatorSupplier())
        wristPid.setGoal(wristSupplier())
    }


    override fun execute() {

        subsystem.elevatorVoltage = elevatorPid.calculate(subsystem.elevatorPositionMeters,elevatorSupplier())
        subsystem.elbowVoltage = elbowPid.calculate(subsystem.elbowEncoder.position,elbowSupplier())
        subsystem.wristVoltage = wristPid.calculate(subsystem.wristEncoder.position,wristSupplier())
        subsystem.intakeOneMotor.setVoltage(intakeSupplier())
        subsystem.intakeTwoMotor.setVoltage(intakeSupplier())
    }
}