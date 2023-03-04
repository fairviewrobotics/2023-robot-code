package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.PickAndPlaceSubsystem

class SetPickAndPlacePosition(val continuous: Boolean ,val subsystem: PickAndPlaceSubsystem,
                          val elevatorSupplier: ()->Double,
                          val elbowSupplier: ()->Double,
                          val wristSupplier:()->Double,
                          val intakeSupplier: ()->Double,
                          ) : CommandBase() {
    init {
        addRequirements(subsystem)
    }
    var elevatorPid = PIDController(
    ArmConstants.elevatorP,
    ArmConstants.elevatorI,
    ArmConstants.elevatorD)

    var elbowPid = PIDController(
        ArmConstants.elbowP,
        ArmConstants.elbowI,
        ArmConstants.elbowD, )
    val elbowFeedforward = ArmConstants.elbowFF

    object Telemetry{
        val desiredElbow = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElbow").publish()
        val desiredWrist = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredWrist").publish()
        val desiredElevator = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredElevator").publish()
        val desiredIntake = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("DesiredIntake").publish()

        val elbowPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPidError").publish()
        val elevatorPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPidError").publish()
        val wristPidPosError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPidError").publish()

        val elbowPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowPidError").publish()
        val elevatorPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorPidError").publish()
        val wristPidVelocityError = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristPidError").publish()

        val elbowVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElbowVolts").publish()
        val wristVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("WristVolts").publish()
        val elevatorVoltage = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("ElevatorVolts").publish()
    }

    override fun execute() {

        subsystem.elevatorVoltage = elevatorPid.calculate(subsystem.elevatorPositionMeters,elevatorSupplier().coerceIn(ArmConstants.elevatorMinHeight, ArmConstants.elevatorMaxHeight))
        subsystem.elbowVoltage = elbowPid.calculate(subsystem.elbowPositionRadians,elbowSupplier().coerceIn(ArmConstants.elbowMinRotation, ArmConstants.elbowMaxRotation)) + elbowFeedforward.calculate(subsystem.elbowPositionRadians, Math.PI)
        subsystem.wristVoltage = wristSupplier()
        subsystem.intakesVoltage = intakeSupplier()

        // TODO: Add telemetry: desired states (from suppliers), pid errors, voltages
        Telemetry.desiredWrist.set(wristSupplier())
        Telemetry.desiredElbow.set(elbowSupplier())
        Telemetry.desiredElevator.set(elevatorSupplier())
        Telemetry.desiredIntake.set(intakeSupplier())
        Telemetry.elbowPidPosError.set(elbowPid.positionError)
        Telemetry.elevatorPidPosError.set(elevatorPid.positionError)
        Telemetry.elbowPidVelocityError.set(elbowPid.velocityError)
        Telemetry.elevatorPidVelocityError.set(elevatorPid.velocityError)
        Telemetry.elbowVoltage.set(subsystem.elbowVoltage)
        Telemetry.wristVoltage.set(subsystem.wristVoltage)
        Telemetry.elevatorVoltage.set(subsystem.elevatorVoltage)
    }

    override fun isFinished(): Boolean {
        return !continuous && (elbowPid.atSetpoint() && elevatorPid.atSetpoint())
    }
}

fun NTPnP(pnp: PickAndPlaceSubsystem, controller: XboxController): Command {
    var table = NetworkTableInstance.getDefault().getTable("NtPnP positions")

    return SetPickAndPlacePosition(
        true,
        pnp,
        {
            table.getDoubleTopic("Elevator").subscribe(0.0).get()
        }, // elevator
        { table.getDoubleTopic("Elbow").subscribe(0.0).get() }, // eblbow
        { table.getDoubleTopic("Wrist").subscribe(0.0).get() }, // wrist
        { controller.leftTriggerAxis * 12.0 } // intake
    )
}

class VoltageControlPNP(val pnp: PickAndPlaceSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(pnp)
    }

    override fun execute() {
        pnp.intakesVoltage = if (controller.leftBumper) {
            12.0
        } else if (controller.rightBumper) {
            -12.0
        } else {
            0.0
        }

        pnp.elevatorVoltage = controller.leftY * 12.0
        pnp.elbowVoltage = controller.rightY * 12.0
        pnp.wristVoltage = (controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0
    }

    override fun end(interrupted: Boolean) {
        pnp.intakesVoltage = 0.0
        pnp.elevatorVoltage = 0.0
        pnp.elbowVoltage = 0.0
        pnp.wristVoltage = 0.0
    }
}