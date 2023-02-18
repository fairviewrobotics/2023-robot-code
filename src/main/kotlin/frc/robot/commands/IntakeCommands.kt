package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.constants.IntakeConstants
import frc.robot.subsystems.IntakeSubsystem

/** Run the intake motors and keep a desired pitch.
 * @param intake The intake subsystem to run this command on.
 * @param intakeSpeed This function will be called periodically and used to
 * determine the voltage to supply to the intake motors.
 * @param pitch This function will be called periodically and used to
 * determine the pitch the intake will be at.
 * @param absolute If this is true, the pitch will be the angle the intake makes with
 * the ground. If this is false, the pitch will be the angle the intake makes with the arm.
 *
 * Warning: This command has not been tested and/or tuned.**/
class RunIntakeWithPitch(val intake: IntakeSubsystem, val intakeVoltage: () -> Double, val pitch: () -> Double, val absolute: Boolean): CommandBase() {

    val intakePID = PIDController(IntakeConstants.pitchP, IntakeConstants.pitchI, IntakeConstants.pitchD)

    init {
        addRequirements(intake)
        // 5 degrees ~= 0.08 radians
        intakePID.setTolerance(0.08)
        intakePID.enableContinuousInput(0.0, 2.0 * Math.PI)
    }

    override fun execute() {
        val pitchToSet = pitch().coerceIn(0.0, 2.0 * Math.PI)
        val intakeToSet = intakeVoltage()

        val pitchObserved = if (absolute) {
            intake.absolutePitch
        } else {
            intake.relativePitch
        }

        val pitchOutput = intakePID.calculate(pitchObserved, pitchToSet)

        intake.intakeVoltage = intakeToSet
        intake.pitchVoltage = pitchToSet
    }

    override fun end(interrupted: Boolean) {
        intake.intakeVoltage = 0.0
        intake.pitchVoltage = 0.0
    }
}

/** Functionally very similar to [RunIntakeWithPitch], but rather than supplying a radians value to
 * pitch the intake at, we set a voltage to spin the intake motors at.
 *
 * Warning: This command has not been tested and/or tuned.
 */
class RunIntakeWithPitchVoltage(val intake: IntakeSubsystem, val intakeVoltage: () -> Double, val pitchVoltage: () -> Double): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun execute() {
        val pitchToSet = pitchVoltage()
        val intakeToSet = intakeVoltage()

        intake.intakeVoltage = intakeToSet
        intake.pitchVoltage = pitchToSet
    }

    override fun end(interrupted: Boolean) {
        intake.intakeVoltage = 0.0
        intake.pitchVoltage = 0.0
    }
}