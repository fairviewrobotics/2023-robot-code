package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.SwerveSubsystem

class UnlimitedDrive(private val swerveSubsystem: SwerveSubsystem, val forward: () -> Double, val sideways: () -> Double, val radians: () -> Double, val fieldRelative: Boolean) : CommandBase() {

    init {
        addRequirements(swerveSubsystem)
    }

    override fun execute() {
        val forwardDesired = MathUtil.applyDeadband(forward(), 0.06)
        val sidewaysDesired = MathUtil.applyDeadband(sideways(), 0.06)
        val radiansDesired = MathUtil.applyDeadband(radians(), 0.06)

        swerveSubsystem.drive(forwardDesired, sidewaysDesired, -1 * radiansDesired, true, false)
    }

    override fun end(interrupted: Boolean) {
        swerveSubsystem.drive(0.0,0.0,0.0,true, false)
    }
}

