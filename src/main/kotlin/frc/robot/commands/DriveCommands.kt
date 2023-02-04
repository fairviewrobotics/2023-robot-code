package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.DrivetrainConstants
import frc.robot.subsystems.SwerveSubsystem

class StandardDrive(val swerve: SwerveSubsystem, val forward: () -> Double, val sideways: () -> Double, val radians: () -> Double, val fieldRelative: Boolean) : CommandBase() {
    var radianLimiter = SlewRateLimiter(1.0, 1.0, 0.0)
    init {
        addRequirements(swerve)
    }

    override fun execute() {
        val forwardDesired = MathUtil.applyDeadband(forward(), 0.06)
        val sidewaysDesired = MathUtil.applyDeadband(sideways(), 0.06)
        val radiansDesired = radianLimiter.calculate(MathUtil.applyDeadband(radians(), 0.06))

        swerve.drive(forwardDesired, sidewaysDesired, radiansDesired, fieldRelative)
    }

    override fun end(interrupted: Boolean) {
        swerve.drive(0.0,0.0,0.0,false)
    }
}