package frc.robot.commands

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import frc.robot.AutoBalance

class Balance(val swerveSubsystem: SwerveSubsystem) : CommandBase() {
    init {
        addRequirements(swerveSubsystem)
    }
    var AutoBalance = AutoBalance()
    override fun execute() {
        swerveSubsystem.drive(AutoBalance.autoBalanceRoutine(), 0.0, 0.0, true, false)
    }
}