package frc.robot.commands

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController

class Balance(val swerveSubsystem: SwerveSubsystem, val gyro: AHRS, val balancePID: PIDController) : CommandBase() {
  init {
    addRequirements(swerveSubsystem)
    balancePID: PIDController = PIDController(1, 0, 0)
  }

  override fun execute(){
    var pitch = gyro.getPitch() as Double;
    swerveSubsystem.drive(-balancePID.calculate(pitch, 0.0), 0.0, 0.0, false, true)

  }
}