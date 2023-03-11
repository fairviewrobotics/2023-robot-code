package frc.robot.commands

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController

class Balance(val swerveSubsystem: SwerveSubsystem) : CommandBase() {
  val balancePID = PIDController(1.0, 0.0, 0.0)
  init {
    addRequirements(swerveSubsystem)
  }

  override fun execute(){
    var pitch = swerveSubsystem.gyro.pitch.toDouble()
    swerveSubsystem.drive(-balancePID.calculate(pitch, 0.0), 0.0, 0.0, true, true)
  }

  override fun end(interrupted: Boolean) {
    swerveSubsystem.drive(0.0,0.0,0.0,true,true)
    swerveSubsystem.setX()
  }
}