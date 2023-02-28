package frc.robot.commands

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil

class Balance(val swerveSubsystem: SwerveSubsystem, val gyro: AHRS) : CommandBase() {
  init {
    addRequirements(swerveSubsystem)
  }

  override fun execute(){
    var pitch = gyro.getPitch() as Double;
    pitch = MathUtil.applyDeadband(pitch, 1.0)
    if(pitch < 0.0){
      swerveSubsystem.drive(-0.1, 0.0, 0.0, false, true)
    } else if (pitch > 0.0){
      swerveSubsystem.drive(0.1, 0.0, 0.0, false, true)
    }

  }
}