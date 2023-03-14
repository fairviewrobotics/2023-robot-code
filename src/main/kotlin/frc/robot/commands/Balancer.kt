package frc.robot.commands
import edu.wpi.first.wpilibj.*
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase

class Balancer(val swerveDrive: SwerveSubsystem): CommandBase() {
    var gyro = swerveDrive.gyro
    var pitch = gyro.pitch.toDouble()
    val toleranceDegrees = 2.0//on the field, it is 2.5, but we dont want it getting too close
    val targetAngle = 0.0
    //This balance speed means that at 45 degrees off, the swerve will use 3.0 m/s to fix the error.
    val balanceSpeed = 0.066 //TODO: TUNE

    init {
        addRequirements(swerveDrive)
    }

    override fun execute() {
        pitch = gyro.pitch.toDouble()
        if (!isBalanced()) {
            var gyroAngle = pitch
            var error = targetAngle - gyroAngle
            var correction = error * balanceSpeed
            swerveDrive.drive(correction, 0.0, 0.0, false, true)
            Timer.delay(0.01)
        } else {
            swerveDrive.setX()
        }
    }

    fun isBalanced(): Boolean {
        val gyroAngle = pitch
        return gyroAngle in (targetAngle - toleranceDegrees)..(targetAngle + toleranceDegrees)
    }

    override fun end(interrupted: Boolean) {
        swerveDrive.setX()
    }

}