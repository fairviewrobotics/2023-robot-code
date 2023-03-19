package frc.robot.commands
import edu.wpi.first.wpilibj.*
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase

class Balancer(val swerveDrive: SwerveSubsystem): CommandBase() {
    var gyro = swerveDrive.gyro
    var pitch = gyro.pitch.toDouble()
    val toleranceDegrees = 7.0//on the field, it is 2.5, but we dont want it getting too close
    val targetAngle = 0.0
    //This balance speed means that at 45 degrees off, the swerve will use 3.0 m/s to fix the error.
    val balanceSpeed = 0.15 //TODO: TUNE

    var coerceIn = 1.0
    var cutCoerce = 1

    init {
        addRequirements(swerveDrive)
    }

    override fun execute() {
        pitch = gyro.pitch.toDouble()
        if (!isBalanced()) {
            cutCoerce = 1
            var gyroAngle = pitch
            var error = (targetAngle-gyroAngle).coerceIn(-coerceIn,coerceIn)
            var correction = error * balanceSpeed
            swerveDrive.drive(-correction, 0.0, 0.0, false, true)
        } else {
            swerveDrive.setX()
            if (cutCoerce == 1){
                coerceIn = coerceIn/1.5
                cutCoerce = 0
            }

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