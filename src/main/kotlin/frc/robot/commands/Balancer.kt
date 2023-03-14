package frc.robot.commands
import edu.wpi.first.wpilibj.*
import frc.robot.subsystems.SwerveSubsystem
import com.kauailabs.navx.frc.AHRS

class Balancer(
    private val gyro: AHRS,
    private val swerveDrive: SwerveSubsystem
) {
    private val toleranceDegrees = 1.0
    private val targetAngle = 0.0
    private val balanceSpeed = 0.2

    private fun isBalanced(): Boolean {
        val gyroAngle = gyro.angle
        return gyroAngle in (targetAngle - toleranceDegrees)..(targetAngle + toleranceDegrees)
    }

    fun balance() {
        while (!isBalanced()) {
            val gyroAngle = gyro.angle
            val error = targetAngle - gyroAngle
            val correction = error * balanceSpeed
            swerveDrive.drive(0.0, 0.0, correction, false, true)
            Timer.delay(0.01)
        }
        swerveDrive.setX()
    }
}