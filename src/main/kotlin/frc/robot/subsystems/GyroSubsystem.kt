package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GyroSubsystem(private var gyro: AHRS): SubsystemBase() {

    fun reset() {
        gyro.reset()
    }
    fun getHeading(): Double {
            return gyro.fusedHeading.toDouble()
    }

    fun getYaw(): Double {
        return gyro.yaw.toDouble()
    }

    fun getPitch(): Double {
        return gyro.pitch.toDouble()
    }

}