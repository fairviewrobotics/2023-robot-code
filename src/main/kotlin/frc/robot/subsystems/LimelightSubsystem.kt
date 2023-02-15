package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.NetworkTableUtils

class LimelightSubsystem : SubsystemBase() {

    private val limelightNetworkTable = NetworkTableUtils("limelight")

    fun aprilTagGuard() {
        if (pipeline != Pipelines.APRILTAG) {
            pipeline = Pipelines.APRILTAG
        }
    }

    fun getPose() : DoubleArray {
        aprilTagGuard()
        return limelightNetworkTable.getDoubleArray("botpose", DoubleArray(0))
    }

    fun getTargetPose(): DoubleArray {
        aprilTagGuard()
        return limelightNetworkTable.getDoubleArray("targetpose_robotspace", DoubleArray(0))
    }

    fun getAprilTagOffset() : Double {
        aprilTagGuard()
        return limelightNetworkTable.getEntry("tx", 0.0) as Double
    }

    var pipeline
        get() = limelightNetworkTable.getEntry("pipeline", 0) as Pipelines
        set(x: Pipelines) = limelightNetworkTable.setEntry("pipeline", x as Int)

    fun canSeeTarget(): Boolean {
        return limelightNetworkTable.getEntry("tv", 0) == 1
    }

    enum class Pipelines(pipeline: Int) {
        APRILTAG(0),
        CARGO(1)
    }

}