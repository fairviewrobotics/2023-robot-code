package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.NetworkTableUtils

class LimelightSubsystem : SubsystemBase() {

    private val limelightNetworkTable = NetworkTableUtils("limelight")
    fun getPose() : DoubleArray {
        return limelightNetworkTable.getDoubleArray("botpose", DoubleArray(0))
    }
    fun getTargetPose(): DoubleArray {
        return limelightNetworkTable.getDoubleArray("targetpose_robotspace", DoubleArray(0))
    }

    fun getAprilTagOffseta() : Double {
        setPipeline(Pipelines.APRILTAG)
        return limelightNetworkTable.getEntry("tx", 0.0) as Double
    }

    fun setPipeline(pipeline: Pipelines) {
        limelightNetworkTable.setEntry("pipeline", pipeline)
    }

    fun getPipeline(): Int {
        return limelightNetworkTable.getEntry("pipeline", 0) as Int
    }

    fun canSeeTarget(): Boolean {
        return limelightNetworkTable.getEntry("tv", 0) == 1
    }

    enum class Pipelines(pipeline: Int) {
        APRILTAG(0),
        CARGO(1)
    }

}