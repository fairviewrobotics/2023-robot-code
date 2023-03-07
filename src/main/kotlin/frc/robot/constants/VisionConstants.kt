package frc.robot.constants

import edu.wpi.first.math.trajectory.TrapezoidProfile

object VisionConstants {
    enum class Pipelines(x: Int) {
        APRILTAG(0),
        CONE(1),
        RETROREFLECTIVE(2)
    }

    /** This value is to compensate when the camera is not on the center axis of the robot. To set, open up the
     * Limelight development page and center the robot with the target (AprilTag or peg), and note down the tx
     * offset it displays.
     *
     * If this value is 0, that means the camera is perfectly centered on the robot.
     */
    const val alignmentTxOffset = -7.36

    const val lineupP = 1.0
    const val lineupI = 0.0
    const val lineupD = 0.0
    val lineupConstraints = TrapezoidProfile.Constraints(5.0, 100.0)
}