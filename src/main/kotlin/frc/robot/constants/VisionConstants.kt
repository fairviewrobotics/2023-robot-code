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
    const val alignmentTxOffset = 0

    const val targetZ = 4.0
    const val lineupXP = 0.05
    const val lineupXI = 0.0
    const val lineupXD = 0.0
    const val lineupZP = 0.04
    const val lineupZI = 0.0
    const val lineupZD = 0.0
    const val lineupRotP = 0.1
    const val lineupRotI = 0.0
    const val lineupRotD = 0.0
    val lineupXConstraints = TrapezoidProfile.Constraints(1.0, 10.0)
    val lineupZConstraints = TrapezoidProfile.Constraints(5.0, 10.0)
    val lineupRotConstraints = TrapezoidProfile.Constraints(5.0, 10.0)
}