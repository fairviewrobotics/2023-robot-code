package frc.robot.constants

import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.math.trajectory.TrapezoidProfile

object VisionConstants {
    enum class Pipelines(x: Int) {
        APRILTAG(0),
        CONE(1),
        RETROREFLECTIVE(2),
        CUBE(3)
    }

    /** This value is to compensate when the camera is not on the center axis of the robot. To set, open up the
     * Limelight development page and center the robot with the target (AprilTag or peg), and note down the tx
     * offset it displays.
     *
     * If this value is 0, that means the camera is perfectly centered on the robot.
     */

    const val lineupZP = 0.4
    const val lineupZI = 0.0
    const val lineupZD = 0.0

    val lineupZConstraints = TrapezoidProfile.Constraints(0.75, 0.5)

    const val lineupRotP = 0.5
    const val lineupRotI = 0.0
    const val lineupRotD = 0.0

    val lineupRotConstraints = TrapezoidProfile.Constraints(0.75, 0.2)


    const val retroreflectiveP = 0.03
    const val retroreflectiveI = 0.0
    const val retroreflectiveD = 0.0

    const val ttaP = 5.0
    const val ttaI = 0.0
    const val ttaD = 0.0
    val ttaConstraints = TrapezoidProfile.Constraints(3.14159, 3.14159)


    val mapper = ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
}