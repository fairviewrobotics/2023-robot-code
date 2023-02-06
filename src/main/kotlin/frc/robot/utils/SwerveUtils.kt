package frc.robot.utils

object SwerveUtils {
    /**
     * Steps a value towards a target with a specified step size.
     * @param current The current or starting value.  Can be positive or negative.
     * @param target The target value the algorithm will step towards.  Can be positive or negative.
     * @param stepsize The maximum step size that can be taken.
     * @return The new value for `current` after performing the specified step towards the specified target.
     */
    fun StepTowards(current: Double, target: Double, stepsize: Double): Double {
        return if (Math.abs(current - target) <= stepsize) {
            target
        } else if (target < current) {
            current - stepsize
        } else {
            current + stepsize
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for `current` after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    fun StepTowardsCircular(current: Double, target: Double, stepsize: Double): Double {
        var current = current
        var target = target
        current = WrapAngle(current)
        target = WrapAngle(target)
        val stepDirection = Math.signum(target - current)
        val difference = Math.abs(current - target)
        return if (difference <= stepsize) {
            target
        } else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (current + 2 * Math.PI - target < stepsize || target + 2 * Math.PI - current < stepsize) {
                target
            } else {
                WrapAngle(current - stepDirection * stepsize) //this will handle wrapping gracefully
            }
        } else {
            current + stepDirection * stepsize
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angleA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    fun AngleDifference(angleA: Double, angleB: Double): Double {
        val difference = Math.abs(angleA - angleB)
        return if (difference > Math.PI) 2 * Math.PI - difference else difference
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    fun WrapAngle(angle: Double): Double {
        val twoPi = 2 * Math.PI
        return if (angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            0.0
        } else if (angle > twoPi) {
            angle - twoPi * Math.floor(angle / twoPi)
        } else if (angle < 0.0) {
            angle + twoPi * (Math.floor(-angle / twoPi) + 1)
        } else {
            angle
        }
    }
}