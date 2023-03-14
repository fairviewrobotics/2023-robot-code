package frc.robot

import edu.wpi.first.wpilibj.BuiltInAccelerometer

class AutoBalance() {
    private val mRioAccel: BuiltInAccelerometer
    private var state: Int
    private var debounceCount: Int
    private val robotSpeedSlow: Double
    private val robotSpeedFast: Double
    private val onChargeStationDegree: Double
    private val levelDegree: Double
    private val debounceTime: Double
    private val singleTapTime: Double
    private val scoringBackUpTime: Double
    private val doubleTapTime: Double

    init {
        mRioAccel = BuiltInAccelerometer()
        state = 0
        debounceCount = 0
        /**********
         * CONFIG *
         */
        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = 0.4

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = 0.2

        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = 13.0

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = 6.0

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 0.2

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3
    }

    val pitch: Double
        get() = Math.atan2(
            (-mRioAccel.x),
            Math.sqrt(mRioAccel.y * mRioAccel.y + mRioAccel.z * mRioAccel.z)
        ) * 57.3
    val roll: Double
        get() {
            return Math.atan2(mRioAccel.y, mRioAccel.z) * 57.3
        }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    val tilt: Double
        get() {
            val pitch: Double = pitch
            val roll: Double = roll
            if ((pitch + roll) >= 0) {
                return Math.sqrt(pitch * pitch + roll * roll)
            } else {
                return -Math.sqrt(pitch * pitch + roll * roll)
            }
        }

    fun secondsToTicks(time: Double): Int {
        return (time * 50).toInt()
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    fun autoBalanceRoutine(): Double {
        when (state) {
            0 -> {
                if (tilt > onChargeStationDegree) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1
                    debounceCount = 0
                    return robotSpeedSlow
                }
                return robotSpeedFast
            }

            1 -> {
                if (tilt < levelDegree) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2
                    debounceCount = 0
                    return 0.0
                }
                return robotSpeedSlow
            }

            2 -> {
                if (Math.abs(tilt) <= levelDegree / 2) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4
                    debounceCount = 0
                    return 0.0
                }
                if (tilt >= levelDegree) {
                    return 0.1
                } else if (tilt <= -levelDegree) {
                    return -0.1
                }
                return 0.0
            }

            3 -> return 0.0
        }
        return 0.0
    }

    // Same as auto balance above, but starts auto period by scoring
    // a game piece on the back bumper of the robot
    fun scoreAndBalance(): Double {
        when (state) {
            0 -> {
                debounceCount++
                if (debounceCount < secondsToTicks(singleTapTime)) {
                    return -robotSpeedFast
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime)) {
                    return robotSpeedFast
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime + doubleTapTime)) {
                    return -robotSpeedFast
                } else {
                    debounceCount = 0
                    state = 1
                    return 0.0
                }
            }

            1 -> {
                if (tilt > onChargeStationDegree) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2
                    debounceCount = 0
                    return robotSpeedSlow
                }
                return robotSpeedFast
            }

            2 -> {
                if (tilt < levelDegree) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3
                    debounceCount = 0
                    return 0.0
                }
                return robotSpeedSlow
            }

            3 -> {
                if (Math.abs(tilt) <= levelDegree / 2) {
                    debounceCount++
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4
                    debounceCount = 0
                    return 0.0
                }
                if (tilt >= levelDegree) {
                    return robotSpeedSlow / 2
                } else if (tilt <= -levelDegree) {
                    return -robotSpeedSlow / 2
                }
                return 0.0
            }

            4 -> return 0.0
        }
        return 0.0
    }
}