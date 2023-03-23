package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.DoubleEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import frc.robot.VisionUtils
import frc.robot.constants.VisionConstants
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.*

fun NT(x: String): DoubleEntry {
    return NetworkTableInstance.getDefault().getTable("Vision").getDoubleTopic(x).getEntry(0.0)
}

fun SetPipeline(pipeline: VisionConstants.Pipelines): Command {
    return InstantCommand({
        VisionUtils.setPipelineIndex("", pipeline.ordinal)
    })
}

class RumbleCheck(val controller: XboxController, val check: () -> Boolean) : CommandBase() {
    override fun execute() {
        controller.setRumble(RumbleType.kBothRumble, (if (check()) 1.0 else 0.0));
    }

    override fun end(int2errupted: Boolean) {
        controller.setRumble(RumbleType.kBothRumble, 0.0)
    }

    override fun isFinished(): Boolean {
        return !check()
    }
}

class ZAlign(val driveSubsystem: SwerveSubsystem, val targetZ: Double) : CommandBase() {
    val lineupZPID = ProfiledPIDController(
        VisionConstants.lineupZP,
        VisionConstants.lineupZI,
        VisionConstants.lineupZD,
        VisionConstants.lineupZConstraints
    )
    val zFilter = LinearFilter.singlePoleIIR(0.1, 0.02)
    var mostRecentZ = 0.0

    val ZSetpoint = NT("ZSetpoint")
    val ZMeasurement = NT("ZMeasurement")
    val ZOutput = NT("ZOutput")

    val CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false)

    init {
        addRequirements(driveSubsystem)
        calculateTargets()
    }

    // moving this into init may help
    override fun initialize() {
        CommandRunning.set(true)
        lineupZPID.setTolerance(0.1, 0.1)
        lineupZPID.reset(mostRecentZ)
        lineupZPID.setGoal(targetZ)
    }

    fun calculateTargets() {
        val targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Fiducials
        val latestTarget = if (targets.isNotEmpty()) {
            targets[0]
        } else {
            null
        }

        mostRecentZ = zFilter.calculate(
            if (latestTarget == null) {
                mostRecentZ
            } else {
                latestTarget.robotPose_TargetSpace[2]
            }
        )
    }

    override fun execute() {
        calculateTargets()

        // TODO: maybe invert zoutput?
        val zOutput = lineupZPID.calculate(mostRecentZ)
        driveSubsystem.drive(-zOutput, 0.0, 0.0, true, false)


        ZMeasurement.set(mostRecentZ)
        ZOutput.set(-zOutput)
        ZSetpoint.set(lineupZPID.goal.position)
    }

    override fun isFinished(): Boolean {
        return lineupZPID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        CommandRunning.set(false)
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}

class RetroreflectiveAlign(val driveSubsystem: SwerveSubsystem) : CommandBase() {
    val lineupXPID = PIDController(
        VisionConstants.retroreflectiveP,
        VisionConstants.retroreflectiveI,
        VisionConstants.retroreflectiveD
    )

    val xFilter = LinearFilter.singlePoleIIR(0.1, 0.02)

    var mostRecentX = 0.0

    val XSetpoint = NT("XSetpoint")
    val XMeasurement = NT("XMeasurement")
    val XOutput = NT("XOutput")

    val CommandRunning = NetworkTableInstance.getDefault().getTable("Vision").getBooleanTopic("Running").getEntry(false)

    init {
        addRequirements(driveSubsystem)
        calculateTargets()
    }

    // moving this into init may help
    override fun initialize() {
        CommandRunning.set(true)
        lineupXPID.setTolerance(0.1, 0.1)
    }

    fun calculateTargets() {
        val targets = VisionUtils.getLatestResults(VisionConstants.mapper, "").targetingResults.targets_Retro
        val latestTarget = if (targets.isNotEmpty()) {
            targets[0]
        } else {
            null
        }

        mostRecentX = xFilter.calculate(
            if (latestTarget == null) {
                mostRecentX
            } else {
                latestTarget.tx
            }
        )
        println("Ending calculate targets...")
    }

    override fun execute() {
        calculateTargets()

        val xOutput = -lineupXPID.calculate(mostRecentX, 0.1)
        driveSubsystem.drive(0.0, xOutput, 0.0, true, false)


        XMeasurement.set(mostRecentX)
        XOutput.set(xOutput)

    }

    override fun isFinished(): Boolean {
        return lineupXPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        CommandRunning.set(false)
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}

// CCW is positive
class TurnToAngle(val driveSubsystem: SwerveSubsystem, val targetAngleRadians: Double) : CommandBase() {
    val ttaPID = PIDController(VisionConstants.ttaP, VisionConstants.ttaI, VisionConstants.ttaD)

    val setpoint = NT("TTASetpoint")
    val measurement = NT("TTAMeasurement")
    val output = NT("TTAOutput")

    val heading get() = Rotation2d.fromRadians(driveSubsystem.heading)

    init {
        addRequirements(driveSubsystem)
    }

    override fun initialize() {
        ttaPID.setTolerance(0.1, 0.1)
        ttaPID.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun execute() {
        val out = ttaPID.calculate(heading.radians, targetAngleRadians)
        driveSubsystem.drive(0.0, 0.0, out, true, false)

        setpoint.set(targetAngleRadians)
        measurement.set(heading.radians)
        output.set(out)
    }

    override fun isFinished(): Boolean {
        return ttaPID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false)
    }
}

fun ChuteVision(driveSubsystem: SwerveSubsystem, controller: XboxController): Command {
    return SequentialCommandGroup(
        SetPipeline(VisionConstants.Pipelines.APRILTAG),
        RumbleCheck(controller) { !VisionUtils.getTV("") },
        ZAlign(driveSubsystem, -3.1),
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
            TurnToAngle(driveSubsystem, Math.PI / 2.0)
        else
            TurnToAngle(driveSubsystem, Math.PI / 2.0)
    )
}

fun RetroreflectiveVision(driveSubsystem: SwerveSubsystem, controller: XboxController): SequentialCommandGroup {
    return (SequentialCommandGroup(
        SetPipeline(VisionConstants.Pipelines.RETROREFLECTIVE),
        RumbleCheck(controller) { !VisionUtils.getTV("") },
        TurnToAngle(driveSubsystem, 0.0),
        RetroreflectiveAlign(driveSubsystem)
    ))
}