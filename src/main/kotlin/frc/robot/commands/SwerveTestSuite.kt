package frc.robot.commands

import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.DrivetrainConstants
import frc.robot.Main
import frc.robot.subsystems.SparkMaxSubsystem
import frc.robot.controllers.SwerveModuleControlller
import frc.robot.subsystems.SwerveSubsystem
import kotlin.math.*

class MotorTest(val subsystem: SparkMaxSubsystem) : CommandBase() {
    init {
        addRequirements(subsystem)
    }
    override fun execute() {
        subsystem.controller.setVoltage(4.0)
    }
}

class EncoderReadout(name: String, val subsystem: SparkMaxSubsystem, absolute: Boolean, val offset: Double) : CommandBase(), Sendable {
    val velocityTopic = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("${name}_Velocity").getEntry(0.0)
    val positionTopic = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("${name}_Position").getEntry(0.0)
    val rawPositionTopic = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("${name}_PositionRaw").getEntry(0.0)
    val absoluteEncoder = subsystem.controller.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val relativeEncoder = if (!absolute) {
        subsystem.controller.encoder
    } else {
        null
    }

    init {
        addRequirements(subsystem)

        if (absoluteEncoder != null) {
            absoluteEncoder.positionConversionFactor = 2 * Math.PI;
            absoluteEncoder.velocityConversionFactor = (2 * Math.PI) / 60.0;
        }

        if (relativeEncoder != null) {
            relativeEncoder.positionConversionFactor = 1.0
            relativeEncoder.velocityConversionFactor = 1.0
        }
    }

    fun getPosition(): Double {
        if (absoluteEncoder != null) {
            return if (absoluteEncoder.position < offset) {
                (2* Math.PI) + (absoluteEncoder.position-offset);
            } else {
                absoluteEncoder.position - offset;
            }
        }

        if (relativeEncoder != null) {
            return relativeEncoder.position
        }

        return 0.0
    }
    /*fun getPosition(): Double {
        if (absoluteEncoder != null) {
            absoluteEncoder.position
        }
    }*/


    fun getVelocity(): Double {
        if (absoluteEncoder != null) {
            return absoluteEncoder.velocity
        }

        if (relativeEncoder != null) {
            return relativeEncoder.velocity
        }

        return 0.0
    }

    override fun execute() {
        positionTopic.set(getPosition())
        velocityTopic.set(getVelocity())
        rawPositionTopic.set(absoluteEncoder.position)
    }
}


class OpenLoopTest(val controller: XboxController, subsystem: SparkMaxSubsystem) : CommandBase() {
    val encoder = subsystem.controller.encoder
    val motor = subsystem.controller

    val velEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("OpenLoopTest_Velocity").getEntry(0.0)
    val setpointEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("OpenLoopTest_Setpoint").getEntry(0.0)

    init {
        addRequirements(subsystem)

        encoder.velocityConversionFactor = 1.0
        encoder.positionConversionFactor = 1.0
    }

    override fun execute() {
        if (controller.aButton) {
            println("set")
            motor.setVoltage(12.0)
            setpointEntry.set(1.0)
        } else {
            motor.setVoltage(0.0)
            setpointEntry.set(0.0)
        }

        velEntry.set(encoder.velocity)
    }

    override fun end(interrupted: Boolean) {
        motor.set(0.0)
    }
}

class ClosedLoopTest(val controller: XboxController, val subsystem: SparkMaxSubsystem) : CommandBase() {
    val encoder = subsystem.controller.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val motor = subsystem.controller
    val setpoint = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("ClosedLoopTest_Setpoint").getEntry(0.0) // TODO: figure out absolute encoder readouts and set value(s) accordingly
    val pv = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("ClosedLoopTest_PV").getEntry(0.0)
    val pidP = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("ClosedLoopTest_Pid_P").getEntry(0.0)
    val pidI = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("ClosedLoopTest_Pid_I").getEntry(0.0)
    val pidD = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("ClosedLoopTest_Pid_D").getEntry(0.0)

    val pid = PIDController(pidP.get(), pidI.get(), pidD.get())

    init {
        addRequirements(subsystem)

        // todo: Convert these and shit
        pid.enableContinuousInput(0.0, DrivetrainConstants.turningEncoderPositionPIDMaxInput)

        // todo: convert these into what we actually find.
        encoder.velocityConversionFactor = 1.0
        encoder.positionConversionFactor = 1.0

    }

    override fun execute() {
        pid.p = pidP.get()
        pid.i = pidI.get()
        pid.d = pidD.get()

        if (controller.aButton) {
            motor.set(pid.calculate(encoder.position, setpoint.get()))
        } else {
            motor.set(0.0)
        }

        pv.set(encoder.position)

    }
}

class SingleModuleTest(val module: SwerveModuleControlller, val controller: XboxController) : CommandBase() {
    val angleEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("SingleModuleTest_Angle").getEntry(0.0)
    val speedEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("SingleModuleTest_Speed").getEntry(0.0)
    override fun execute() {
        val x = controller.leftX
        val y = controller.leftY
        val speed = controller.leftTriggerAxis * DrivetrainConstants.maxSpeedMetersPerSecond
        val dist = sqrt(x.pow(2) + y.pow(2))
        var angle: Double = 0.0
        if (dist > 0.2) {
            angle = atan(controller.leftY / controller.leftX)
            if (x > 0 && y > 0) {
            } else if (x < 0 && y > 0) {
                angle = Math.PI - abs(angle)
            } else if (x < 0 && y < 0) {
                angle = Math.PI + abs(angle)
            } else {
                angle = (2 * Math.PI) - abs(angle)
            }
        }

        module.setDesiredState(SwerveModuleState(speed, Rotation2d.fromRadians(angle)))
        angleEntry.set(angle)
        speedEntry.set(speed)
    }
}

class PointInDirection(val swerveSubsystem: SwerveSubsystem,
                       val controller: XboxController) : CommandBase() {
//    val angleEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("SingleModuleTest_Angle").getEntry(0.0)
//    val speedEntry = NetworkTableInstance.getDefault().getTable("SwerveTestSuite").getDoubleTopic("SingleModuleTest_Speed").getEntry(0.0)

    init {
        addRequirements(swerveSubsystem)
    }

    fun motorAngle(): Double {
        var x = controller.leftX
        var y = controller.leftY
        var angle: Double = 0.0
        var dist = sqrt(x.pow(2) + y.pow(2))
        if (dist > 0.2) {
            angle = atan2(controller.leftY, controller.leftX) - Math.PI/2
            if (angle < 0){
                angle = (2 * Math.PI) - angle
            }
        }
        return angle
    }

    override fun execute() {

        var speed = controller.leftTriggerAxis * DrivetrainConstants.maxSpeedMetersPerSecond

        var angle: Double = motorAngle()

        var moduleState: SwerveModuleState = SwerveModuleState(speed, Rotation2d.fromRadians(angle))

        swerveSubsystem.directionDrive(moduleState)

//        angleEntry.set(angle)
//        speedEntry.set(speed)
    }
}