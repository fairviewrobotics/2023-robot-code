package frc.robot.commands

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.Subsystem
/*
fun t(): NetworkTable {
    return NetworkTableInstance.getDefault().getTable("Arm")
}

class KCMTMasterElevatorCommand(val dummySubsystem: Subsystem, val elevatorMotorId: Int, val topBreaker: Int, val bottomBreaker: Int, val controller: XboxController): CommandBase() {

    val elevatorMotor = CANSparkMax(elevatorMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val forwardLimit = DigitalInput(topBreaker)
    val reverseLimit = DigitalInput(bottomBreaker)

    init {
        addRequirements(dummySubsystem)
        elevatorMotor.inverted = false
    }

    val topHit get() = !forwardLimit.get()
    val bottomHit get() = !reverseLimit.get()

    override fun execute() {
        var voltage = (controller.leftTriggerAxis - controller.rightTriggerAxis) * 12.0

        println(bottomHit)
        if (voltage > 0.0 && topHit) {
            voltage = 0.0
        }

        if (voltage < 0.0 && bottomHit) {
            voltage = 0.0
        }

        elevatorMotor.setVoltage(voltage)
    }

    override fun end(interrupted: Boolean) {
        elevatorMotor.setVoltage(0.0)
    }
}

class KCMTMasterArmCommand(val dummySubsystem: Subsystem, val wristMotorID : Int, val elbowMotorID: Int, val controller: XboxController) : CommandBase() {
    val wristMotor = CANSparkMax(wristMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val elbowMotor = CANSparkMax(elbowMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeMotorL = CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless)
    val intakeMotorR = CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless)

    val encoder1 = wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    val encoder2 = elbowMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

    var actual = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("Actual").publish()
    var setpointT = NetworkTableInstance.getDefault().getTable("Arm").getDoubleTopic("Setpoint").publish()
    var pidError = t().getDoubleTopic("Pid Error").publish()
    var voltageP = t().getDoubleTopic("Voltage (PID)").publish()
    var voltageFF = t().getDoubleTopic("Voltage (FF)").publish()

    val wristPosition get() = Rotation2d(encoder1.position).minus(Rotation2d(0.900)).radians
    val elbowPosition get() = Rotation2d(encoder2.position).plus(Rotation2d(wristPosition)).minus(Rotation2d(05.880)).minus(
        Rotation2d(-1.25)
    ).radians


    val pid = ProfiledPIDController(6.0, 0.0, 0.0, TrapezoidProfile.Constraints(6.0, 24.0))
    val feedforward = ArmFeedforward(0.13, 2.07, 1.00)
    var elbowSetpoint = elbowPosition
    
    var wristPid = PIDController(1.0,0.0,0.0)
    var wristSetpoint = wristPosition
    init {
        wristMotor.idleMode = CANSparkMax.IdleMode.kBrake
        addRequirements(dummySubsystem)
        encoder1.positionConversionFactor = 2.0*Math.PI
        encoder2.positionConversionFactor = 2.0*Math.PI
        encoder1.velocityConversionFactor = 2.0*Math.PI/60.0
        encoder2.velocityConversionFactor = 2.0*Math.PI/60.0
        // conversion factor
        elbowMotor.inverted = true
        intakeMotorL.inverted = false
        elbowMotor.setSmartCurrentLimit(30)
    }

    override fun execute() {
        // intake
        if (controller.leftTriggerAxis >= 0.1) {
            intakeMotorL.setVoltage(10.0)
            intakeMotorR.setVoltage(10.0)
        } else if (controller.rightTriggerAxis >= 0.1) {
            intakeMotorL.setVoltage(-10.0)
            intakeMotorR.setVoltage(-10.0)
        } else {
            intakeMotorR.setVoltage(0.0)
            intakeMotorL.setVoltage(0.0)
        }

        /// elbow
        elbowSetpoint = (elbowSetpoint + (controller.leftY)/25).coerceIn(-Math.PI / 2, Math.PI / 4)

        var outputP = pid.calculate(elbowPosition, elbowSetpoint)
        var outputFF = feedforward.calculate(elbowPosition, 0.0)
        elbowMotor.setVoltage(outputP + outputFF)
    
        wristSetpoint = (wristSetpoint + (controller.rightY)/25).coerceIn(-Math.PI/2.0 - Math.PI/8.0,Math.PI/4.0)
        val outputPW = wristPid.calculate(wristPosition, wristSetpoint + elbowPosition)
        wristMotor.setVoltage(outputPW)
    }
}


 */