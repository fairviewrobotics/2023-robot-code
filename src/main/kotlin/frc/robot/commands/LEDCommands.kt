package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.CommandValues
import frc.robot.subsystems.BlinkinLEDSubsystem

class SetLEDValueConeCube(val subsystem: BlinkinLEDSubsystem): CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        if (CommandValues.auto) {
            if (CommandValues.balanced) {
                subsystem.setLED(0.59)
                println("Balanced")
            } else if (CommandValues.balancing) {
                subsystem.setLED(-0.31)
                println("Balancing")
            } else {
                subsystem.setLED(-0.29)
                println("Auto LEDs")
            }
        } else if (CommandValues.visionIsMovingRobot) {
            subsystem.setLED(0.73)
        } else {
            if (CommandValues.cube) {
                subsystem.setLED(0.91)
            } else if (!CommandValues.cube) {
                subsystem.setLED(0.69)
            } else {
                subsystem.setLED(0.93)
            }
        }
    }
}

class SetLEDValue(val subsystem: BlinkinLEDSubsystem, val value: Double): CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        subsystem.setLED(value)
    }
}
