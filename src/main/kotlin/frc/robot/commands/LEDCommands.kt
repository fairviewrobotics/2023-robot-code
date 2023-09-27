package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.CommandValues
import frc.robot.subsystems.BlinkinLEDSubsystem

class SetLEDValueConeCube(val subsystem: BlinkinLEDSubsystem): CommandBase() {

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        if (CommandValues.cube) {
            subsystem.setLED(0.91)
        } else {
            subsystem.setLED(0.69)
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
