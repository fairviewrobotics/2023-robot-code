package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.subsystems.LEDSubsystem
import frc.robot.subsystems.LEDSubsystemState

fun SetLEDs(leds: LEDSubsystem, state: LEDSubsystemState) : Command {
    return RunCommand({
        leds.setLEDState(state)
    }, leds)
}