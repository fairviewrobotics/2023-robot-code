package frc.robot.commands
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.subsystems.PickAndPlaceSubsystem

fun ControllerCommands(pickup: Boolean, cube: Boolean, middlePlace: Boolean, floor: Boolean, chute: Boolean, pnp: PickAndPlaceSubsystem): Command {
    var command: Command
    if (!pickup && floor){
        command = FloorPlace(pnp)
    } else if (pickup && chute) {
        command = ChutePick(pnp)
    } else if (!pickup && cube && middlePlace) {
        command = MidPlaceCube(pnp)
    } else if (!pickup && !cube && middlePlace) {
        command =  MidPlaceCone(pnp)
    } else if (!pickup && cube && !middlePlace) {
        command =  HighPlaceCube(pnp)
    } else if (!pickup && !cube && !middlePlace) {
        command =  HighPlaceCone(pnp)
    } else if (pickup && cube) {
        command =  LowPickCube(pnp)
    } else if (pickup && cube) {
        command =  LowPickCone(pnp)
    } else {
        command =  Base(pnp)
    }
    return command

}