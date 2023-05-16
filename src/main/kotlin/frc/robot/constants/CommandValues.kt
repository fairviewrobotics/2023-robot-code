package frc.robot.constants

object CommandValues {
    //Swerve
    var fieldOriented: Boolean = true

    //Starting Config: Cube, with Middle Place, and Ground Pickup
    var cube: Boolean = true // Both
    var middlePlace: Boolean = true // Place
    var floor: Boolean = false // Place
    var chute: Boolean = false // Pickup
    var pickup: Boolean = false

    //Vision
    var vision = false

    // THESE ARE ONLY FOR DRIVERS(Network Tables), NOT USED IN CODE
    var ground: Boolean = true // Pickup
    var cone: Boolean = false // Both
    var highPlace: Boolean = false // Place


}
