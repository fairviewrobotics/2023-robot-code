package frc.robot.subsystems
//
//import edu.wpi.first.wpilibj.AddressableLED
//import edu.wpi.first.wpilibj.AddressableLEDBuffer
//import edu.wpi.first.wpilibj.DriverStation
//import edu.wpi.first.wpilibj2.command.SubsystemBase
//import frc.robot.constants.CommandValues
//
//class LEDSubsystemRBG(val swerveDrive: SwerveSubsystem) : SubsystemBase() {
//    private var ds = DriverStation.isTeleop()
//    var led = AddressableLED(5)
//    var ledBuffer = AddressableLEDBuffer(17)
//    var autobalancing : Boolean = false
//    var autobalanced : Boolean = false
//    var gyro = swerveDrive.gyro
//    var pitch = gyro.pitch.toDouble()
//    //TODO: Remember, the lights are RBG not RGB so G and B must be switched to get the correct color
//    init {
//        led.setLength(ledBuffer.length)
//        led.setData(ledBuffer)
//        led.start()
//    }
//    var time = 0
//    override fun periodic() {
//        time += 1
//        println(time)
//        if (time >= 68) {
//            time = 0
//        }
//        if (DriverStation.isEnabled()) {
//            if (DriverStation.isAutonomous()) {
//                if (autobalancing == true) {
//                    balanceColor()
//                } else if (autobalanced == true) {
//                    balanceColor()
//                } else {
//                    autoColor()
//                }
//            } else if (DriverStation.isTeleop()) {
//                if (pitch > 2 || pitch < 2) { //pitch is set to 2 because there is a tolerance of 2.5 degrees on charge station
//                    balanceColor()
//                } else {
//                    teleop()
//                }
//            }
//        } else if (DriverStation.isDisabled()) {
//            flashOfRedColor()
//        } else {
//            offLeds()
//        }
//    }
//
//    fun offLeds() {
//        for (i in 0..ledBuffer.length-1) {
//            ledBuffer.setRGB(i, 0, 0, 0)
//        }
//        led.setData(ledBuffer)
//    }
//    fun autoColor() {
//        if (DriverStation.isAutonomous()) {
//            for (i in 0..ledBuffer.length - 1) {
//                ledBuffer.setRGB(i, 0, 255, 0)
//            }
//            led.setData(ledBuffer)
//        }
//    }
//    fun balanceColor() {
//        if (time >= 1) {
//            ledBuffer.setRGB(time / 2, 255, 255, 255)
//            led.setData(ledBuffer)
//            for (i in 0..ledBuffer.length - 1) {
//                ledBuffer.setRGB(i, 0, 0, 0)
//            }
//            led.setData(ledBuffer)
//        }
//    }
//    fun teleop() {
//            if (CommandValues.cone == true) {
//                for (i in 0..ledBuffer.length - 1) {
//                    ledBuffer.setRGB(i, 255, 0, 200)
//
//                }
//                led.setData(ledBuffer)
//            } else if (CommandValues.cube == true) {
//                for (i in 0..ledBuffer.length - 1) {
//                    ledBuffer.setRGB(i, 95, 150, 0)
//                }
//                led.setData(ledBuffer)
//            } else {
//                for (i in 0..ledBuffer.length - 1) {
//                    ledBuffer.setRGB(i, 0, 0, 0)
//                }
//                led.setData(ledBuffer)
//            }
//    }
//    fun flashOfRedColor() {
//        if (DriverStation.isTeleop()) {
//            if (time >= 1) {
//                ledBuffer.setRGB(time / 4, 255, 0, 0)
//                led.setData(ledBuffer)
//                for (i in 0..ledBuffer.length - 1) {
//                    ledBuffer.setRGB(i, 0, 0, 255)
//                }
//                led.setData(ledBuffer)
////                for (i in 0..ledBuffer.length - 1) {
////                    ledBuffer.setRGB(i, 0, 0, 0)
////                }
////                led.setData(ledBuffer)
//            }
//        }
//    }
//}