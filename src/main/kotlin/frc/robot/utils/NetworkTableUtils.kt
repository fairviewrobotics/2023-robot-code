package frc.robot.utils

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance

class NetworkTableUtils(table: String) {

    private val table: NetworkTable;


    init {
        this.table = NetworkTableInstance.getDefault().getTable(table)
    }


    fun getTable(): NetworkTable {
        return this.table
    }

    fun getDouble(key: String, defaultValue: Double) : Double {
        return this.table.getEntry(key).getDouble(defaultValue)
    }

    fun getString(key: String, defaultValue: String) : String {
        return this.table.getEntry(key).getString(defaultValue)
    }

     fun getDoubleArray(key: String, doubles: DoubleArray): DoubleArray {
        return this.table.getEntry(key).getDoubleArray(doubles)
    }

    fun setDouble(key: String, value: Double) {
        this.table.getEntry(key).setDouble(value)
    }

    fun setString(key: String, value: String) {
        this.table.getEntry(key).setString(value)
    }


    fun <T: Any> getEntry(key: String, value: T): Any {
        return if (this.table.getEntry(key).exists()) {
            when(value) {
                Double -> getDouble(key, value as Double)
                String -> getString(key, value as String)
                else -> {
                    IllegalArgumentException("Invalid value type")
                }
            }
        } else {
            IllegalArgumentException("Invalid key")
        }
    }



    fun <T: Any> setEntry(key: String, value: T) {
        if (this.table.getEntry(key).exists()) {
            when(value) {
                Double -> setDouble(key, value as Double)
                String -> setString(key, value as String)
                else -> {
                    throw IllegalArgumentException("Invalid value type")
                }
            }
        } else {
            throw IllegalArgumentException("Invalid key")
        }
    }
}