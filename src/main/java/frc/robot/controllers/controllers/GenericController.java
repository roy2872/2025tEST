package frc.robot.controllers.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controllers.interfaces.DriverInterface;

public class GenericController implements DriverInterface {
    private GenericHID controller;
    
    public GenericController(int id) {
        controller = new GenericHID(id);

    }

    @Override
    public DoubleSupplier getLeftX() {
        
        return () -> controller.getRawAxis(0);
    }

    @Override
    public DoubleSupplier getLeftY() {
        
        return () -> controller.getRawAxis(1);
    }

    @Override
    public DoubleSupplier getRightX() {
        
        return () -> controller.getRawAxis(2);
    }
}
