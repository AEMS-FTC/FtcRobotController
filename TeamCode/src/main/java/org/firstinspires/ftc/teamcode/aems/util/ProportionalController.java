package aems;

public class ProportionalController {
    private double proportional = 0; //The proportional modifier value

    public ProportionalController(double pro) {
        this.proportional = pro; //assigns the proportional modifier value to a givin pro value
    }

    public double calculate(double target, double current) {
        return (target - current) * proportional; //Returns the output of the proportional calculation
    }
}