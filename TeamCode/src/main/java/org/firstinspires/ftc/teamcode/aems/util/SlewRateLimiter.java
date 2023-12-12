package aems;

public class SlewRateLimiter {
    private double slewRate = 0; //The maximum amount a value can change per second

    public SlewRateLimiter(double rate) {
        this.slewRate = rate; //assigns the maximum slew rate to a givin rate value
    }

    public double calculate(double input, double prev) {
        if (slewRate < Math.abs(input - prev)) {
            if (input < prev) {
                return prev - slewRate;
            } else if (input > prev) {
                return prev + slewRate;
            }
        }
        return input;
    }
}
