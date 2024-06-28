package frc.robot.commands.Interpolation;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.Pair;

public class NonLinearSolver {
    public static double[] shoot(double x, double y, double vx, double vy) {
            //position 
            // double x = -2;
            // double y = 2;
            // robot velocity
            // double vx = 0;
            // double vy = 10;
            // note velocity
            double vn = 36;
            double g = 9.8;
            double hs = 2.045;
            double hr = 0.3;
            double l = .74;   
            LeastSquaresProblem problem = new LeastSquaresBuilder().
                    start(new double[]{0, Math.PI/4}). // initial guess
                            model(jacobian(x,y,vx,vy,vn,g,hs,hr,l)). // derivative of the function
                            target(new double[]{0, 0}). // target values
                            lazyEvaluation(false).
                    maxEvaluations(1000).
                    maxIterations(1000).
                    build();        
            LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer();
            long start = System.currentTimeMillis();
            RealVector solution = optimizer.optimize(problem).getPoint();
            long end = System.currentTimeMillis();
            // System.out.println(end-start);
            // System.out.println("Solution: theta = " + solution.getEntry(0) + ", phi = " + solution.getEntry(1));
            return new double[]{solution.getEntry(0), solution.getEntry(1)};
    }

    private static MultivariateJacobianFunction jacobian(double x, double y, double vx, double vy, double vn, double g, double hs, double hr, double l) {
            return new MultivariateJacobianFunction() {
                public Pair<RealVector, RealMatrix> value(final RealVector point) {
                        double theta = point.getEntry(0);
                        double phi = point.getEntry(1);
        
                        double cosTheta = Math.cos(theta);
                        double sinTheta = Math.sin(theta);
                        double cosPhi = Math.cos(phi);
                        double cosPhiPrime = Math.cos(phi + 0.6457718);
                        double sinPhi = Math.sin(phi);
                        double sinPhiPrime = Math.sin(phi + 0.6457718);
        
                        double h = -hs + hr;
                        double timeRoot = Math.pow(vn * cosPhiPrime, 2) + (2 * g * (h + (l * sinPhi)));
                        double timeConstant = (vn * cosPhiPrime - Math.sqrt(timeRoot)) / g;
                        double timeConstantDPhi = (-sinPhiPrime * vn / g) - (1 / g) * Math.pow(timeRoot, -0.5) * (-vn * vn * cosPhiPrime * sinPhiPrime + g*l*cosPhi);
        
                        // Compute value
                        double[] value = new double[2];
                        value[0] = x - l*cosPhi*cosTheta + (vx+(vn*cosTheta*sinPhiPrime)) * timeConstant;
                        value[1] = y + l*cosPhi*sinTheta + (vy+(vn*sinTheta*sinPhiPrime)) * timeConstant;
        
                        double df1dt = l*cosPhi*sinTheta -vn*sinPhiPrime*sinTheta*timeConstant;
                        double df1dp = -l*sinTheta*sinPhi + (vx+(vn*cosTheta*sinPhiPrime))* timeConstantDPhi + timeConstant*vn*cosTheta*cosPhiPrime;
        
                        double df2dt = l*cosPhi*cosTheta + vn*sinPhiPrime*cosTheta*timeConstant;
                        double df2dp = -l*sinTheta*sinPhi + (vy+(vn*sinTheta*sinPhiPrime))* timeConstantDPhi + timeConstant*vn*sinTheta*cosPhiPrime;
        
                        // Compute Jacobian
                        double[][] jacobian = new double[][]{
                                {df1dt, df1dp},
                                {df2dt, df2dp}
                        };
        
                        return new Pair<>(new ArrayRealVector(value), new Array2DRowRealMatrix(jacobian));
                    }
            };
        }
}