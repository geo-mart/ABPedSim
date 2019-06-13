package de.fhg.ivi.crowdsimulation.simulation.numericintegration;

import java.util.List;

import de.fhg.ivi.crowdsimulation.simulation.forcemodel.ForceModel;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;

/**
 * Algorithms of numeric integrations are necessary to dissolve Ordinary Differential Equations like
 * the Social Force Model (SFM) respectively {@link ForceModel}. It exists many algorithms, which
 * can dissolve the SFM, e.g. different Euler algorithms, Verlet algorithm, many Runge-Kutta
 * algorithms and many more.
 * <p>
 * This class is an abstract class and the super class of this numericintegration package. For that
 * one can use three different classes or algorithms, which are capable to dissolve the
 * {@link ForceModel}. These classes are {@link SemiImplicitEulerIntegrator},
 * {@link SimpleEulerIntegrator} and {@link RungeKuttaIntegrator}.
 *
 * @author hahmann/meinert
 */
public abstract class NumericIntegrator
{

    /**
     * Invokes one of the three numeric integration methods, which means
     * {@link SemiImplicitEulerIntegrator}, {@link SimpleEulerIntegrator} or
     * {@link RungeKuttaIntegrator}, to calculate the velocity and position of all
     * {@link Pedestrian}s.
     *
     * @param currentTimeMillis the current unix time stamp in simulated time, given in milliseconds
     * @param simulationInterval the time between this method invocation and the last one
     * @param pedestrian the one {@link Pedestrian}, whose movement is calculated
     * @param pedestrians represents a list of all {@link Pedestrian}s
     * @param boundaries represents a list of all {@link Boundary}s
     * @param forceModel an {@link Object} of {@link ForceModel}
     */
    public abstract void move(long currentTimeMillis, double simulationInterval,
        Pedestrian pedestrian, List<Pedestrian> pedestrians, List<Boundary> boundaries,
        ForceModel forceModel);
}
