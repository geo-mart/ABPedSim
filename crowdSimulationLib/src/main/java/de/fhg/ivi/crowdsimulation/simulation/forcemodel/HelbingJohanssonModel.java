package de.fhg.ivi.crowdsimulation.simulation.forcemodel;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;

/**
 * TODO: This implementation is incomplete at the moment. This will be fixed soon.
 * <p>
 * This HelbingJohansson model is another empirical approach of the Social Force Model, based on the
 * publications of Helbing and Molnár (1995) and Helbing et al. (2005). This model itself based on
 * the publication Johansson et al. (2007) (see the last hyperlink below).
 * <p>
 * Implements the 10 parameters of the {@link HelbingModel}: A1, B1, A2, B2 for
 * pedestrian-pedestrian interaction and A1, B1, A2, B2 for pedestrian-boundary (obstacle)
 * interaction. Values for pedestrian-pedestrian interaction as well as tau and lambda are empirical
 * values as described in Johansson et al. (2007). The remaining values are described in Helbing et
 * al. (2005).
 *
 * @see <a href=
 *      "https://pubsonline.informs.org/doi/abs/10.1287/trsc.1040.0108">https://pubsonline.informs.org/doi/abs/10.1287/trsc.1040.0108</a>
 * @see <a href=
 *      "http://www.worldscientific.com/doi/pdf/10.1142/S0219525907001355">http://www.worldscientific.com/doi/pdf/10.1142/S0219525907001355</a>
 *
 * @see <a href=
 *      "http://nbn-resolving.de/urn:nbn:de:bsz:14-qucosa-20900">http://nbn-resolving.de/urn:nbn:de:bsz:14-qucosa-20900</a>
 *
 * @author hahmann/meinert
 *
 */
public class HelbingJohanssonModel extends HelbingModel
{

    /**
     * Gets the non-empirical strength parameter A1 of the acceleration resulting from the
     * interaction of a {@link Pedestrian} with another {@link Pedestrian} given in m/s².
     * <p>
     *
     * @return the non-empirical strength parameter A1 of the acceleration resulting from the
     *         interaction of a {@link Pedestrian} with another {@link Pedestrian} given in m/s².
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterPedestrianA1()
     */
    @Override
    public float getParameterPedestrianA1()
    {
        return 0.33f;
        // return 0.04f;
    }

    /**
     * Gets the B1 Parameter of the interaction range range with another {@link Pedestrian}. Given
     * in meters. Parameterizes the private zone of a {@link Pedestrian}.
     *
     * @return the non-empirical B1 Parameter of the interaction range. Given in meters.
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterPedestrianB1()
     */
    @Override
    public float getParameterPedestrianB1()
    {
        return 0.55f;
        // return 3.22f;
    }

    /**
     * Gets the strength parameter A2 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with another {@link Pedestrian}. Given in m/s². cf. Helbing et al (2005)
     * p. 12.
     *
     * @return the non-empirical strength parameter A1 of the acceleration resulting from the
     *         interaction of a {@link Pedestrian} with another {@link Pedestrian}. Given in m/s².
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterPedestrianA2()
     */
    @Override
    public float getParameterPedestrianA2()
    {
        return 0f;
    }

    /**
     * Gets the B2 Parameter of the interaction range with another {@link Pedestrian}. Given in
     * meters. Parameterizes the private zone of a {@link Pedestrian}
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterPedestrianB2()
     */
    @Override
    public float getParameterPedestrianB2()
    {
        return 0f;
    }

    /**
     * TODO explain why set to zero or check literature
     *
     *
     * Gets the strength parameter A1 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with a {@link Boundary} given in m/s².
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with a {@link Boundary} given in m/s²
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterBoundaryA1()
     */
    @Override
    public float getParameterBoundaryA1()
    {

        return 0f;
    }

    /**
     * TODO explain why set to zero or check literature
     *
     *
     * Gets the B2 Parameter of the interaction range with a {@link Boundary}. Given in meters.
     * Parameterizes the comfort zone of a {@link Pedestrian}.
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     *
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterBoundaryB1()
     */
    @Override
    public float getParameterBoundaryB1()
    {
        return 0f;
    }

    /**
     * Gets the strength parameter A2 of the acceleration resulting from the interaction of a
     * {@link Pedestrian} with a {@link Boundary}. Given in m/s�. cf. Helbing et al (2005) p. 12.
     *
     * @return the strength parameter A1 of the acceleration resulting from the interaction of a
     *         {@link Pedestrian} with a {@link Boundary}. Given in m/s�
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterBoundaryA2()
     */
    @Override
    public float getParameterBoundaryA2()
    {
        return 5f;
    }

    /**
     * Gets the B2 Parameter of the interaction range with a {@link Boundary}. Given in meters.
     * Parameterizes the comfort zone of a {@link Pedestrian}
     *
     * @return the B2 Parameter of the interaction range. Given in meters.
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getParameterBoundaryB2()
     */
    @Override
    public float getParameterBoundaryB2()
    {
        return 0.1f;
    }

    /**
     * "Relaxation time" of a {@link Pedestrian}. Given in seconds.
     *
     * @return the "Relaxation time" of a {@link Pedestrian}. Given in seconds.
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getTau()
     */
    @Override
    public float getTau()
    {
        return 0.6f;
    }

    /**
     * Gets the parameter that takes into account the anisotropic character of pedestrian
     * interactions, as the situation in front of a pedestrian has a larger impact on his or her
     * behavior than things happening behind. Cf.
     *
     * @return the parameter that takes into account the anisotropic character of pedestrian
     *         interactions
     * @see de.fhg.ivi.crowdsimulation.simulation.forcemodel.HelbingModel#getLambda()
     */
    @Override
    public float getLambda()
    {
        return 0.06f;
    }
}
