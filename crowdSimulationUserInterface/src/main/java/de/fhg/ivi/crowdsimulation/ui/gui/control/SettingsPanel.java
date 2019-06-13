package de.fhg.ivi.crowdsimulation.ui.gui.control;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JSeparator;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.NumericIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.RungeKuttaIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SemiImplicitEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.numericintegration.SimpleEulerIntegrator;
import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Crowd;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.tools.MathTools;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.MapPanel;

/**
 * A {@link JPanel} extending class that contains the user interface elements that allow the user to
 * manipulate the simulation itself and its visualization in the {@link MapPanel}.
 * <p>
 * To get an overview about the possibilities in case of the manipulation and visualisation take a
 * look into the right window of the GUI.
 *
 * @author hahmann/meinert
 */
public class SettingsPanel extends JPanel implements ActionListener, ChangeListener
{
    /**
     * Uses the object logger for printing specific messages in the console.
     */
    private static final Logger                        logger                                            = LoggerFactory
        .getLogger(SettingsPanel.class);

    /**
     * default serial version ID
     */
    private static final long                          serialVersionUID                                  = 1L;

    /**
     * thickness of thin separator
     */
    private static final byte                          separatorThin                                     = 1;

    /**
     * thickness of normal separator
     */
    private static final byte                          separatorNormal                                   = 2;

    /**
     * Class for executing the application.
     */
    private CrowdSimulation                            crowdSimulation;

    /**
     * {@link JLabel} which symbolizes the headline for the {@code JCheckbox} part.
     */
    private JLabel                                     headline                                          = new JLabel(
        "Visibilities");

    /**
     * {@link JLabel} which symbolizes the headline for the background visibilities (i.e. WayPoints,
     * Boundaries) {@code JCheckbox} part.
     */
    private JLabel                                     headlineBackGroundVisibilities                    = new JLabel(
        "Background Visibilities");

    /**
     * {@link JCheckBox} which makes the {@code wayPoints} visible or not.
     */
    private JCheckBox                                  checkBoxWayPoints                                 = new JCheckBox(
        "Waypoints");

    /**
     * {@link JCheckBox} which makes the {@code wayPointVerticals} visible or not.
     */
    private JCheckBox                                  checkBoxWayPointVerticals                         = new JCheckBox(
        "Waypoint Verticals");

    /**
     * {@link JCheckBox} which makes the {@code wayPointAxis} visible or not.
     */
    private JCheckBox                                  checkBoxWayPointAxis                              = new JCheckBox(
        "Waypoint Axis");

    /**
     * {@link JCheckBox} which makes the {@code wayPointLabels} visible or not.
     */
    private JCheckBox                                  checkBoxWayPointLabels                            = new JCheckBox(
        "Waypoint Labels");

    /**
     * {@link JCheckBox} which makes the {@code boundaries} visible or not.
     */
    private JCheckBox                                  checkBoxBoundaries                                = new JCheckBox(
        "Boundaries");

    /**
     * {@link JLabel} which symbolizes the headline for the background visibilities (i.e. WayPoints,
     * Boundaries) {@code JCheckbox} part.
     */
    private JLabel                                     headlinePedestrianVisibilities                    = new JLabel(
        "Pedestrian Visibilities");

    /**
     * {@link JCheckBox} which makes the {@code pedestrians} visible or not.
     */
    private JCheckBox                                  checkBoxPedestrians                               = new JCheckBox(
        "Pedestrians");

    /**
     * {@link JCheckBox} which makes the {@code pedestrians} visible or not.
     */
    private JRadioButton                               radioButtonPedestrianDefaultFillMode              = new JRadioButton(
        "Default Fill Mode");

    /**
     * {@link JCheckBox} which makes the {@code pedestrians} visible or not.
     */
    private JRadioButton                               radioButtonPedestrianForcesAndOrientationFillMode = new JRadioButton(
        "Orientation|Forces Fill Mode");

    /**
     * {@link JCheckBox} which makes the {@code pedestrians} visible or not.
     */
    private JRadioButton                               radioButtonPedestrianExtrinsicForcesFillMode      = new JRadioButton(
        "Quantitative Forces Fill Mode");

    /**
     * {@link JCheckBox} which makes the {@code normalizedDirectionVector#Pedestrian} visible or
     * not.
     */
    private JCheckBox                                  checkBoxCurrentVelocityVector                     = new JCheckBox(
        "Velocity vectors (i.e. where the pedestrian actually go)");

    /**
     * {@link JCheckBox} which makes the current target vector (i.e. vector normalize to length of 1
     * to target point) of all {@link Pedestrian} visible or not.
     */
    private JCheckBox                                  checkBoxCurrentTargetVector                       = new JCheckBox(
        "Target vectors (i.e. where the pedestrians want to go)");

    /**
     * {@link JCheckBox} which makes the current target point of all {@link Pedestrian} visible or
     * not.
     */
    private JCheckBox                                  checkBoxCurrentTargetPoint                        = new JCheckBox(
        "Target Points (i.e. target points of the pedestrians)");

    /**
     * {@link JCheckBox} which makes the {@code currentVelocity#Pedestrian} visible or not.
     */
    private JCheckBox                                  checkBoxVelocity                                  = new JCheckBox(
        "Velocities (km/h)");

    /**
     * {@link JCheckBox} which makes the vector of current total extrinsic forces (i.e. forces
     * caused by other {@link Pedestrian} and {@link Boundary}) of all {@link Pedestrian} visible or
     * not.
     */
    private JCheckBox                                  checkBoxCurrentExtrinsicForces                    = new JCheckBox(
        "Extrinsic forces vectors (i.e. forces caused by other pedestrians and boundaries)");

    /**
     * {@link JCheckBox} which makes the vector of forces caused by other {@link Pedestrian} objects
     * of all {@link Pedestrian} visible or not.
     */
    private JCheckBox                                  checkBoxCurrentPedestrianForces                   = new JCheckBox(
        "Pedestrian forces vectors (i.e. forces caused by other pedestrians only)");

    /**
     * {@link JCheckBox} which makes the the vector of current forces caused by {@link Boundary}
     * objects) of all {@link Pedestrian} visible or not.
     */
    private JCheckBox                                  checkBoxCurrentBoundaryForces                     = new JCheckBox(
        "Boundary forces vectors (i.e. forces caused by boundaries only)");

    /**
     * {@link JLabel} which symbolizes the headline for the crowd visibilities (i.e. Crowd Outline)
     * {@code JCheckbox} part.
     */
    private JLabel                                     headlineCrowdVisibilities                         = new JLabel(
        "Crowd Visibilities");

    /**
     * {@link JCheckBox} which makes the {@link CrowdSimulator#getCrowds()} visible or not.
     */
    private JCheckBox                                  checkBoxCrowdOutline                              = new JCheckBox(
        "Crowdoutline");

    /**
     * {@link JCheckBox} which enables/disables clustering for crowd outlines
     */
    private JCheckBox                                  checkBoxClusterCrowdOutline                       = new JCheckBox(
        "Cluster Crowdoutline");

    /**
     * {@link JRadioButton} sets the crowd outlines to use a convex hull algorithm
     */
    private JRadioButton                               radioButtonConvex                                 = new JRadioButton(
        "Convex Hull");

    /**
     * {@link JRadioButton} sets the crowd outlines to use a concave hull algorithm (chi-shape,
     * Duckham et al.)
     */
    private JRadioButton                               radioButtonConcave                                = new JRadioButton(
        "Concave Hull");

    /**
     * {@link JCheckBox} which makes the {@link CrowdSimulator#getGrid()} visible or not.
     */
    private JCheckBox                                  checkBoxGrid                                      = new JCheckBox(
        "Grid (local crowd densities)");

    /**
     * {@link JCheckBox} which makes the {@link CrowdSimulator#getGrid()} visible or not.
     */
    private JCheckBox                                  checkBoxGridLabels                                = new JCheckBox(
        "Grid labels (pedestrians/m²)");

    /**
     * {@link JLabel} which symbolizes the headline for the {@code JSlider} part.
     */
    private JLabel                                     headlineSimulationParameters                      = new JLabel(
        "Simulation Parameters");

    /**
     * {@link JLabel} which symbolizes the headline for the Pedestrian Velocity parameters.
     */
    private JLabel                                     headlinePedestrianVelocities                      = new JLabel(
        "Pedestrian Velocity Parameters");

    /**
     * Contains the set of values, which is used for the {@link #meanNormalDesiredVelocitySlider}
     */
    private static final LinkedHashMap<Integer, Float> desiredVelocityValues;
    static
    {
        desiredVelocityValues = new LinkedHashMap<>();
        desiredVelocityValues.put(0, 0f);
        desiredVelocityValues.put(1, 0.5f);
        desiredVelocityValues.put(2, 1f);
        desiredVelocityValues.put(3, 1.5f);
        desiredVelocityValues.put(4, 2f);
        desiredVelocityValues.put(5, 2.5f);
        desiredVelocityValues.put(6, 3f);
        desiredVelocityValues.put(7, 3.5f);
        desiredVelocityValues.put(8, 4f);
        desiredVelocityValues.put(9, 4.5f);
        desiredVelocityValues.put(10, 5f);
        desiredVelocityValues.put(11, 5.5f);
        desiredVelocityValues.put(12, 6f);
        desiredVelocityValues.put(13, 6.5f);
        desiredVelocityValues.put(14, 7f);
        desiredVelocityValues.put(15, 7.5f);
        desiredVelocityValues.put(16, 8.0f);
    }

    /**
     * {@link JSlider} which manipulates the mean of the normal desired velocities of the
     * {@link Pedestrian}s.
     */
    private JSlider                                    meanNormalDesiredVelocitySlider = new JSlider(
        JSlider.HORIZONTAL, 0, desiredVelocityValues.size() - 1, 8);

    /**
     * {@link JLabel} which symbolizes the name for the {@code meanDesiredVelocitySlider}
     * {@link JSlider}.
     */
    private JLabel                                     meanDesiredVelocityLabel        = new JLabel(
        "Mean desired velocity of all pedestrians [km/h]");

    /**
     * Contains the set of values, which is used for the
     * {@link #standardDeviationOfNormalDesiredVelocitySlider}
     */
    private static final LinkedHashMap<Integer, Float> standardDeviationOfDesiredVelocityValues;
    static
    {
        standardDeviationOfDesiredVelocityValues = new LinkedHashMap<>();
        standardDeviationOfDesiredVelocityValues.put(0, 0f);
        standardDeviationOfDesiredVelocityValues.put(1, 0.5f);
        standardDeviationOfDesiredVelocityValues.put(2, 1f);
        standardDeviationOfDesiredVelocityValues.put(3, 1.5f);
        standardDeviationOfDesiredVelocityValues.put(4, 2f);
        standardDeviationOfDesiredVelocityValues.put(5, 2.5f);
        standardDeviationOfDesiredVelocityValues.put(6, 3f);
    }

    /**
     * {@link JSlider} which manipulates the standard deviation of the the normal desired velocities
     * of the {@link Pedestrian}s.
     */
    private JSlider                                    standardDeviationOfNormalDesiredVelocitySlider = new JSlider(
        JSlider.HORIZONTAL, 0, standardDeviationOfDesiredVelocityValues.size() - 1, 2);

    /**
     * {@link JLabel} which symbolizes the description for the
     * {@code standardDeviationOfDesiredVelocitySlider} {@link JSlider}.
     */
    private JLabel                                     standardDeviationOfDesiredVelocityLabel        = new JLabel(
        "Standard deviation of mean desired velocity of all pedestrians [km/h]");

    /**
     * Contains the set of values, which is used for the {@link #meanMaximumVelocitySlider}
     */
    private static final LinkedHashMap<Integer, Float> maximumVelocityValues;
    static
    {
        maximumVelocityValues = new LinkedHashMap<>();
        maximumVelocityValues.put(0, 0f);
        maximumVelocityValues.put(1, 0.5f);
        maximumVelocityValues.put(2, 1f);
        maximumVelocityValues.put(3, 1.5f);
        maximumVelocityValues.put(4, 2f);
        maximumVelocityValues.put(5, 2.5f);
        maximumVelocityValues.put(6, 3f);
        maximumVelocityValues.put(7, 3.5f);
        maximumVelocityValues.put(8, 4f);
        maximumVelocityValues.put(9, 4.5f);
        maximumVelocityValues.put(10, 5f);
        maximumVelocityValues.put(11, 5.5f);
        maximumVelocityValues.put(12, 6f);
        maximumVelocityValues.put(13, 6.5f);
        maximumVelocityValues.put(14, 7f);
        maximumVelocityValues.put(15, 7.5f);
        maximumVelocityValues.put(16, 8.0f);
    }

    /**
     * {@link JSlider} which manipulates the mean of the {@code maximumDesiredVelocity}.
     */
    private JSlider                                    meanMaximumVelocitySlider = new JSlider(
        JSlider.HORIZONTAL, 0, maximumVelocityValues.size() - 1, 12);

    /**
     * {@link JLabel} which symbolizes the description for the {@code meanMaximalVelocitySlider}
     * {@link JSlider}.
     */
    private JLabel                                     meanMaximumVelocityLabel  = new JLabel(
        "Mean maximum velocity of all pedestrians [km/h]");

    /**
     * Contains the set of values, which is used for the
     * {@link #standardDeviationOfNormalDesiredVelocitySlider}
     */
    private static final LinkedHashMap<Integer, Float> standardDeviationOfMaximumVelocityValues;
    static
    {
        standardDeviationOfMaximumVelocityValues = new LinkedHashMap<>();
        standardDeviationOfMaximumVelocityValues.put(0, 0f);
        standardDeviationOfMaximumVelocityValues.put(1, 0.5f);
        standardDeviationOfMaximumVelocityValues.put(2, 1f);
        standardDeviationOfMaximumVelocityValues.put(3, 1.5f);
        standardDeviationOfMaximumVelocityValues.put(4, 2f);
        standardDeviationOfMaximumVelocityValues.put(5, 2.5f);
        standardDeviationOfMaximumVelocityValues.put(6, 3f);
    }

    /**
     * {@link JSlider} which manipulates the standard deviation of the
     * {@code maximumDesiredVelocity}.
     */
    private JSlider                                    standardDeviationOfMaximumVelocitySlider = new JSlider(
        JSlider.HORIZONTAL, 0, standardDeviationOfMaximumVelocityValues.size() - 1, 2);

    /**
     * {@link JLabel} which symbolizes the description for the
     * {@code standardDeviationOfMaximalVelocitySlider} {@link JSlider}.
     */
    private JLabel                                     standardDeviationOfMaximumVelocityLabel  = new JLabel(
        "Standard deviation of maximum velocity of all pedestrians [km/h]");

    /**
     * Contains the set of values, which is used for the {@link #fastForwardSlider}
     */
    private static final LinkedHashMap<Integer, Float> fastForwardValues;
    static
    {
        fastForwardValues = new LinkedHashMap<>();
        fastForwardValues.put(0, 0f);
        fastForwardValues.put(1, 1f);
        fastForwardValues.put(2, 2f);
        fastForwardValues.put(3, 3f);
        fastForwardValues.put(4, 4f);
        fastForwardValues.put(5, 5f);
    }

    /**
     * {@link JSlider} which manipulates the {@code fastForwardFactor}
     */
    private JSlider      fastForwardSlider                    = new JSlider(JSlider.HORIZONTAL, 0,
        fastForwardValues.size() - 1, 2);

    /**
     * {@link JLabel} which symbolizes the name for the {@code fastForwardSlider} {@link JSlider}.
     */
    private JLabel       fastForwardLabel                     = new JLabel(
        "Simulation speed (1 = real time, >1 = fast forward)");

    /**
     * {@link JLabel} which symbolizes the headline for the Simulation parameters.
     */
    private JLabel       headlineNumericIntegrationParameters = new JLabel("Numeric Integration");

    /**
     * {@link JRadioButton} for setting {@link NumericIntegrator} of {@link CrowdSimulator} to
     * {@link SimpleEulerIntegrator}
     */
    private JRadioButton radioButtonSimpleEuler               = new JRadioButton("Simple Euler");

    /**
     * {@link JRadioButton} for setting {@link NumericIntegrator} of {@link CrowdSimulator} to
     * {@link SemiImplicitEulerIntegrator}
     */
    private JRadioButton radioButtonSemiImplicitEuler         = new JRadioButton(
        "Semi Implicit Euler");

    /**
     * {@link JRadioButton} for setting {@link NumericIntegrator} of {@link CrowdSimulator} to
     * {@link RungeKuttaIntegrator}
     */
    private JRadioButton radioButtonRungeKutta                = new JRadioButton("Runge Kutta");

    /**
     * Constructor. Adds {@link JCheckBox}, {@link JRadioButton} and {@link JSlider} elements to
     * this {@link JPanel}, sets its initial states according to {@link MapPanel} and
     * {@link CrowdSimulator} states and {@link ActionListener}/{@link ChangeListener} to these UI
     * elements.
     *
     * @param cs the reference to main {@link CrowdSimulation} application in order to get the
     *            initial states of {@link MapPanel} and {@link CrowdSimulator}
     */
    public SettingsPanel(CrowdSimulation cs)
    {
        // reference to the main application
        this.crowdSimulation = cs;

        // to avoid warnings about unused logger
        logger.trace("VisualisationSettingsPanel(), " + "just a log");

        // set properties of this JPanel
        setOpaque(false);
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
        setPreferredSize(new Dimension(450, 850));

        // add UI elements
        addElements();

        // configure sliders
        configureSliders();

        // add actionlisteners and changelisteners
        addListeners();

        // sets the initial values of ui elements
        setInitialValues();

    }

    /**
     * Sets the initial values of all UI elements (i.e. all {@link JCheckBox}, {@link JRadioButton}
     * and {@link JSlider} objects) according to the initial states of {@link MapPanel} and
     * {@link CrowdSimulator}.
     */
    private void setInitialValues()
    {
        MapPanel mapPanel = crowdSimulation.getPanelManager().getPanel(MapPanel.class);
        CrowdSimulator crowdSimulator = crowdSimulation.getCrowdSimulator();

        // set initial selection states
        // background
        {
            checkBoxWayPoints.setSelected(mapPanel.isWayPointsVisible());
            checkBoxWayPointVerticals.setSelected(mapPanel.isWayPointVerticalsVisible());
            checkBoxWayPointAxis.setSelected(mapPanel.isWayPointAxisVisible());
            checkBoxWayPointLabels.setSelected(mapPanel.isWayPointLabelsVisible());
            checkBoxBoundaries.setSelected(mapPanel.isBoundariesVisible());
        }

        // pedestrians
        {
            checkBoxPedestrians.setSelected(mapPanel.isPedestriansVisible());
            radioButtonPedestrianDefaultFillMode.setSelected(
                mapPanel.getPedestriansFillMode() == MapPanel.PEDESTRIAN_FILL_MODE_DEFAULT);
            radioButtonPedestrianForcesAndOrientationFillMode.setSelected(mapPanel
                .getPedestriansFillMode() == MapPanel.PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION);
            radioButtonPedestrianExtrinsicForcesFillMode.setSelected(mapPanel
                .getPedestriansFillMode() == MapPanel.PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE);
            checkBoxVelocity.setSelected(mapPanel.isVelocityVisible());
            checkBoxCurrentVelocityVector.setSelected(mapPanel.isCurrentVelocityVectorVisible());
            checkBoxCurrentTargetVector.setSelected(mapPanel.isCurrentTargetVectorVisible());
            checkBoxCurrentTargetPoint.setSelected(mapPanel.isCurrentTargetPointVisible());
            checkBoxCurrentExtrinsicForces.setSelected(mapPanel.isCurrentExtrinsicForcesVisible());
            checkBoxCurrentPedestrianForces.setSelected(mapPanel.isCurrentPedestrianForceVisible());
            checkBoxCurrentBoundaryForces.setSelected(mapPanel.isCurrentBoundaryForceVisible());
        }

        // grid
        {
            checkBoxGrid.setSelected(mapPanel.isGridVisible());
            checkBoxGridLabels.setSelected(mapPanel.isGridLabelsVisible());
        }

        // crowd
        {
            checkBoxCrowdOutline.setSelected(mapPanel.isCrowdOutlineVisible());
            Crowd defaultCrowd = null;
            try
            {
                defaultCrowd = crowdSimulator.getCrowds().get(0);
            }
            catch (IndexOutOfBoundsException e)
            {
                // nothing. defaultCrowd remains null
            }
            // derive default values either from an already loaded crowd or from default values of
            // Crowd class
            boolean defaultIsClustering = defaultCrowd == null
                ? Crowd.defaultIsClusteringCrowdOutlines
                : defaultCrowd.isClusteringCrowdOutlines();
            boolean defaultIsConvex = defaultCrowd == null ? Crowd.defaultIsCrowdOutlineConvex
                : defaultCrowd.isCrowdOutlineConvex();
            float defaultMeanNormalDesiredVelocity = defaultCrowd == null
                ? Crowd.defaultMeanNormalDesiredVelocity
                : defaultCrowd.getMeanNormalDesiredVelocity();
            float defaultStandardDeviationOfNormalDesiredVelocity = defaultCrowd == null
                ? Crowd.defaultStandardDeviationOfNormalDesiredVelocity
                : defaultCrowd.getStandardDeviationOfNormalDesiredVelocity();
            float defaultMeanMaximumDesiredVelocity = defaultCrowd == null
                ? Crowd.defaultMeanMaximumDesiredVelocity
                : defaultCrowd.getMeanMaximumDesiredVelocity();
            float defaultStandardDeviationOfMaximumVelocity = defaultCrowd == null
                ? Crowd.defaultStandardDeviationOfMaximumDesiredVelocity
                : defaultCrowd.getStandardDeviationOfMaximumVelocity();

            checkBoxClusterCrowdOutline.setSelected(defaultIsClustering);
            radioButtonConcave.setSelected( !defaultIsConvex);
            radioButtonConvex.setSelected(defaultIsConvex);

            // simulation params
            // velocities
            meanNormalDesiredVelocitySlider.setValue(getIndex(desiredVelocityValues,
                MathTools.convertMsToKmh(defaultMeanNormalDesiredVelocity)));
            standardDeviationOfNormalDesiredVelocitySlider
                .setValue(getIndex(standardDeviationOfDesiredVelocityValues,
                    MathTools.convertMsToKmh(defaultStandardDeviationOfNormalDesiredVelocity)));
            meanMaximumVelocitySlider.setValue(getIndex(maximumVelocityValues,
                MathTools.convertMsToKmh(defaultMeanMaximumDesiredVelocity)));
            standardDeviationOfMaximumVelocitySlider
                .setValue(getIndex(standardDeviationOfMaximumVelocityValues,
                    MathTools.convertMsToKmh(defaultStandardDeviationOfMaximumVelocity)));
        }

        // numeric integration
        {
            fastForwardSlider.setValue(
                getIndex(fastForwardValues, (float) crowdSimulator.getFastForwardFactor()));
            radioButtonSimpleEuler.setSelected(
                crowdSimulator.getNumericIntegrator() instanceof SimpleEulerIntegrator);
            radioButtonSemiImplicitEuler.setSelected(
                crowdSimulator.getNumericIntegrator() instanceof SemiImplicitEulerIntegrator);
            radioButtonRungeKutta
                .setSelected(crowdSimulator.getNumericIntegrator() instanceof RungeKuttaIntegrator);
        }
    }

    /**
     * Adds {@link ActionListener} to {@link JCheckBox} and {@link JRadioButton} and
     * {@link ChangeListener} to {@link JSlider} objects of this {@link JPanel}
     */
    private void addListeners()
    {
        // add actionListeners to JCheckBox/JRadioButton
        checkBoxWayPoints.addActionListener(this);
        checkBoxWayPointVerticals.addActionListener(this);
        checkBoxWayPointAxis.addActionListener(this);
        checkBoxWayPointLabels.addActionListener(this);
        checkBoxBoundaries.addActionListener(this);
        checkBoxPedestrians.addActionListener(this);
        radioButtonPedestrianDefaultFillMode.addActionListener(this);
        radioButtonPedestrianForcesAndOrientationFillMode.addActionListener(this);
        radioButtonPedestrianExtrinsicForcesFillMode.addActionListener(this);
        checkBoxVelocity.addActionListener(this);
        checkBoxCurrentVelocityVector.addActionListener(this);
        checkBoxCurrentTargetPoint.addActionListener(this);
        checkBoxCurrentTargetVector.addActionListener(this);
        checkBoxCurrentExtrinsicForces.addActionListener(this);
        checkBoxCurrentPedestrianForces.addActionListener(this);
        checkBoxCurrentBoundaryForces.addActionListener(this);
        checkBoxCrowdOutline.addActionListener(this);
        checkBoxClusterCrowdOutline.addActionListener(this);
        radioButtonConcave.addActionListener(this);
        radioButtonConvex.addActionListener(this);
        checkBoxGrid.addActionListener(this);
        checkBoxGridLabels.addActionListener(this);
        radioButtonSimpleEuler.addActionListener(this);
        radioButtonSemiImplicitEuler.addActionListener(this);
        radioButtonRungeKutta.addActionListener(this);

        // add ChangeListener to all JSliders
        meanNormalDesiredVelocitySlider.addChangeListener(this);
        standardDeviationOfNormalDesiredVelocitySlider.addChangeListener(this);
        meanMaximumVelocitySlider.addChangeListener(this);
        standardDeviationOfMaximumVelocitySlider.addChangeListener(this);
        fastForwardSlider.addChangeListener(this);
    }

    /**
     * Calls {@link #sliderSettings(JSlider, HashMap, int)} for {@link JSlider} elements in this
     * {@link JPanel}
     */
    private void configureSliders()
    {
        sliderSettings(meanNormalDesiredVelocitySlider, desiredVelocityValues, 2);
        sliderSettings(standardDeviationOfNormalDesiredVelocitySlider,
            standardDeviationOfDesiredVelocityValues, 2);
        sliderSettings(meanMaximumVelocitySlider, maximumVelocityValues, 2);
        sliderSettings(standardDeviationOfMaximumVelocitySlider,
            standardDeviationOfMaximumVelocityValues, 2);
        sliderSettings(fastForwardSlider, fastForwardValues, 1);
    }

    /**
     * Adds all UI elements (i.e. all {@link JCheckBox}, {@link JRadioButton} and {@link JSlider}
     * objects)
     */
    private void addElements()
    {
        // set Fonts
        Font fontHeadlines = new Font(Font.SANS_SERIF, Font.BOLD, 14);
        Font fontSubHeadlines = new Font(Font.SANS_SERIF, Font.BOLD, 11);
        headline.setFont(fontHeadlines);
        headlineSimulationParameters.setFont(fontHeadlines);
        headlineBackGroundVisibilities.setFont(fontSubHeadlines);
        headlineCrowdVisibilities.setFont(fontSubHeadlines);
        headlinePedestrianVelocities.setFont(fontSubHeadlines);
        headlinePedestrianVisibilities.setFont(fontSubHeadlines);
        headlineNumericIntegrationParameters.setFont(fontSubHeadlines);

        // here starts the slider part

        add(headlineSimulationParameters);
        add(Box.createRigidArea(new Dimension(0, 5)));
        add(Box.createRigidArea(new Dimension(0, 2)));
        addSeparator(separatorThin);
        add(Box.createRigidArea(new Dimension(0, 2)));

        // add sliders and associated labels
        add(headlinePedestrianVelocities);
        add(Box.createRigidArea(new Dimension(0, 10)));
        add(meanDesiredVelocityLabel);
        add(meanNormalDesiredVelocitySlider);
        add(standardDeviationOfDesiredVelocityLabel);
        add(standardDeviationOfNormalDesiredVelocitySlider);
        add(meanMaximumVelocityLabel);
        add(meanMaximumVelocitySlider);
        add(standardDeviationOfMaximumVelocityLabel);
        add(standardDeviationOfMaximumVelocitySlider);
        add(Box.createRigidArea(new Dimension(0, 5)));
        add(Box.createRigidArea(new Dimension(0, 2)));
        addSeparator(separatorThin);
        add(Box.createRigidArea(new Dimension(0, 2)));
        add(headlineNumericIntegrationParameters);
        add(Box.createRigidArea(new Dimension(0, 10)));
        add(fastForwardLabel);
        add(fastForwardSlider);

        // radio buttons for numeric integrators
        ButtonGroup numericIntegratorChooser = new ButtonGroup();
        numericIntegratorChooser.add(radioButtonSimpleEuler);
        numericIntegratorChooser.add(radioButtonSemiImplicitEuler);
        numericIntegratorChooser.add(radioButtonRungeKutta);
        JPanel numericIntegratorsPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
        numericIntegratorsPanel.add(radioButtonSimpleEuler);
        numericIntegratorsPanel.add(radioButtonSemiImplicitEuler);
        numericIntegratorsPanel.add(radioButtonRungeKutta);
        numericIntegratorsPanel.setMaximumSize(new Dimension(300, 30));
        numericIntegratorsPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(numericIntegratorsPanel);

        // add headline
        addSeparator(separatorNormal);
        add(Box.createRigidArea(new Dimension(0, 10)));
        add(headline);
        // add elements
        // background
        // add(Box.createRigidArea(new Dimension(0, 2)));
        // addSeparator(separatorThin);
        // add(Box.createRigidArea(new Dimension(0, 2)));
        // add(headlineBackGroundVisibilities);
        // add(Box.createRigidArea(new Dimension(0, 5)));
        // JPanel backgroundPanel = new JPanel(new GridLayout(2, 3));
        // backgroundPanel.add(checkBoxWayPoints);
        // backgroundPanel.add(checkBoxWayPointVerticals);
        // backgroundPanel.add(checkBoxWayPointAxis);
        // backgroundPanel.add(checkBoxWayPointLabels);
        // backgroundPanel.add(checkBoxBoundaries);
        // backgroundPanel.setMaximumSize(new Dimension(350, 45));
        // backgroundPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        // add(backgroundPanel);
        add(Box.createRigidArea(new Dimension(0, 2)));
        addSeparator(separatorThin);
        add(Box.createRigidArea(new Dimension(0, 2)));

        // pedestrian
        add(headlinePedestrianVisibilities);
        add(Box.createRigidArea(new Dimension(0, 5)));
        // radio buttons for numeric integrators
        ButtonGroup fillModeChooser = new ButtonGroup();
        fillModeChooser.add(radioButtonPedestrianDefaultFillMode);
        fillModeChooser.add(radioButtonPedestrianForcesAndOrientationFillMode);
        fillModeChooser.add(radioButtonPedestrianExtrinsicForcesFillMode);
        JPanel fillModesPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, -5));
        fillModesPanel.add(checkBoxPedestrians);
        fillModesPanel.add(radioButtonPedestrianDefaultFillMode);
        fillModesPanel.add(radioButtonPedestrianForcesAndOrientationFillMode);
        fillModesPanel.add(radioButtonPedestrianExtrinsicForcesFillMode);
        fillModesPanel.setMaximumSize(new Dimension(325, 35));
        fillModesPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(fillModesPanel);
        add(checkBoxVelocity);
        add(checkBoxCurrentVelocityVector);
        add(checkBoxCurrentTargetVector);
        add(checkBoxCurrentTargetPoint);
        add(checkBoxCurrentExtrinsicForces);
        add(checkBoxCurrentPedestrianForces);
        add(checkBoxCurrentBoundaryForces);
        add(Box.createRigidArea(new Dimension(0, 2)));
        addSeparator(separatorThin);
        add(Box.createRigidArea(new Dimension(0, 2)));

        // crowd
        add(headlineCrowdVisibilities);
        add(Box.createRigidArea(new Dimension(0, 5)));
        JPanel crowdPanel = new JPanel(new GridLayout(3, 2));
        crowdPanel.add(checkBoxCrowdOutline);
        // checkBoxCluster Crowd Outlines
        crowdPanel.add(checkBoxClusterCrowdOutline);
        // radio button convex hull vs. concave hull
        ButtonGroup crowdOutlineChooser = new ButtonGroup();
        crowdOutlineChooser.add(radioButtonConcave);
        crowdOutlineChooser.add(radioButtonConvex);
        JPanel crowdOutlineButtonsPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
        crowdOutlineButtonsPanel.add(radioButtonConcave);
        crowdOutlineButtonsPanel.add(radioButtonConvex);
        crowdPanel.add(crowdOutlineButtonsPanel);
        crowdPanel.add(checkBoxGrid);
        crowdPanel.add(checkBoxGridLabels);
        crowdPanel.setMaximumSize(new Dimension(400, 65));
        crowdPanel.setAlignmentX(Component.LEFT_ALIGNMENT);
        add(crowdPanel);

    }

    /**
     * Contains a specific set of values, which is used for {@link JSlider} components.
     *
     * @param values name and values of the specific {@link LinkedHashMap}, which could be used.
     * @param value of the {@link JSlider}, which should be used.
     *
     * @return a {@link Integer} value, which is set to be the new number of a {@link JSlider}.
     */
    private static int getIndex(LinkedHashMap<Integer, Float> values, float value)
    {
        float difference = Float.MAX_VALUE;
        int index = 0;
        int i = 0;

        // look for an appropriate class value depending on the value
        for (Map.Entry<Integer, Float> entry : values.entrySet())
        {
            float tempDifference = Math.abs(entry.getValue() - value);
            if (tempDifference < difference)
            {
                index = i;
                difference = tempDifference;
            }
            i++ ;
        }
        return index;
    }

    /**
     * Determines which action is performed if an element, like {@link JButton} or
     * {@link JCheckBox}, will be manipulated.
     *
     * @param actionEvent object is the {@link ActionEvent} which occurs if a {@link JButton} or so
     *            is pressed.
     *
     * @see java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
     */
    @Override
    public void actionPerformed(ActionEvent actionEvent)
    {
        MapPanel mapPanel = crowdSimulation.getPanelManager().getPanel(MapPanel.class);
        CrowdSimulator crowdSimulator = crowdSimulation.getCrowdSimulator();

        if (actionEvent.getSource() == checkBoxWayPoints)
        {
            mapPanel.setWayPointsVisible(checkBoxWayPoints.isSelected());
        }

        if (actionEvent.getSource() == checkBoxWayPointVerticals)
        {
            mapPanel.setWayPointVerticalsVisible(checkBoxWayPointVerticals.isSelected());
        }

        if (actionEvent.getSource() == checkBoxWayPointAxis)
        {
            mapPanel.setWayPointAxisVisible(checkBoxWayPointAxis.isSelected());
        }

        if (actionEvent.getSource() == checkBoxWayPointLabels)
        {
            mapPanel.setWayPointLabelsVisible(checkBoxWayPointLabels.isSelected());
        }

        if (actionEvent.getSource() == checkBoxBoundaries)
        {
            mapPanel.setBoundariesVisible(checkBoxBoundaries.isSelected());
        }

        if (actionEvent.getSource() == checkBoxPedestrians)
        {
            mapPanel.setPedestriansVisible(checkBoxPedestrians.isSelected());
        }

        if (actionEvent.getSource() == radioButtonPedestrianDefaultFillMode)
        {
            mapPanel.setPedestriansFillMode(MapPanel.PEDESTRIAN_FILL_MODE_DEFAULT);
        }

        if (actionEvent.getSource() == radioButtonPedestrianForcesAndOrientationFillMode)
        {
            mapPanel.setPedestriansFillMode(MapPanel.PEDESTRIAN_FILL_MODE_FORCES_AND_ORIENTATION);
        }

        if (actionEvent.getSource() == radioButtonPedestrianExtrinsicForcesFillMode)
        {
            mapPanel.setPedestriansFillMode(
                MapPanel.PEDESTRIAN_FILL_MODE_EXTRINSIC_FORCES_QUANTITATIVE);
        }

        if (actionEvent.getSource() == checkBoxVelocity)
        {
            mapPanel.setVelocityVisible(checkBoxVelocity.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentVelocityVector)
        {
            mapPanel.setCurrentVelocityVectorVisible(checkBoxCurrentVelocityVector.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentTargetPoint)
        {
            mapPanel.setCurrentTargetPointVisible(checkBoxCurrentTargetPoint.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentTargetVector)
        {
            mapPanel.setCurrentTargetVectorVisible(checkBoxCurrentTargetVector.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentExtrinsicForces)
        {
            mapPanel.setCurrentExtrinsicForcesVisible(checkBoxCurrentExtrinsicForces.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentPedestrianForces)
        {
            mapPanel.setCurrentPedestrianForceVisible(checkBoxCurrentPedestrianForces.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCurrentBoundaryForces)
        {
            mapPanel.setCurrentBoundaryForceVisible(checkBoxCurrentBoundaryForces.isSelected());
        }

        if (actionEvent.getSource() == checkBoxCrowdOutline)
        {
            mapPanel.setCrowdOutlineVisible(checkBoxCrowdOutline.isSelected());
        }

        if (actionEvent.getSource() == checkBoxClusterCrowdOutline)
        {
            for (Crowd crowd : crowdSimulator.getCrowds())
            {
                crowd.setClusteringCrowdOutlines(checkBoxClusterCrowdOutline.isSelected());
            }
        }

        if (actionEvent.getSource() == radioButtonConcave)
        {
            for (Crowd crowd : crowdSimulator.getCrowds())
            {
                crowd.setCrowdOutlineConvex(false);
            }
        }

        if (actionEvent.getSource() == radioButtonConvex)
        {
            for (Crowd crowd : crowdSimulator.getCrowds())
            {
                crowd.setCrowdOutlineConvex(true);
            }
        }

        if (actionEvent.getSource() == checkBoxGrid)
        {
            mapPanel.setGridVisible(checkBoxGrid.isSelected());
        }

        if (actionEvent.getSource() == checkBoxGridLabels)
        {
            mapPanel.setGridLabelsVisible(checkBoxGridLabels.isSelected());
        }

        if (actionEvent.getSource() == radioButtonSimpleEuler)
        {
            crowdSimulator.setNumericIntegrator(new SimpleEulerIntegrator());
        }

        if (actionEvent.getSource() == radioButtonSemiImplicitEuler)
        {
            crowdSimulator.setNumericIntegrator(new SemiImplicitEulerIntegrator());
        }

        if (actionEvent.getSource() == radioButtonRungeKutta)
        {
            crowdSimulator.setNumericIntegrator(new RungeKuttaIntegrator());
        }
        mapPanel.repaint();
    }

    /**
     * Changes the value of an object if a {@link JSlider} will be manipulated.
     *
     * @param changeEvent is the {@link ChangeEvent} which occurs if a {@link JSlider} or so is
     *            changed.
     *
     * @see javax.swing.event.ChangeListener#stateChanged(javax.swing.event.ChangeEvent)
     */
    @Override
    public void stateChanged(ChangeEvent changeEvent)
    {
        CrowdSimulator crowdSimulator = crowdSimulation.getCrowdSimulator();

        if (changeEvent.getSource() == meanNormalDesiredVelocitySlider)
        {
            if ( !meanNormalDesiredVelocitySlider.getValueIsAdjusting())
            {
                for (Crowd crowd : crowdSimulator.getCrowds())
                {
                    float desiredVelocityKmh = desiredVelocityValues
                        .get(meanNormalDesiredVelocitySlider.getValue());
                    float desiredVelocityMs = MathTools.convertKmhToMs(desiredVelocityKmh);
                    crowd.setMeanNormalDesiredVelocity(desiredVelocityMs);
                }
            }
        }

        if (changeEvent.getSource() == standardDeviationOfNormalDesiredVelocitySlider)
        {
            if ( !standardDeviationOfNormalDesiredVelocitySlider.getValueIsAdjusting())
            {
                for (Crowd crowd : crowdSimulator.getCrowds())
                {
                    float standardDeviationOfNormalDesiredVelocityKmh = standardDeviationOfDesiredVelocityValues
                        .get(standardDeviationOfNormalDesiredVelocitySlider.getValue());
                    float standardDeviationOfNormalDesiredVelocityMs = MathTools
                        .convertKmhToMs(standardDeviationOfNormalDesiredVelocityKmh);
                    crowd.setStandardDeviationOfNormalDesiredVelocity(
                        standardDeviationOfNormalDesiredVelocityMs);
                }
            }
        }

        if (changeEvent.getSource() == meanMaximumVelocitySlider)
        {
            if ( !meanMaximumVelocitySlider.getValueIsAdjusting())
            {
                for (Crowd crowd : crowdSimulator.getCrowds())
                {
                    float maximumVelocityKmh = maximumVelocityValues
                        .get(meanMaximumVelocitySlider.getValue());
                    float maxiumVelocityMs = MathTools.convertKmhToMs(maximumVelocityKmh);
                    crowd.setMeanMaximumDesiredVelocity(maxiumVelocityMs);
                }
            }
        }

        if (changeEvent.getSource() == standardDeviationOfMaximumVelocitySlider)
        {
            if ( !standardDeviationOfMaximumVelocitySlider.getValueIsAdjusting())
            {
                for (Crowd crowd : crowdSimulator.getCrowds())
                {
                    float standardDeviationOfMaximumDesiredVelocityKmh = standardDeviationOfMaximumVelocityValues
                        .get(standardDeviationOfMaximumVelocitySlider.getValue());
                    float standardDeviationOfMaximumDesiredVelocityMs = MathTools
                        .convertKmhToMs(standardDeviationOfMaximumDesiredVelocityKmh);
                    crowd.setStandardDeviationOfMaximumVelocity(
                        standardDeviationOfMaximumDesiredVelocityMs);
                }
            }
        }

        if (changeEvent.getSource() == fastForwardSlider)
        {
            int fastForwardFactor = fastForwardValues.get(fastForwardSlider.getValue()).intValue();
            crowdSimulator.setFastForwardFactor(fastForwardFactor);
        }
    }

    /**
     * Adds a {@link JSeparator} to this {@link JPanel}. Can either be thin
     * {@link SettingsPanel#separatorThin} or normal {@link SettingsPanel#separatorNormal}
     *
     * @param thickness the thickness of the separator. Can either be thin
     *            {@link SettingsPanel#separatorThin} or normal
     *            {@link SettingsPanel#separatorNormal}
     */
    private void addSeparator(byte thickness)
    {
        JSeparator seperator = new JSeparator();
        seperator.setMaximumSize(new Dimension(Integer.MAX_VALUE, thickness));
        add(seperator);
    }

    /**
     * Configures the {@link JSlider} {@code slider}. Sets the its possible values according to
     * {@code entries} and the major tick spacing according to {@code majorTickSpacing}
     *
     * @param slider the slider to be configured
     * @param entries the possible values of the slider
     * @param majorTickSpacing the major tick spacing
     */
    private void sliderSettings(JSlider slider, HashMap<Integer, Float> entries,
        int majorTickSpacing)
    {
        Hashtable<Integer, JLabel> labelTable = new Hashtable<>();
        for (Map.Entry<Integer, Float> entry : entries.entrySet())
        {
            if (entry.getKey() % majorTickSpacing == 0)
                labelTable.put(entry.getKey(), new JLabel(entry.getValue().toString()));
        }
        slider.setLabelTable(labelTable);
        slider.setSize(this.getWidth() - 20, 10);
        slider.setMajorTickSpacing(majorTickSpacing);
        slider.setMinorTickSpacing(1);
        slider.setPaintTicks(true);
        slider.setSnapToTicks(false);
        slider.setPaintLabels(true);
        slider.setMaximumSize(new Dimension(600, 50));
    }
}
