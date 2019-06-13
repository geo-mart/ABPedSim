package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.simulation;

import java.awt.event.ActionEvent;

import javax.swing.Action;

import de.fhg.ivi.crowdsimulation.simulation.CrowdSimulator;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import de.fhg.ivi.crowdsimulation.ui.gui.visualisation.MapPanel;
import jiconfont.icons.FontAwesome;

/**
 * Action, which starts the simulation thread and the graphics thread and thereby the pedestrian
 * movement.
 *
 * @author hahmann/meinert
 */
public class StartAction extends AbstractAction
{
    /**
     * default serial version uid
     */
    private static final long   serialVersionUID = 1L;

    /**
     * the name and the short description of this action
     */
    private final static String name             = "Start";

    /**
     * Creates a new {@link StartAction}, which can be used by the user to starts the simulation
     * thread and the graphics thread and thereby the pedestrian movement.
     *
     * @param crowdSimulation object of the main class of the simulation {@link CrowdSimulation}
     */
    public StartAction(CrowdSimulation crowdSimulation)
    {
        super(name, crowdSimulation, FontAwesome.PLAY_CIRCLE_O, greenColor);
        putValue(Action.SHORT_DESCRIPTION, name);
    }

    /**
     * Overrides the same method in {@link AbstractAction} and invokes a specific {@link Action}.
     * <p>
     * This action starts the simulation in dependence of two cases. First in case of a re-start,
     * which happens after a {@link PauseAction}. Second in the case of a fresh start of the
     * simulation.
     *
     * @see java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
     */
    @Override
    public void actionPerformed(ActionEvent e)
    {
        CrowdSimulator crowdSimulator = crowdSimulation.getCrowdSimulator();
        MapPanel mapPanel = crowdSimulation.getPanelManager().getPanel(MapPanel.class);

        // resume to simulation after simulation was paused
        crowdSimulator.resumeSimulation();

        // fresh start of simulation if the simulation is not running
        if ( !crowdSimulator.isSimulationThreadRunning())
        {
            crowdSimulation.startAgentSimulationThread();
        }
        if ( !mapPanel.isGraphicsThreadRunning())
        {
            crowdSimulation.startGraphicsThread();
        }
    }
}
