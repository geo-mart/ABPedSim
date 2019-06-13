package de.fhg.ivi.crowdsimulation.ui.gui.control;

import javax.swing.Action;
import javax.swing.JToolBar;

import de.fhg.ivi.crowdsimulation.simulation.objects.Boundary;
import de.fhg.ivi.crowdsimulation.simulation.objects.Pedestrian;
import de.fhg.ivi.crowdsimulation.simulation.objects.WayPoint;
import de.fhg.ivi.crowdsimulation.ui.CrowdSimulation;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.file.ExitAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.file.RestartAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.file.StopClearAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io.BoundaryLoadAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io.CrowdLoadAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io.ExportCurrentPedsJSONAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io.ExportJSONAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.io.WayPointLoadAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.simulation.PauseAction;
import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.simulation.StartAction;
import jiconfont.icons.FontAwesome;
import jiconfont.swing.IconFontSwing;

/**
 * This class contains a set of {@link Action}s that can be performed by the user after the
 * {@link CrowdSimulation} has been started. This includes starting, pausing, stopping and resetting
 * the simulation as well as loading alternative {@link Boundary}, {@link Pedestrian} and
 * {@link WayPoint} data.
 * <p>
 * Hint: This class contains the same actions like the class {@link MenuBar}.
 *
 * @author hahmann/meinert
 */
public class Toolbar extends JToolBar
{

    /**
     * default serial version uid
     */
    private static final long serialVersionUID = 1L;

    /**
     * Adds the functionality of all {@link Action}s from the actions package to the
     * {@link Toolbar}.
     *
     * @param crowdSimulation object of the main class of the simulation {@link CrowdSimulation}
     */
    public Toolbar(CrowdSimulation crowdSimulation)
    {
        // register font awesome font for icons
        IconFontSwing.register(FontAwesome.getIconFont());

        add(new RestartAction(crowdSimulation));
        add(new StopClearAction(crowdSimulation));
        add(new ExitAction());
        addSeparator();
        add(new CrowdLoadAction(crowdSimulation));
        add(new BoundaryLoadAction(crowdSimulation));
        add(new WayPointLoadAction(crowdSimulation));
        addSeparator();
        add(new ExportJSONAction(crowdSimulation));
        add(new ExportCurrentPedsJSONAction(crowdSimulation));
        addSeparator();
        add(new StartAction(crowdSimulation));
        add(new PauseAction(crowdSimulation));
    }
}
