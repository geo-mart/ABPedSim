package de.fhg.ivi.crowdsimulation.ui.gui.control.actions.file;

import java.awt.Color;
import java.awt.event.ActionEvent;

import javax.swing.Action;

import de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction;
import jiconfont.icons.FontAwesome;

/**
 * Action, which exits the program.
 *
 * @author hahmann/meinert
 */
public class ExitAction extends AbstractAction
{

    /**
     * default serial version uid
     */
    private static final long serialVersionUID = 1L;

    /**
     * Creates a new {@link ExitAction}, which can be used by the user to exit the program.
     */
    public ExitAction()
    {
        super("Exit", null, FontAwesome.SIGN_OUT, Color.black);
    }

    /**
     * Overrides the same method in {@link AbstractAction} and invokes a specific {@link Action}.
     *
     * @see de.fhg.ivi.crowdsimulation.ui.gui.control.actions.AbstractAction#actionPerformed(ActionEvent)
     */
    @Override
    public void actionPerformed(ActionEvent e)
    {
        System.exit(0);
    }
}
