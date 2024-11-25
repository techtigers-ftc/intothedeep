package team.techtigers.core.display;

/**
 * An abstract class for a view on the LEDs
 */
public abstract class DisplayView {
    private final DisplayRegion[] regions;

    /**
     * Creates a new view
     *
     * @param regions the regions that make up the view
     */
    protected DisplayView(DisplayRegion[] regions) {
        this.regions = regions;
    }

    /**
     * @return the regions that make up the view
     */
    public DisplayRegion[] getRegions() {
        return this.regions;
    }

}
