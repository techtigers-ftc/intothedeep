package team.techtigers.core.paths.routeplanning;


import team.techtigers.core.paths.geometry.Rectangle;

/**
 * A node that represents a tile on the field
 */
public class FieldNode extends Node<Rectangle> {

    /**
     * Construct a Node using a rectangle
     *
     * @param value This is a rectangle which describes the bounds for a tile on the field
     */
    public FieldNode(Rectangle value) {
        super(value);
    }
}
