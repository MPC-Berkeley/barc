"""
.. module:: clean1

This Python module should pass ``pylint`` cleanly.

"""

def get(msg, key):
    """ Get property value.

    :param msg: Message containing properties.
    :param key: Property key to match.

    :returns: Corresponding value, if defined; None otherwise.
              Beware: the value may be '', which evaluates False as a
              predicate, use ``is not None`` to test for presence.
    """
    for prop in msg.props:
        if prop.key == key:
            return prop.value
    return None
