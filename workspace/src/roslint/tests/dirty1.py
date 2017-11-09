"""
.. module:: dirty1

This Python module should *not* pass ``pylint`` cleanly.

"""

def another_get(msg, key):
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

# name does not follow PEP-8 convention:
def longCamelCaseName():
    # no doc string
    # line too long:
    return "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"
