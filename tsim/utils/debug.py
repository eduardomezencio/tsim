"""Functions for debugging."""

from itertools import product


def dprint(value):
    """Print a value and return it."""
    print(f'>>>>> {value}')
    return value


def dproduct(itera, iterb):
    """Equivalent to product, printing all values."""
    for pair in product(itera, iterb):
        print(pair)
        yield pair
