import functools

ALL = "all"

def context(contexts):
    """Needs to have exactly one argument, with no keyword,
    consisting of either contexts.ALL or a list of strings.
    """
    def deco(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        return wrapper
    return deco
