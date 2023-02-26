from json import load



def read_constant(keyword : str) -> dict:
    """
    this function allows you to read constants you want

    1. gain    -> for PD loop gains
        "Kp", "Kd"
    2. gravity -> gravity constant 
        "g"
    3. thrust  -> thrust constant and P gain
        "alpha", "Kp
    """

    _path = '../constants/{}'.format(keyword)

    with open(_path, 'r') as constants:

        constant = load(constants)

    return constant


def _be_constant(self, name, value):

    if name in self.__dict__:
        raise Exception("you can not chance const variable")