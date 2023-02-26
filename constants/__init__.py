from json import load



def load_constant():

    with open('../constants/constant.json', 'r') as constants:

        constant = load(constants)

    return constant