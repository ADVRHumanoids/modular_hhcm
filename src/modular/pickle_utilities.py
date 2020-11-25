import pickle


def load_pickle(filename):
    # 'list_of_candidates.pkl'
    obj_list = []
    with open(filename, 'rb') as input:
        while True:
            try:
                obj_list.append(pickle.load(input))
            except EOFError:
                break

    return obj_list


def dump_pickle(filename, obj_list):
    # 'list_of_candidates.pkl'
    with open(filename, 'wb') as output:
        for c in obj_list:
            pickle.dump(c, output, pickle.HIGHEST_PROTOCOL)