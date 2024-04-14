def dict_index(d, target_value):
    return [key for key, value in d.items() if value == target_value]

def dict_reverse(d):
    return {v: i for i,v in d.items()}