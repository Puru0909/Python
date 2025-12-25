from functools import reduce

l = [1, 2, 34234, 53, 6234235, 64343, 65, 754, 45, 55]

def greater(a, b):
    if a > b:
        return a
    return b

print(reduce(greater, l))
   