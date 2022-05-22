import pickle

l = lambda a, b, c :  a**b - c


print(l(2, 4, 1))

tpl = (1, 't')
print(type(tpl))

lst = [1, 't']
print(type(lst))

st = {43, 'tt'}
print(type(st))


print(pickle.dump(tpl))