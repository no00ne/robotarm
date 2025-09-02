import importlib.util, pprint
spec = importlib.util.find_spec("leap_ext")
pprint.pprint(spec)
if spec:
    print("origin:", spec.origin)