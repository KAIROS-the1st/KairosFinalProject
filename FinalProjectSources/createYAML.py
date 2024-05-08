import yaml

data = {
    "train" : r"C:\elevator\train",
        "val" : r"C:\elevator\valid",
        "test" : r"C:\elevator\test", 
        "names" : {0 : "open", 1 : 'close'}}

with open('./ev.yaml', 'w') as f :
    yaml.dump(data, f)

# check written file
with open('./ev.yaml', 'r') as f :
    lines = yaml.safe_load(f)
    print(lines)