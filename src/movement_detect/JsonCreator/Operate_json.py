
import json

jsonFile = open("data.json", "w")

for i in range(10):
    aDict = {"a":54, "b":i}
    jsonString = json.dumps(aDict)
    jsonFile.write(jsonString)

jsonFile.close()