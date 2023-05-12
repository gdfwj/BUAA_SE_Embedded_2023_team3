import re
message = 'map/save/:1123'
result = re.search("[0-9]+", message)
print(result.group())