import csv
import numpy as np

data = csv.reader(open('usethis.csv', 'rt'), quoting=csv.QUOTE_NONNUMERIC)
column1 = []

for row in data:
    column1.append(row[0])
    
    
print(column1)

