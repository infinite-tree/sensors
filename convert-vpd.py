#! /usr/bin/python

import csv
import os
import sys


VPD_CSV = "vpd.csv"
VPD_HEADER = "src/vpd.h"

def main():
    data = []
    row_count = 0
    column_count = 0
    print("Reading CSV...")
    with open(VPD_CSV, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            r = []
            for cell in row:
                try:
                    r.append(float(cell))
                except:
                    print("Format Error for value: %s "%cell)
                    sys.exit(1)
            data.append(r)
            row_count += 1
    column_count = len(data[0])

    print("CSV Parsed.\nGenerating header file...")
    with open(VPD_HEADER, 'w') as f:
        f.write("#ifndef VPD_H\n")
        f.write("#define VPD_H\n")
        f.write("\n")
        f.write("int VPD_ROWS = %d;\n"%row_count)
        f.write("int VPD_COLUMNS = %d;\n"%column_count)
        f.write("float PROGMEM VPD_TABLE [%d][%d] = {\n"%(row_count, column_count))

        contents = []
        for row in data:
            contents.append("  { %s }"%(", ".join([str(c) for c in row])))
        f.write(",\n".join(contents))
        f.write("\n")
        f.write("};\n")
        f.write("\n")
        f.write("#endif\n")
    
    print("Done generating %s"%VPD_HEADER)


if __name__ == "__main__":
    main()