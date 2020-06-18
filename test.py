from __future__ import print_function

def run():
    sum = 0.0
    sum = 1.0
    for line in open("test.txt","r"):
        sum = sum + float(line)
        print(sum)

if __name__ == "__main__":
    run()

